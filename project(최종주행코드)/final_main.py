# final_main.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, time, threading, socket, struct, pickle, json
import cv2, afb
from dotenv import load_dotenv

from modules.steering_module import SteeringAnalyzer
from modules.fsm_module import LaneFSM
from modules.drive_module import (
    get_current_mode, get_current_speed, BASE_ANGLE
)

load_dotenv()

# ──────────────────────────────────────────
# 설정
# ──────────────────────────────────────────
WIDTH, HEIGHT, FPS = 640, 480, 30
JPEG_QUALITY = 85
SEND_FPS = 1

LAPTOP_IP   = os.environ["LAPTOP_IP"]
LAPTOP_PORT = int(os.environ["LAPTOP_PORT"])

PI_LISTEN_HOST = os.environ["PI_LISTEN_HOST"]
PI_LISTEN_PORT = int(os.environ["PI_LISTEN_PORT"])

PRINT_DEBUG = True   # 터미널 디버그 출력 on/off

# ──────────────────────────────────────────
# 전역
# ──────────────────────────────────────────
latest_frame = None
frame_lock = threading.Lock()
selected_label = None
key_start_flag = False

analyzer = SteeringAnalyzer(
    base_angle= 85,
    max_offset=48.5,
    angle_limits=(30.0, 150.0),
    y_bottom_ratio=0.70,
    y_top_ratio=0.40,
    k_head=0.60
)
fsm = LaneFSM()

# ──────────────────────────────────────────
# 유틸: 모드 → 서보 목표각 추정
# ──────────────────────────────────────────
def compute_servo_target(mode: str, angle: float | None) -> float:
    """
    drive_module 동작 패턴을 반영해 현재 모드에서 실제로 나갈 서보 목표각을 추정.
    - FORWARD/BACK/STOP: BASE_ANGLE
    - LEFT : 30
    - RIGHT: 150
    - LANE_ADJ_LEFT/RIGHT: angle (없으면 BASE_ANGLE)
    """
    m = (mode or "").upper()
    if m == "LEFT":
        return 30.0
    if m == "RIGHT":
        return 150.0
    if m in ("LANE_ADJ_LEFT", "LANE_ADJ_RIGHT"):
        return float(angle) if angle is not None else float(BASE_ANGLE)
    # FORWARD/BACK/STOP/기타
    return float(BASE_ANGLE)

# ──────────────────────────────────────────
# 카메라 캡처
# ──────────────────────────────────────────
def camera_thread():
    global latest_frame
    while True:
        f = afb.camera.get_image()
        if f is not None:
            with frame_lock:
                latest_frame = f
        time.sleep(0.0)

# ──────────────────────────────────────────
# 노트북으로 프레임 송신
# ──────────────────────────────────────────
def sender_thread():
    sock = None
    while True:
        try:
            if sock is None:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.connect((LAPTOP_IP, LAPTOP_PORT))
                sock = s
                if PRINT_DEBUG:
                    print(f"[TX] connected to {LAPTOP_IP}:{LAPTOP_PORT}")
            with frame_lock:
                f = latest_frame
            if f is None:
                time.sleep(0.05); continue
            ok, jpg = cv2.imencode(".jpg", f, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ok:
                time.sleep(1.0/max(1,SEND_FPS)); continue
            data = pickle.dumps(jpg, protocol=4)
            sock.sendall(struct.pack(">L", len(data)) + data)
            time.sleep(1.0/max(1,SEND_FPS))
        except (ConnectionError, OSError):
            try:
                if sock: sock.close()
            except: pass
            sock = None
            if PRINT_DEBUG:
                print("[TX] reconnecting in 2s ...")
            time.sleep(2)

# ──────────────────────────────────────────
# 노트북에서 라벨 수신
# ──────────────────────────────────────────
def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk: raise ConnectionError("closed")
        buf += chunk
    return buf

def receiver_thread():
    global selected_label
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((PI_LISTEN_HOST, PI_LISTEN_PORT))
    srv.listen(1)
    if PRINT_DEBUG:
        print(f"[RX] listening on {PI_LISTEN_HOST}:{PI_LISTEN_PORT}")
    while True:
        try:
            conn, addr = srv.accept()
            if PRINT_DEBUG:
                print(f"[RX] connected from {addr}")
            with conn:
                while True:
                    size = struct.unpack(">L", recv_exact(conn, 4))[0]
                    blob = recv_exact(conn, size)
                    msg = json.loads(blob.decode("utf-8"))
                    selected_label = msg.get("selected")
                    if PRINT_DEBUG:
                        print(f"[RX] label={selected_label} raw={msg}")
        except Exception as e:
            if PRINT_DEBUG:
                print(f"[RX] error: {e}")
            time.sleep(0.5)

# ──────────────────────────────────────────
# 키 입력(시작 트리거)
# ──────────────────────────────────────────
def key_thread():
    global key_start_flag
    try:
        while True:
            ch = input().strip().lower()
            if ch == "s":
                key_start_flag = True
                if PRINT_DEBUG:
                    print("[KEY] start trigger ON")
    except Exception:
        pass

# ──────────────────────────────────────────
# 주행 루프 (터미널 디버그 출력)
# ──────────────────────────────────────────
def drive_loop():
    global key_start_flag
    last_log_t = 0.0
    while True:
        with frame_lock:
            f = latest_frame
        if f is None:
            time.sleep(0.01); continue

        result = analyzer.analyze_frame(f)
        angle = result["angle"]

        # FSM 업데이트
        fsm.update(angle, selected_label, key_start_flag)

        # 1회성 키 트리거 소모
        key_start_flag = False

        # 디버그 출력 (FPS와 무관하게 100ms 간격으로 제한)
        now = time.time()
        if PRINT_DEBUG and (now - last_log_t) >= 0.10:
            mode = get_current_mode()
            speed = get_current_speed()
            servo = compute_servo_target(mode, angle)
            # LaneFSM에 get_state()가 없으니 내부 필드 직접 조회
            try:
                state_val = getattr(fsm, "state", "?")
            except Exception:
                state_val = "?"
            print(f"[DBG] FSM:{state_val} | MODE:{mode} | SPEED:{speed} | SERVO:{servo:.1f} | ANGLE:{angle:.1f} | LABEL:{selected_label}")
            last_log_t = now

        time.sleep(1.0 / max(1, FPS))

def main():
    afb.gpio.init()
    afb.camera.init(WIDTH, HEIGHT, FPS)

    threading.Thread(target=camera_thread, daemon=True).start()
    threading.Thread(target=sender_thread, daemon=True).start()
    threading.Thread(target=receiver_thread, daemon=True).start()
    threading.Thread(target=key_thread, daemon=True).start()

    try:
        drive_loop()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
