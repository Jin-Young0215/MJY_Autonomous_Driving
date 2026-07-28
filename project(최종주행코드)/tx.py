#!/usr/bin/env python3
import os
import socket
import struct
import pickle
import json
import time
import cv2
from dotenv import load_dotenv
from ultralytics import YOLO

load_dotenv()

# ──────────────────────────────────────────
# 설정
# ──────────────────────────────────────────
FRAME_LISTEN_HOST = os.environ["FRAME_LISTEN_HOST"]
FRAME_LISTEN_PORT = int(os.environ["FRAME_LISTEN_PORT"])

PI_RESULT_HOST    = os.environ["PI_RESULT_HOST"]
PI_RESULT_PORT    = int(os.environ["PI_RESULT_PORT"])

MODEL_PATH        = os.environ["MODEL_PATH"]
CONF_TH           = 0.25
IMG_SIZE          = 640

CLASS_RULES = {
    "gateclosed": {"conf_min": 0.20},
    "gateopen":   {"conf_min": 0.30},
    "redlight":   {"conf_min": 0.40},
    "greensign":  {"conf_min": 0.40},
    "rightsign":  {"w_min": 43,  "conf_min": 0.40},
    "leftsign":   {"w_min": 37,  "conf_min": 0.40},
    "smallhill":  {"w_min": 240, "conf_min": 0.40},
    "staticb":    {"w_min": 65,  "conf_min": 0.40},
    "staticy":    {"w_min": 45,  "conf_min": 0.40},
}

PRIORITY = {
    "smallhill": 100,
    "rightsign": 80,
    "gateclosed": 70,
    "gateopen": 60,
    "staticb": 60,
    "staticy": 50,
    "redlight": 45,
    "greensign": 40,
    "leftsign": 30,
}

def send_with_size(sock, payload: bytes):
    sock.sendall(struct.pack(">L", len(payload)) + payload)

def recv_exact(sock, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf += chunk
    return buf

def main():
    print("[NB] Loading YOLO...")
    model = YOLO(MODEL_PATH)
    names = model.model.names if hasattr(model.model, "names") else model.names
    print("[NB] YOLO ready.")

    print(f"[NB] Connecting to Pi {PI_RESULT_HOST}:{PI_RESULT_PORT} ...")
    pi_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pi_sock.connect((PI_RESULT_HOST, PI_RESULT_PORT))
    print("[NB] Connected to Pi result receiver.")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((FRAME_LISTEN_HOST, FRAME_LISTEN_PORT))
    srv.listen(1)
    print(f"[NB] Waiting frame stream on {FRAME_LISTEN_HOST}:{FRAME_LISTEN_PORT} ...")
    conn, addr = srv.accept()
    print(f"[NB] Frame stream connected from {addr}")

    payload_size = struct.calcsize(">L")
    buffer = b""
    frame_count = 0  # 프레임 카운터

    try:
        while True:
            while len(buffer) < payload_size:
                chunk = conn.recv(4096)
                if not chunk:
                    raise ConnectionError("frame stream closed")
                buffer += chunk
            packed = buffer[:payload_size]
            buffer = buffer[payload_size:]
            (msg_size,) = struct.unpack(">L", packed)

            while len(buffer) < msg_size:
                chunk = conn.recv(4096)
                if not chunk:
                    raise ConnectionError("frame stream closed during data")
                buffer += chunk
            frame_blob = buffer[:msg_size]
            buffer = buffer[msg_size:]

            jpg = pickle.loads(frame_blob)
            frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
            
            # BGR을 RGB로 변경하는 코드
            
            frame_count += 1

            # 수신 로그
            print(f"[NB] Frame #{frame_count} received - {msg_size} bytes, "
                  f"Resolution: {frame.shape[1]}x{frame.shape[0]}")

            results = model.predict(frame, conf=CONF_TH, imgsz=IMG_SIZE, verbose=False)
            r = results[0]
            boxes = r.boxes

            satisfied = []
            selected  = None

            if boxes is not None and len(boxes) > 0:
                xyxy = boxes.xyxy.cpu().numpy()
                conf = boxes.conf.cpu().numpy()
                cls  = boxes.cls.cpu().numpy().astype(int)

                for i in range(len(cls)):
                    cname = names[cls[i]] if (names and cls[i] in range(len(names))) else str(cls[i])
                    cconf = float(conf[i])
                    x1, y1, x2, y2 = [float(v) for v in xyxy[i]]
                    w = max(0.0, x2 - x1)
                    h = max(0.0, y2 - y1)
                    area = w * h

                    rule = CLASS_RULES.get(cname)
                    if rule is None:
                        continue
                    if cconf < rule.get("conf_min", 0.0):
                        continue
                    if w < rule.get("w_min", 0.0):
                        continue
                    if h < rule.get("h_min", 0.0):
                        continue
                    if area < rule.get("area_min", 0.0):
                        continue

                    satisfied.append({
                        "class": cname,
                        "conf": round(cconf, 3),
                        "w": int(w),
                        "h": int(h)
                    })

                if satisfied:
                    ranked = sorted(
                        satisfied,
                        key=lambda d: PRIORITY.get(d["class"], 0),
                        reverse=True
                    )
                    selected = ranked[0]["class"]

            out = {
                "ts": time.time(),
                "selected": selected,
                "satisfied_classes": [d["class"] for d in satisfied],
            }
            json_bytes = json.dumps(out).encode("utf-8")
            send_with_size(pi_sock, json_bytes)

            # 전송 로그
            print(f"[NB] Sent to Pi ({len(json_bytes)} bytes): "
                  f"selected={selected}, all={out['satisfied_classes']}")

    except KeyboardInterrupt:
        pass
    finally:
        try: conn.close()
        except: pass
        try: srv.close()
        except: pass
        try: pi_sock.close()
        except: pass
        print("[NB] closed.")

if __name__ == "__main__":
    main()
