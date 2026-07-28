#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import cv2
import numpy as np
from flask import Flask, Response

class FlaskModule:
    def __init__(self, port=5000, jpeg_quality=85, fps=30):
        self.port = int(port)
        self.jpeg_quality = int(jpeg_quality)
        self.fps = int(max(1, fps))
        self.app = Flask(__name__)
        self.server_started = False
        self._frames = [None, None, None]
        self._lock = threading.Lock()
        self._setup_routes()

    def update_rgb(self, frame_bgr):
        with self._lock:
            self._frames[0] = frame_bgr

    def update_pre(self, binary, points):
        vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        if points:
            yb, yt = points["yb"], points["yt"]
            c_bot, c_top = points["c_bot"], points["c_top"]
            center_x = points["center_x"]
            w, h = points["w"], points["h"]

            # 스캔라인
            cv2.line(vis, (0, yb), (w-1, yb), (0,180,0), 2, cv2.LINE_AA)
            cv2.line(vis, (0, yt), (w-1, yt), (0,180,0), 2, cv2.LINE_AA)

            # 중앙 세로 점선
            for y in range(0, h, 6):
                cv2.line(vis, (center_x, y), (center_x, min(y+3, h-1)), (180,180,180), 1, cv2.LINE_AA)

            # 포인트 + 연결선
            if c_bot is not None:
                cv2.circle(vis, (int(c_bot), int(yb)), 6, (255,100,0), -1, cv2.LINE_AA)
            if c_top is not None:
                cv2.circle(vis, (int(c_top), int(yt)), 6, (0,90,255), -1, cv2.LINE_AA)
            if c_bot is not None and c_top is not None:
                cv2.line(vis, (int(c_bot), int(yb)), (int(c_top), int(yt)), (0,255,255), 2, cv2.LINE_AA)

        with self._lock:
            self._frames[1] = vis

    def update_origin(self, frame_bgr, homography, points, angle, mode, speed):
        """
        3번째 슬롯 오버레이:
        - ROI 폴리라인
        - 포인트 역투영 + 연결선
        - 멀티라인 텍스트 (MODE / FSM 등 mode 문자열의 줄바꿈 지원 + SPEED + ANGLE)
        """
        vis = frame_bgr.copy()

        # ROI 라인
        if homography and "src_points" in homography:
            src = homography["src_points"].astype(np.int32)
            cv2.polylines(vis, [src], True, (0,200,255), 2, cv2.LINE_AA)

        # 포인트 역투영
        if points and points.get("c_bot") is not None and points.get("c_top") is not None:
            yb, yt = points["yb"], points["yt"]
            c_bot, c_top = points["c_bot"], points["c_top"]
            Minv = homography.get("Minv", None) if homography else None
            if Minv is not None:
                p_bot = np.array([[[float(c_bot), float(yb)]]], dtype=np.float32)
                p_top = np.array([[[float(c_top), float(yt)]]], dtype=np.float32)
                p_bot_w = cv2.perspectiveTransform(p_bot, Minv)[0,0,:]
                p_top_w = cv2.perspectiveTransform(p_top, Minv)[0,0,:]
                cv2.circle(vis, (int(p_bot_w[0]), int(p_bot_w[1])), 6, (255,100,0), -1, cv2.LINE_AA)
                cv2.circle(vis, (int(p_top_w[0]), int(p_top_w[1])), 6, (0,90,255), -1, cv2.LINE_AA)
                cv2.line(vis, (int(p_bot_w[0]), int(p_bot_w[1])), (int(p_top_w[0]), int(p_top_w[1])), (0,255,255), 2, cv2.LINE_AA)

        # 중앙 점선 가이드
        h, w = vis.shape[:2]
        cx = w // 2
        for y in range(0, h, 6):
            cv2.line(vis, (cx, y), (cx, min(y+3, h-1)), (180,180,180), 1, cv2.LINE_AA)

        # ───────── 멀티라인 텍스트 렌더링 ─────────
        # mode 인자로 "FORWARD\nFSM:2" 처럼 들어올 수 있으므로 줄바꿈 지원
        mode_lines = str(mode).splitlines() if mode is not None else []
        lines = []

        if len(mode_lines) == 0:
            lines.append("MODE: -")
        else:
            # 첫 줄은 MODE, 이후 줄은 그대로(예: FSM:2, LABEL:xxx 등)
            lines.append(f"MODE: {mode_lines[0]}")
            for extra in mode_lines[1:]:
                lines.append(extra)

        # 숫자 정보 추가
        try:
            lines.append(f"SPEED: {int(speed)}")
        except Exception:
            lines.append("SPEED: -")

        try:
            lines.append(f"ANGLE: {float(angle):.1f}")
        except Exception:
            lines.append("ANGLE: -")

        # 좌상단에 줄단위로 표시
        y0 = 32      # 첫 줄 y
        dy = 28      # 줄 간격
        x0 = 10
        for i, text in enumerate(lines):
            y = y0 + i * dy
            # 바깥 흰색 두껍게
            cv2.putText(vis, text, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 3, cv2.LINE_AA)
            # 안쪽 진한 회색 얇게
            cv2.putText(vis, text, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (20,20,20), 1, cv2.LINE_AA)

        with self._lock:
            self._frames[2] = vis

    def _setup_routes(self):
        @self.app.route("/slot/<int:idx>")
        def slot(idx):
            def gen():
                while True:
                    with self._lock:
                        frame = self._frames[idx-1] if 1 <= idx <= 3 else None
                    if frame is None:
                        time.sleep(0.01); continue
                    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                    if not ok:
                        time.sleep(0.01); continue
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")
                    time.sleep(1.0 / self.fps)
            return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

        @self.app.route("/")
        def index():
            return '''
            <div style="display:flex;gap:12px;flex-wrap:wrap;">
              <div><h3>1) RGB</h3><img src="/slot/1" style="max-width:100%;height:auto;"></div>
              <div><h3>2) Preprocessed + Points</h3><img src="/slot/2" style="max-width:100%;height:auto;"></div>
              <div><h3>3) Origin + ROI + Points + MODE/SPEED/ANGLE</h3><img src="/slot/3" style="max-width:100%;height:auto;"></div>
            </div>
            '''

    def start(self, host="0.0.0.0"):
        if not self.server_started:
            threading.Thread(
                target=self.app.run,
                kwargs={"host": host, "port": self.port, "debug": False, "use_reloader": False},
                daemon=True
            ).start()
            self.server_started = True
