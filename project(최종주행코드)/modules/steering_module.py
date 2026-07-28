#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from modules.preprocessing_module import PreprocessingModule

class SteeringAnalyzer:
    """
    전처리 + 조향각/포인트/호모그래피 등 '데이터만' 계산.
    - 주행 정책(예: TURN_DELTA, LANE_ADJ_DELTA) 불포함
    - 그리기/오버레이 불포함
    """

    def __init__(self,
                 base_angle: float = 78.5,
                 max_offset: float = 48.5,
                 angle_limits: tuple = (30.0, 150.0),
                 y_bottom_ratio: float = 0.70,
                 y_top_ratio: float = 0.40,
                 k_head: float = 0.60):
        self.base_angle = float(base_angle)
        self.max_offset = float(max_offset)
        self.angle_limits = (float(angle_limits[0]), float(angle_limits[1]))
        self.yb_ratio = float(y_bottom_ratio)
        self.yt_ratio = float(y_top_ratio)
        self.k_head = float(k_head)
        self.pre = PreprocessingModule()  # 전처리기(계산 전용, 그리기 없음)

    # ---------------- 내부 유틸 ----------------
    def _compute_scan_points(self, binary: np.ndarray):
        """
        전처리 이진 영상에서 스캔라인 y, 각 라인의 중앙점(c_bot, c_top) 계산.
        반환: dict {yb, yt, c_bot, c_top, w, h, center_x} 또는 None
        """
        if binary is None or binary.ndim != 2:
            return None

        h, w = binary.shape
        yb = int(np.clip(h * self.yb_ratio, 0, h - 1))
        yt = int(np.clip(h * self.yt_ratio, 0, h - 1))

        def center_on_row(y: int):
            line = binary[y, :]
            xs = np.flatnonzero(line > 0)  # 255 픽셀 인덱스
            if xs.size < 2:
                return None
            return int((xs[0] + xs[-1]) // 2)

        c_bot = center_on_row(yb)
        c_top = center_on_row(yt)

        return {
            "yb": yb, "yt": yt,
            "c_bot": c_bot, "c_top": c_top,
            "w": w, "h": h,
            "center_x": w // 2
        }

    def _estimate_angle(self, binary: np.ndarray, pts: dict | None) -> float:
        """
        스캔 포인트와 이진 영상을 바탕으로 조향각 계산.
        """
        if binary is None or binary.ndim != 2 or not pts:
            return float(np.clip(self.base_angle, *self.angle_limits))

        yb, yt = pts["yb"], pts["yt"]
        c_bot, c_top = pts["c_bot"], pts["c_top"]
        w = pts["w"]

        if c_bot is None or c_top is None:
            return float(np.clip(self.base_angle, *self.angle_limits))

        # (1) 하단 중앙의 좌우 오프셋
        dx_lat_norm = (c_bot - (w // 2)) / (w / 2)

        # (2) 위/아래 중앙을 잇는 선의 기울기(헤딩)
        dy = max(1, abs(yb - yt))    # 0 나눗셈 방지
        dx = (c_top - c_bot)
        heading_deg = float(np.degrees(np.arctan2(dx, dy)))

        angle = self.base_angle + dx_lat_norm * self.max_offset + self.k_head * heading_deg
        return float(np.clip(angle, *self.angle_limits))

    # ---------------- 공개 API ----------------
    def analyze_binary(self, binary: np.ndarray) -> dict:
        """
        이미 전처리된 이진 영상을 받아 각도/포인트 계산.
        반환:
        {
          "angle": float,
          "points": {yb, yt, c_bot, c_top, w, h, center_x}
        }
        """
        pts = self._compute_scan_points(binary)
        angle = self._estimate_angle(binary, pts)
        return {"angle": angle, "points": pts}

    def analyze_frame(self, frame_bgr) -> dict:
        """
        BGR 프레임을 받아 전처리 → 포인트 → 각도 → 호모그래피까지 계산.
        (그리기 없음, 정책 파라미터 없음)
        반환:
        {
          "angle": float,
          "points": {yb, yt, c_bot, c_top, w, h, center_x},
          "homography": { "src_points", "dst_points", "M", "Minv" },
          "binary": np.ndarray(HxW, uint8)
        }
        """
        binary = self.pre.process(frame_bgr)
        pts = self._compute_scan_points(binary)
        angle = self._estimate_angle(binary, pts)

        # 호모그래피 데이터 (표시 레이어에서 ROI/역투영에 사용)
        src = self.pre.src_points.copy()
        dst = self.pre.dst_points.copy()
        M = self.pre.M.copy()
        Minv = cv2.getPerspectiveTransform(dst, src)

        return {
            "angle": angle,
            "points": pts,
            "homography": {
                "src_points": src,
                "dst_points": dst,
                "M": M,
                "Minv": Minv
            },
            "binary": binary
        }
