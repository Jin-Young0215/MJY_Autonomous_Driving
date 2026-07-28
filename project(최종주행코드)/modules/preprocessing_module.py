# modules/preprocessing_module.py
import cv2
import numpy as np

class PreprocessingModule:
    def __init__(self,
                 src_points=None,
                 dst_points=None,
                 blur_kernel=(5,5),
                 hsv_white_lower=(0, 0, 190),
                 hsv_white_upper=(180, 30, 255),
                 min_area=150,
                 weak_connect=False):
        self.src_points = np.float32([[0, 390], [640, 390], [640, 190], [0, 190]]) if src_points is None else src_points
        self.dst_points = np.float32([[0, 480], [640, 480], [640,   0], [0,   0]]) if dst_points is None else dst_points
        self.blur_kernel   = blur_kernel
        self.hsv_lower     = np.array(hsv_white_lower, dtype=np.uint8)
        self.hsv_upper     = np.array(hsv_white_upper, dtype=np.uint8)
        self.min_area      = min_area
        self.weak_connect  = weak_connect

        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        self.k3 = np.ones((7, 1), np.uint8)

    def _base_binary(self, frame):
        bev = cv2.warpPerspective(frame, self.M, (640, 480))
        hsv  = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        lane = cv2.bitwise_and(bev, bev, mask=mask)
        gray = cv2.cvtColor(lane, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, self.blur_kernel, 0)
        _, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.k3, iterations=1)
        if self.weak_connect:
            binary = cv2.dilate(binary, self.k3, iterations=1)
        return binary

    def process(self, frame):
        binary = self._base_binary(frame)
        num, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
        areas = []
        for lid in range(1, num):
            area = stats[lid, cv2.CC_STAT_AREA]
            if area >= self.min_area:
                areas.append((area, lid))
        areas.sort(reverse=True, key=lambda x: x[0])
        top2 = [lid for _, lid in areas[:2]]
        kept = np.zeros_like(binary)
        for lid in top2:
            kept[labels == lid] = 255
        return kept
