"""Camera perception: red line, blue target, and T-junction detection."""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from picamera2 import Picamera2


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class FrameDetection:
    red_found:  bool        # red line detected in ROI
    red_error:  float       # [-1, 1], positive = line is right of centre
    blue_found: bool        # blue target marker visible
    t_junction: bool        # wide red stripe = end T-junction


class Perception:
    """
    Reads one camera frame and returns a FrameDetection.

    Red line : HSV mask → largest contour → centroid error
    Blue target : HSV mask → any contour above min area
    T-junction  : red bounding-rect width > half the frame = horizontal end bar
    """

    # ── HSV colour ranges ─────────────────────────────────────────────────────
    # Red often wraps around 0/179 in HSV, so keep two bands.
    _RED_LO1 = np.array([  0, 100,  70], np.uint8)
    _RED_HI1 = np.array([ 12, 255, 255], np.uint8)
    _RED_LO2 = np.array([168, 100,  70], np.uint8)
    _RED_HI2 = np.array([179, 255, 255], np.uint8)

    # Blue tuned to be a bit more forgiving but still reject washed-out noise.
    _BLUE_LO = np.array([ 95, 120,  70], np.uint8)
    _BLUE_HI = np.array([135, 255, 255], np.uint8)

    def __init__(
        self,
        width:  int   = 640,
        height: int   = 480,
        roi_top_ratio: float = 0.5,   # ignore top fraction of frame (avoids far-ahead noise)
        red_min_area:  float = 80.0,
        blue_min_area: float = 1500.0,  # require a substantial blue patch to avoid false triggers
        t_junction_width_ratio: float = 0.5,  # red bbox > this fraction of frame = T-junction
    ) -> None:
        self.width  = width
        self.height = height
        self.roi_top_ratio = _clamp(roi_top_ratio, 0.0, 0.95)
        self.red_min_area  = red_min_area
        self.blue_min_area = blue_min_area
        self.t_junction_width_ratio = t_junction_width_ratio

        self._cam = Picamera2()
        cfg = self._cam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"},
            buffer_count=2,
        )
        self._cam.configure(cfg)
        self._cam.start()

    # ── Public ────────────────────────────────────────────────────────────────

    def read_frame(self) -> np.ndarray | None:
        frame = self._cam.capture_array("main")
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def detect(self, frame: np.ndarray) -> FrameDetection:
        h, w = frame.shape[:2]
        roi_y = int(h * self.roi_top_ratio)
        roi   = frame[roi_y:, :]
        hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_found, red_error, t_junction = self._detect_red(hsv, w)
        blue_found = self._detect_blue(hsv)

        return FrameDetection(
            red_found=red_found,
            red_error=red_error,
            blue_found=blue_found,
            t_junction=t_junction,
        )

    def release(self) -> None:
        self._cam.stop()
        self._cam.close()

    def draw_debug(self, frame: np.ndarray, det: FrameDetection) -> np.ndarray:
        out = frame.copy()
        h, w = out.shape[:2]
        roi_y = int(h * self.roi_top_ratio)

        # ROI boundary and centre line
        cv2.line(out, (w // 2, roi_y), (w // 2, h - 1), (255, 255, 255), 2)
        cv2.line(out, (0, roi_y), (w - 1, roi_y), (200, 200, 200), 1)

        tags = [
            f"red={'Y' if det.red_found else 'N'}  err={det.red_error:+.2f}",
            f"blue={'Y' if det.blue_found else 'N'}",
            f"T={'Y' if det.t_junction else 'N'}",
        ]
        for i, tag in enumerate(tags):
            cv2.putText(out, tag, (10, 30 + i * 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
        return out

    # ── Internals ─────────────────────────────────────────────────────────────

    def _detect_red(self, hsv: np.ndarray, frame_w: int) -> tuple[bool, float, bool]:
        """Returns (found, error, t_junction)."""
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self._RED_LO1, self._RED_HI1),
            cv2.inRange(hsv, self._RED_LO2, self._RED_HI2),
        )
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 0.0, False

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.red_min_area:
            return False, 0.0, False

        _, _, bw, _ = cv2.boundingRect(largest)
        t_junction  = (bw / frame_w) > self.t_junction_width_ratio

        moments = cv2.moments(largest)
        if moments["m00"] <= 1e-6:
            return True, 0.0, t_junction

        cx    = moments["m10"] / moments["m00"]
        error = _clamp((cx - frame_w / 2.0) / (frame_w / 2.0), -1.0, 1.0)
        return True, error, t_junction

    def _detect_blue(self, hsv: np.ndarray) -> bool:
        """Returns True when a blue marker of sufficient area is visible."""
        mask      = cv2.inRange(hsv, self._BLUE_LO, self._BLUE_HI)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(cv2.contourArea(c) >= self.blue_min_area for c in contours)
