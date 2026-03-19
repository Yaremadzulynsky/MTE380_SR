"""Line detector — finds the red tape centroid and returns a normalized lateral error."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class VisionResult:
    error: float       # lateral offset, [-1, 1]: negative = line left, positive = line right
    detected: bool
    centroid_x: int    # pixel x of centroid in full frame coords
    centroid_y: int    # pixel y of centroid in full frame coords
    debug_frame: np.ndarray


class LineDetector:
    def __init__(self, cfg: dict, roi_bottom_frac: float):
        self._red_lo1 = np.array(cfg['red_lo1'], dtype=np.uint8)
        self._red_hi1 = np.array(cfg['red_hi1'], dtype=np.uint8)
        self._red_lo2 = np.array(cfg['red_lo2'], dtype=np.uint8)
        self._red_hi2 = np.array(cfg['red_hi2'], dtype=np.uint8)
        k = cfg['morph_kernel']
        self._kernel   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        self._min_area = cfg['min_area']
        self._roi_frac = roi_bottom_frac

    def process(self, frame: np.ndarray) -> VisionResult:
        h, w = frame.shape[:2]
        roi_y = int(h * (1.0 - self._roi_frac))

        roi = frame[roi_y:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self._red_lo1, self._red_hi1),
            cv2.inRange(hsv, self._red_lo2, self._red_hi2),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) >= self._min_area]

        debug = frame.copy()
        # Draw ROI boundary
        cv2.line(debug, (0, roi_y), (w, roi_y), (80, 80, 80), 1)
        # Draw center reference
        cv2.line(debug, (w // 2, roi_y), (w // 2, h), (60, 60, 60), 1)

        if not valid:
            cv2.putText(debug, 'NO LINE', (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 200), 1)
            return VisionResult(error=0.0, detected=False,
                                centroid_x=w // 2, centroid_y=h // 2,
                                debug_frame=debug)

        largest = max(valid, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return VisionResult(error=0.0, detected=False,
                                centroid_x=w // 2, centroid_y=h // 2,
                                debug_frame=debug)

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00']) + roi_y   # back to full-frame coords

        # Normalize: positive → line is right of center → steer right
        half_w = w / 2.0
        error = (cx - half_w) / half_w

        # Draw detection
        cv2.drawContours(debug[roi_y:], [largest], -1, (0, 200, 0), 2)
        cv2.circle(debug, (cx, cy), 6, (0, 0, 255), -1)
        cv2.line(debug, (w // 2, cy), (cx, cy), (0, 255, 255), 1)
        cv2.putText(debug, f'err={error:+.3f}', (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return VisionResult(error=error, detected=True,
                            centroid_x=cx, centroid_y=cy,
                            debug_frame=debug)
