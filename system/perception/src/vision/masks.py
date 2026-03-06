"""HSV masking and morphology helpers."""

from __future__ import annotations

from typing import Dict

import cv2
import numpy as np

from src.config import AppConfig, HSVRange


def crop_roi(frame: np.ndarray, roi_y_start: int) -> np.ndarray:
    """Crop frame to region-of-interest from roi_y_start to bottom."""
    y = max(0, min(int(roi_y_start), frame.shape[0] - 1))
    return frame[y:, :]


def to_hsv(roi_bgr: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)


def _mask_range(hsv: np.ndarray, r: HSVRange) -> np.ndarray:
    lo = np.array(r.lo, dtype=np.uint8)
    hi = np.array(r.hi, dtype=np.uint8)
    return cv2.inRange(hsv, lo, hi)


def clean_mask(mask: np.ndarray, kernel_size: int, open_iters: int, close_iters: int) -> np.ndarray:
    """Apply open+close morphology to reduce noise and fill small gaps."""
    k = max(1, int(kernel_size))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    out = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=max(0, int(open_iters)))
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel, iterations=max(0, int(close_iters)))
    return out


def build_masks(hsv: np.ndarray, cfg: AppConfig) -> Dict[str, np.ndarray]:
    """Build cleaned binary masks for red/green/blue/danger."""
    red = cv2.bitwise_or(_mask_range(hsv, cfg.red1), _mask_range(hsv, cfg.red2))
    green = _mask_range(hsv, cfg.green)
    blue = _mask_range(hsv, cfg.blue)
    danger = _mask_range(hsv, cfg.danger)

    m = cfg.morph
    return {
        "red": clean_mask(red, m.kernel_size, m.open_iters, m.close_iters),
        "green": clean_mask(green, m.kernel_size, m.open_iters, m.close_iters),
        "blue": clean_mask(blue, m.kernel_size, m.open_iters, m.close_iters),
        "danger": clean_mask(danger, m.kernel_size, m.open_iters, m.close_iters),
    }


def mask_ratio(mask: np.ndarray) -> float:
    """Return fraction of non-zero pixels in mask."""
    total = mask.shape[0] * mask.shape[1]
    if total <= 0:
        return 0.0
    return float(cv2.countNonZero(mask)) / float(total)
