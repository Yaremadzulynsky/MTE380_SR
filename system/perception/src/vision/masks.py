"""HSV masking and morphology helpers."""

from __future__ import annotations

from typing import Dict

import cv2
import numpy as np

from src.config import AppConfig, HSVRange


def resolve_roi_y_start(
    frame_height: int,
    roi_y_start: int,
    roi_y_start_ratio: float | None = None,
) -> int:
    """Resolve the ROI top row from either an absolute pixel value or a height ratio."""
    h = max(1, int(frame_height))
    if roi_y_start_ratio is not None:
        y = int(round(float(roi_y_start_ratio) * h))
    else:
        y = int(roi_y_start)
    return max(0, min(y, h - 1))


def crop_roi(frame: np.ndarray, roi_y_start: int, roi_y_start_ratio: float | None = None) -> np.ndarray:
    """Crop frame to region-of-interest from roi_y_start to bottom."""
    y = resolve_roi_y_start(frame.shape[0], roi_y_start, roi_y_start_ratio)
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
    """Build cleaned binary masks for red/green/blue/black/danger."""
    red = cv2.bitwise_or(_mask_range(hsv, cfg.red1), _mask_range(hsv, cfg.red2))
    green = _mask_range(hsv, cfg.green)
    blue = _mask_range(hsv, cfg.blue)
    black = _mask_range(hsv, cfg.black)
    danger = _mask_range(hsv, cfg.danger)

    m = cfg.morph
    return {
        "red": clean_mask(red, m.kernel_size, m.open_iters, m.close_iters),
        "green": clean_mask(green, m.kernel_size, m.open_iters, m.close_iters),
        "blue": clean_mask(blue, m.kernel_size, m.open_iters, m.close_iters),
        "black": clean_mask(black, m.kernel_size, m.open_iters, m.close_iters),
        "danger": clean_mask(danger, m.kernel_size, m.open_iters, m.close_iters),
    }


def mask_ratio(mask: np.ndarray) -> float:
    """Return fraction of non-zero pixels in mask."""
    total = mask.shape[0] * mask.shape[1]
    if total <= 0:
        return 0.0
    return float(cv2.countNonZero(mask)) / float(total)
