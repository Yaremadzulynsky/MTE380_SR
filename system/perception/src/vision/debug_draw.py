"""Debug overlays for ROI and masks."""

from __future__ import annotations

import cv2
import numpy as np


def draw_overlay(
    roi_bgr: np.ndarray,
    heading: np.ndarray,
    zone: str,
    gamma: float,
) -> np.ndarray:
    """Draw heading arrow, zone label, and confidence on ROI frame."""
    out = roi_bgr.copy()
    h, w = out.shape[:2]
    origin = (w // 2, h - 10)
    scale = int(min(h, w) * 0.25)
    tip = (
        int(origin[0] + float(heading[0]) * scale),
        int(origin[1] + float(heading[1]) * scale),
    )
    cv2.arrowedLine(out, origin, tip, (255, 0, 0), 2, tipLength=0.2)  # blue (BGR)
    cv2.putText(out, f"zone={zone}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(out, f"gamma={gamma:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(
        out,
        f"p=({float(heading[0]):.2f},{float(heading[1]):.2f})",
        (10, 76),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    return out


def make_mask_preview(masks: dict[str, np.ndarray]) -> np.ndarray:
    """Create a tiled BGR preview of masks for debug view."""
    red = cv2.cvtColor(masks["red"], cv2.COLOR_GRAY2BGR)
    green = cv2.cvtColor(masks["green"], cv2.COLOR_GRAY2BGR)
    blue = cv2.cvtColor(masks["blue"], cv2.COLOR_GRAY2BGR)
    danger = cv2.cvtColor(masks["danger"], cv2.COLOR_GRAY2BGR)
    top = np.hstack([red, green])
    bot = np.hstack([blue, danger])
    preview = np.vstack([top, bot])
    cv2.putText(preview, "RED", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(preview, "GREEN", (red.shape[1] + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(preview, "BLUE", (10, red.shape[0] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    cv2.putText(
        preview,
        "DANGER",
        (red.shape[1] + 10, red.shape[0] + 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (200, 200, 200),
        2,
    )
    return preview
