"""Red path heading extraction and line fitting."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from src.utils.math2d import unit


def _centerline_points(mask: np.ndarray) -> np.ndarray:
    ys, xs = np.where(mask > 0)
    if ys.size == 0:
        return np.empty((0, 2), dtype=np.float32)

    # Vectorized centerline extraction:
    # for each row y, compute mean x of active pixels.
    h = mask.shape[0]
    counts = np.bincount(ys, minlength=h).astype(np.float32)
    sums = np.bincount(ys, weights=xs.astype(np.float32), minlength=h).astype(np.float32)
    valid = counts > 0
    y_vals = np.nonzero(valid)[0].astype(np.float32)
    x_vals = sums[valid] / counts[valid]
    return np.column_stack((x_vals, y_vals)).astype(np.float32)


def _all_contour_points(contours: list[np.ndarray]) -> np.ndarray:
    if not contours:
        return np.empty((0, 2), dtype=np.float32)
    pts = np.vstack(contours).reshape(-1, 2).astype(np.float32)
    return pts


def extract_heading(
    red_mask: np.ndarray,
    prev_heading: np.ndarray,
    min_area: float,
    use_centerline: bool = True,
) -> tuple[np.ndarray, float, list[np.ndarray], dict[str, Any]]:
    """
    Fit heading vector from accepted red contours.

    Returns:
        heading_raw: unit vector (or previous vector if no detection)
        detected_area: sum area of accepted contours
        accepted_contours: contours used for heading estimation
        debug: diagnostic values
    """
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    accepted = [c for c in contours if cv2.contourArea(c) >= min_area]
    total_area = float(sum(cv2.contourArea(c) for c in accepted))

    if not accepted:
        return unit(prev_heading), 0.0, [], {"fit_ok": False}

    if use_centerline:
        draw_mask = np.zeros_like(red_mask)
        cv2.drawContours(draw_mask, accepted, -1, 255, thickness=cv2.FILLED)
        pts = _centerline_points(draw_mask)
    else:
        pts = _all_contour_points(accepted)

    if pts.shape[0] < 2:
        return unit(prev_heading), total_area, accepted, {"fit_ok": False}

    line = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
    vx, vy = float(line[0][0]), float(line[1][0])

    # Force "forward" direction to point upwards in image coordinates.
    if vy > 0:
        vx, vy = -vx, -vy

    return unit([vx, vy]), total_area, accepted, {"fit_ok": True}
