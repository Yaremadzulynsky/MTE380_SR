"""Path observation extraction from the fitted line."""

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


def _select_lookahead_point(
    points: np.ndarray,
    probe: np.ndarray,
    target_y: float,
) -> np.ndarray:
    if points.shape[0] == 0:
        return probe.astype(np.float32)

    # Prefer points that are at or ahead of the robot in image coordinates.
    ahead = points[points[:, 1] <= probe[1]]
    candidates = ahead if ahead.shape[0] > 0 else points
    order = np.argsort(candidates[:, 1])
    candidates = candidates[order]

    if candidates.shape[0] == 1:
        return np.asarray([candidates[0, 0], float(target_y)], dtype=np.float32)

    y_vals = candidates[:, 1].astype(np.float32)
    x_vals = candidates[:, 0].astype(np.float32)
    target_y_f = float(target_y)

    if target_y_f <= float(y_vals[0]):
        return np.asarray([x_vals[0], target_y_f], dtype=np.float32)
    if target_y_f >= float(y_vals[-1]):
        return np.asarray([x_vals[-1], target_y_f], dtype=np.float32)

    upper_idx = int(np.searchsorted(y_vals, target_y_f, side="left"))
    lower_idx = max(0, upper_idx - 1)
    y0 = float(y_vals[lower_idx])
    y1 = float(y_vals[upper_idx])
    x0 = float(x_vals[lower_idx])
    x1 = float(x_vals[upper_idx])
    if abs(y1 - y0) <= 1e-6:
        interp_x = 0.5 * (x0 + x1)
    else:
        t = (target_y_f - y0) / (y1 - y0)
        interp_x = x0 + t * (x1 - x0)
    return np.asarray([interp_x, target_y_f], dtype=np.float32)


def _clip_line_to_image(
    width: int,
    height: int,
    anchor: np.ndarray,
    tangent: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    extent = float(max(width, height) * 2)
    p1 = anchor - tangent * extent
    p2 = anchor + tangent * extent
    rect = (0, 0, max(1, int(width - 1)), max(1, int(height - 1)))
    ok, clipped_p1, clipped_p2 = cv2.clipLine(
        rect,
        (int(round(float(p1[0]))), int(round(float(p1[1])))),
        (int(round(float(p2[0]))), int(round(float(p2[1])))),
    )
    if not ok:
        return p1.astype(np.float32), p2.astype(np.float32)
    return (
        np.asarray(clipped_p1, dtype=np.float32),
        np.asarray(clipped_p2, dtype=np.float32),
    )


def extract_path_observation(
    path_mask: np.ndarray,
    min_area: float,
    use_centerline: bool = True,
    probe_x_frac: float = 0.5,
    probe_y_frac: float = 0.9,
    lookahead_y_frac: float = 0.72,
    forward_bias: float = 1.0,
    lateral_gain: float = 0.5,
    max_lateral_abs: float = 0.6,
) -> tuple[np.ndarray, np.ndarray, float, list[np.ndarray], dict[str, Any]]:
    """
    Fit a visible path line and return a camera-frame line-follow vector.

    Returns:
        correction_raw: control vector composed from lateral error + forward bias
        tangent_raw: fitted line tangent for debug overlays
        detected_area: sum area of accepted contours
        accepted_contours: contours used for heading estimation
        debug: diagnostic values
    """
    contours, _ = cv2.findContours(path_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    accepted = [c for c in contours if cv2.contourArea(c) >= min_area]
    total_area = float(sum(cv2.contourArea(c) for c in accepted))

    if not accepted:
        return (
            np.zeros(2, dtype=np.float32),
            np.zeros(2, dtype=np.float32),
            0.0,
            [],
            {"fit_ok": False},
        )

    if use_centerline:
        draw_mask = np.zeros_like(path_mask)
        cv2.drawContours(draw_mask, accepted, -1, 255, thickness=cv2.FILLED)
        pts = _centerline_points(draw_mask)
    else:
        pts = _all_contour_points(accepted)

    if pts.shape[0] < 2:
        return (
            np.zeros(2, dtype=np.float32),
            np.zeros(2, dtype=np.float32),
            total_area,
            accepted,
            {"fit_ok": False},
        )

    line = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
    vx, vy = float(line[0][0]), float(line[1][0])
    x0, y0 = float(line[2][0]), float(line[3][0])

    # Force "forward" direction to point upwards in image coordinates.
    if vy > 0:
        vx, vy = -vx, -vy
    tangent = unit([vx, vy]).astype(np.float32)

    h, w = path_mask.shape[:2]
    probe = np.asarray(
        [
            float(max(0, w - 1) * probe_x_frac),
            float(max(0, h - 1) * probe_y_frac),
        ],
        dtype=np.float32,
    )
    lookahead_y = min(float(probe[1]), float(max(0, h - 1) * lookahead_y_frac))
    anchor = np.asarray([x0, y0], dtype=np.float32)
    seg_a, seg_b = _clip_line_to_image(w, h, anchor, tangent)
    target_point = _select_lookahead_point(pts, probe, lookahead_y)
    geometry_vector_px = target_point - probe
    raw_lateral_error = float(geometry_vector_px[0]) / max((w - 1) / 2.0, 1.0)
    lateral_error = float(np.clip(raw_lateral_error * lateral_gain, -max_lateral_abs, max_lateral_abs))
    correction = np.asarray(
        [
            lateral_error,
            -float(forward_bias),
        ],
        dtype=np.float32,
    )
    correction = np.clip(correction, -1.0, 1.0)

    return correction, tangent, total_area, accepted, {
        "fit_ok": True,
        "probe_point": probe.tolist(),
        "closest_point": target_point.tolist(),
        "target_point": target_point.tolist(),
        "lookahead_y": float(lookahead_y),
        "geometry_vector_px": geometry_vector_px.tolist(),
        "raw_lateral_error": float(raw_lateral_error),
        "lateral_error": float(lateral_error),
        "line_vector": correction.tolist(),
        "tangent": tangent.tolist(),
        "line_anchor": anchor.tolist(),
        "line_segment": [seg_a.tolist(), seg_b.tolist()],
    }
