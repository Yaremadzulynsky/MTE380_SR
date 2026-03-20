"""Hough-based heading extraction on a binary path mask."""

from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np

from src.config import HoughConfig
from src.utils.math2d import unit


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _line_length(x1: int, y1: int, x2: int, y2: int) -> float:
    return float(np.hypot(float(x2 - x1), float(y2 - y1)))


def _nearest_point_on_segment(
    px: float,
    py: float,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> tuple[float, float]:
    vx = x2 - x1
    vy = y2 - y1
    vv = vx * vx + vy * vy
    if vv < 1e-9:
        return x1, y1
    t = ((px - x1) * vx + (py - y1) * vy) / vv
    t = _clamp(t, 0.0, 1.0)
    return x1 + t * vx, y1 + t * vy


def _nearest_point_on_centerline(
    mask: np.ndarray,
    origin_x: float,
    origin_y: float,
    prev_x_by_row: np.ndarray | None,
    ema_alpha: float,
) -> tuple[tuple[float, float] | None, list[list[float]], np.ndarray, np.ndarray, np.ndarray]:
    """
    Build a centerline from mask rows (mean x per row) and return the closest
    centerline point to origin. This avoids locking to one tape edge.
    """
    ys, xs = np.where(mask > 0)
    if ys.size == 0:
        empty = np.empty((0,), dtype=np.float32)
        next_prev = prev_x_by_row if prev_x_by_row is not None else empty
        return None, [], empty, empty, next_prev
    h = mask.shape[0]
    counts = np.bincount(ys, minlength=h).astype(np.float32)
    sums = np.bincount(ys, weights=xs.astype(np.float32), minlength=h).astype(np.float32)
    valid = counts > 0
    if not np.any(valid):
        empty = np.empty((0,), dtype=np.float32)
        next_prev = prev_x_by_row if prev_x_by_row is not None else empty
        return None, [], empty, empty, next_prev
    y_vals = np.nonzero(valid)[0].astype(np.float32)
    x_vals = sums[valid] / counts[valid]

    # Smooth centerline row-wise over time to reduce jitter.
    alpha = _clamp(float(ema_alpha), 0.0, 0.999)
    raw_x_by_row = np.full((h,), np.nan, dtype=np.float32)
    raw_x_by_row[y_vals.astype(np.int32)] = x_vals
    if prev_x_by_row is None or prev_x_by_row.shape[0] != h:
        smooth_x_by_row = raw_x_by_row.copy()
    else:
        smooth_x_by_row = prev_x_by_row.copy()
        valid_prev = np.isfinite(prev_x_by_row)
        valid_now = np.isfinite(raw_x_by_row)
        blend = valid_now & valid_prev
        smooth_x_by_row[blend] = alpha * prev_x_by_row[blend] + (1.0 - alpha) * raw_x_by_row[blend]
        smooth_x_by_row[valid_now & (~valid_prev)] = raw_x_by_row[valid_now & (~valid_prev)]

    smooth_valid = np.isfinite(smooth_x_by_row)
    smooth_y_vals = np.nonzero(smooth_valid)[0].astype(np.float32)
    smooth_x_vals = smooth_x_by_row[smooth_valid]
    if smooth_x_vals.size == 0:
        return None, [], np.empty((0,), dtype=np.float32), np.empty((0,), dtype=np.float32), smooth_x_by_row

    # Downsample points for lighter debug payload and rendering.
    stride = max(1, int(len(smooth_x_vals) / 120) + 1)
    centerline_points = [[float(x), float(y)] for x, y in zip(smooth_x_vals[::stride], smooth_y_vals[::stride])]
    dx = smooth_x_vals - float(origin_x)
    dy = smooth_y_vals - float(origin_y)
    d2 = dx * dx + dy * dy
    idx = int(np.argmin(d2))
    return (
        (float(smooth_x_vals[idx]), float(smooth_y_vals[idx])),
        centerline_points,
        smooth_x_vals,
        smooth_y_vals,
        smooth_x_by_row,
    )


def _lookahead_point_on_centerline(
    x_vals: np.ndarray,
    y_vals: np.ndarray,
    target_y: float,
) -> tuple[float, float] | None:
    if x_vals.size == 0 or y_vals.size == 0:
        return None
    idx = int(np.argmin(np.abs(y_vals - float(target_y))))
    return float(x_vals[idx]), float(y_vals[idx])


def _select_longest_segments(rows: np.ndarray, cap: int) -> np.ndarray:
    """Keep up to `cap` segments with largest Euclidean length (cheap pre-filter before scoring)."""
    if cap <= 0 or rows.shape[0] <= cap:
        return rows
    x0 = rows[:, 0].astype(np.float64)
    y0 = rows[:, 1].astype(np.float64)
    x1 = rows[:, 2].astype(np.float64)
    y1 = rows[:, 3].astype(np.float64)
    lens = np.hypot(x1 - x0, y1 - y0)
    idx = np.argpartition(-lens, cap - 1)[:cap]
    return rows[idx]


def _segment_distance(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ax1, ay1, ax2, ay2 = [float(v) for v in a]
    bx1, by1, bx2, by2 = [float(v) for v in b]
    direct = np.hypot(ax1 - bx1, ay1 - by1) + np.hypot(ax2 - bx2, ay2 - by2)
    reverse = np.hypot(ax1 - bx2, ay1 - by2) + np.hypot(ax2 - bx1, ay2 - by1)
    return float(min(direct, reverse))


def extract_hough_heading(
    path_mask: np.ndarray,
    prev_heading: np.ndarray,
    cfg: HoughConfig,
    lock_segment: tuple[int, int, int, int] | None = None,
    lock_missed_frames: int = 0,
    lock_frames: int = 5,
    lock_score_margin: float = 1.2,
    lock_max_missed_frames: int = 3,
    centerline_lookahead_ratio: float = 0.35,
    centerline_prev_x: np.ndarray | None = None,
    centerline_ema_alpha: float = 0.8,
) -> tuple[np.ndarray, float, float, bool, dict[str, Any], np.ndarray | None]:
    """
    Estimate line heading and lateral error with HoughLinesP.

    Returns:
        heading_raw: unit heading vector (image frame)
        path_area_used: active pixels in path mask
        line_error_x: normalized lateral offset to nearest point on path centerline [-1, 1]
        detected: whether a valid dominant segment was found
        debug: diagnostic artifacts, including edges and selected segment
    """
    h, w = path_mask.shape[:2]
    if h <= 0 or w <= 0:
        return unit(prev_heading), 0.0, 0.0, False, {"fit_ok": False}, centerline_prev_x

    scale = _clamp(float(cfg.hough_downscale), 0.25, 1.0)
    sx = 1.0
    sy = 1.0
    path_hough = path_mask
    if scale < 0.999:
        small_w = max(16, int(round(w * scale)))
        small_h = max(16, int(round(h * scale)))
        path_hough = cv2.resize(path_mask, (small_w, small_h), interpolation=cv2.INTER_AREA)
        sx = w / float(small_w)
        sy = h / float(small_h)
        s = min(sx, sy)
        min_ll = max(8, int(round(cfg.min_line_length / s)))
        max_gap = max(1, int(round(cfg.max_line_gap / s)))
        area_ratio = (small_w * small_h) / float(w * h)
        # Fewer edge pixels when downscaled; sqrt keeps vote counts roughly in family with full-res threshold
        hough_thresh = max(8, int(round(cfg.threshold * math.sqrt(area_ratio))))
    else:
        min_ll = int(cfg.min_line_length)
        max_gap = int(cfg.max_line_gap)
        hough_thresh = int(cfg.threshold)

    edges = cv2.Canny(
        path_hough,
        threshold1=int(cfg.canny_threshold1),
        threshold2=int(cfg.canny_threshold2),
    )
    theta_rad = float(np.deg2rad(float(cfg.theta_deg)))
    lines = cv2.HoughLinesP(
        edges,
        rho=float(cfg.rho),
        theta=theta_rad,
        threshold=hough_thresh,
        minLineLength=min_ll,
        maxLineGap=max_gap,
    )

    if scale < 0.999:
        edges_dbg = cv2.resize(edges, (w, h), interpolation=cv2.INTER_NEAREST)
    else:
        edges_dbg = edges

    area_used = float(cv2.countNonZero(path_mask))
    if lines is None or len(lines) == 0:
        return unit(prev_heading), area_used, 0.0, False, {"fit_ok": False, "edges": edges_dbg}, centerline_prev_x

    rows = lines.reshape(-1, 4)
    rows = _select_longest_segments(rows, int(cfg.max_hough_candidates))

    center_x = (w - 1) / 2.0
    half_w = max(1.0, w / 2.0)
    best: tuple[float, tuple[int, int, int, int]] | None = None
    lock_match: tuple[float, tuple[int, int, int, int]] | None = None
    lock_nearest: tuple[float, float, tuple[int, int, int, int]] | None = None
    considered = []

    for row in rows:
        r = row.astype(np.float64)
        x1 = int(round(float(r[0]) * sx))
        y1 = int(round(float(r[1]) * sy))
        x2 = int(round(float(r[2]) * sx))
        y2 = int(round(float(r[3]) * sy))
        dx = float(x2 - x1)
        dy = float(y2 - y1)
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            continue
        slope = float("inf") if abs(dx) < 1e-6 else abs(dy / dx)
        if slope < float(cfg.min_abs_slope):
            continue

        length = _line_length(x1, y1, x2, y2)
        mid_x = 0.5 * (float(x1) + float(x2))
        center_penalty = abs(mid_x - center_x) / half_w
        score = length * (1.0 - float(cfg.center_weight) * center_penalty)
        considered.append((x1, y1, x2, y2, score))
        if best is None or score > best[0]:
            best = (score, (x1, y1, x2, y2))
        if lock_segment is not None:
            d = _segment_distance((x1, y1, x2, y2), lock_segment)
            if lock_nearest is None or d < lock_nearest[0]:
                lock_nearest = (d, score, (x1, y1, x2, y2))
            match_thresh = max(12.0, float(cfg.max_line_gap) * 2.5)
            if d <= match_thresh:
                if lock_match is None or score > lock_match[0]:
                    lock_match = (score, (x1, y1, x2, y2))

    if best is None:
        return unit(prev_heading), area_used, 0.0, False, {"fit_ok": False, "edges": edges_dbg}, centerline_prev_x

    selected_source = "best"
    next_lock_missed = 0
    if lock_segment is not None:
        hold_frames = max(0, int(lock_frames))
        if lock_match is not None:
            lock_score = float(lock_match[0])
            best_score = float(best[0])
            margin = max(1.0, float(lock_score_margin))
            prefer_best = best_score > (lock_score * margin)
            if lock_missed_frames < hold_frames:
                chosen = lock_match
                selected_source = "lock_hold"
            else:
                if prefer_best:
                    chosen = best
                    selected_source = "switch_best"
                else:
                    chosen = lock_match
                    selected_source = "lock_keep"
            next_lock_missed = 0
        else:
            miss_cap = max(0, int(lock_max_missed_frames))
            if lock_nearest is not None and lock_missed_frames < miss_cap:
                chosen = (float(lock_nearest[1]), lock_nearest[2])
                selected_source = "lock_nearest"
                next_lock_missed = lock_missed_frames + 1
            elif lock_missed_frames < miss_cap:
                chosen = (0.0, lock_segment)
                selected_source = "lock_predict"
                next_lock_missed = lock_missed_frames + 1
            else:
                chosen = best
                selected_source = "lock_expired_best"
                next_lock_missed = 0
    else:
        chosen = best
        selected_source = "best"
        next_lock_missed = 0

    x1, y1, x2, y2 = chosen[1]
    if y1 <= y2:
        x_top, y_top, x_bottom, y_bottom = x1, y1, x2, y2
    else:
        x_top, y_top, x_bottom, y_bottom = x2, y2, x1, y1

    vx = float(x_top - x_bottom)
    vy = float(y_top - y_bottom)
    heading = unit([vx, vy])
    if float(np.linalg.norm(heading)) < 1e-9:
        heading = unit(prev_heading)

    # Lateral error from robot origin (bottom-center of ROI) to nearest point on
    # the mask centerline. This avoids side-locking on one tape edge.
    origin_x = center_x
    origin_y = float(h - 1)
    centerline_nearest, centerline_points, center_x_vals, center_y_vals, centerline_next_x = _nearest_point_on_centerline(
        path_mask, origin_x, origin_y, centerline_prev_x, centerline_ema_alpha
    )
    lookahead_ratio = _clamp(float(centerline_lookahead_ratio), 0.05, 0.9)
    lookahead_y = float(h - 1) - lookahead_ratio * float(h)
    centerline_lookahead = _lookahead_point_on_centerline(center_x_vals, center_y_vals, lookahead_y)
    if centerline_lookahead is not None:
        nearest_x, nearest_y = centerline_lookahead
        nearest_source = "centerline_lookahead"
    elif centerline_nearest is not None:
        nearest_x, nearest_y = centerline_nearest
        nearest_source = "centerline_nearest_fallback"
    else:
        nearest_x, nearest_y = _nearest_point_on_segment(
            origin_x,
            origin_y,
            float(x1),
            float(y1),
            float(x2),
            float(y2),
        )
        nearest_source = "segment_fallback"
    line_error_x = _clamp((nearest_x - center_x) / half_w, -1.0, 1.0)
    seg_cap = int(cfg.max_debug_segments)
    hough_segments_out = considered
    if seg_cap > 0 and len(considered) > seg_cap:
        hough_segments_out = sorted(considered, key=lambda t: t[4], reverse=True)[:seg_cap]
    debug = {
        "fit_ok": True,
        "edges": edges_dbg,
        "hough_segments": hough_segments_out,
        "selected_segment": [x1, y1, x2, y2],
        "centerline_points": centerline_points,
        "nearest_point": [float(nearest_x), float(nearest_y)],
        "nearest_source": nearest_source,
        "lookahead_y": float(lookahead_y),
        "origin_point": [float(origin_x), float(origin_y)],
        "line_error_x": line_error_x,
        "selected_source": selected_source,
        "lock_missed_next": int(next_lock_missed),
    }
    return heading, area_used, line_error_x, True, debug, centerline_next_x
