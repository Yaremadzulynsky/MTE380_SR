"""Shared perception pipeline used by runtime and tools."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from src.config import AppConfig
from src.utils.math2d import unit
from src.vision.confidence import compute_gamma
from src.vision.hough_heading import extract_hough_heading
from src.vision.masks import build_masks, to_hsv
from src.vision.zones import classify_zone


@dataclass
class PipelineState:
    """Stateful pipeline fields carried frame-to-frame."""

    p_prev: np.ndarray = field(default_factory=lambda: unit([0.0, -1.0]))
    err_prev: float = 0.0
    path_x_hist: deque[float] = field(default_factory=lambda: deque(maxlen=7))
    path_y_hist: deque[float] = field(default_factory=lambda: deque(maxlen=7))
    err_hist: deque[float] = field(default_factory=lambda: deque(maxlen=7))
    missed_path_frames: int = 0
    lock_segment: tuple[int, int, int, int] | None = None
    lock_missed_frames: int = 0
    centerline_prev_x: np.ndarray | None = None
    path_mask_key: str = "red"


@dataclass
class PipelineOutput:
    px: float
    py: float
    line_error_x: float
    zone: str
    gamma: float
    path_detected: bool
    path_mask_key: str
    target_detected: bool = False
    target_px: float = 0.0
    target_py: float = 0.0
    debug_artifacts: dict[str, Any] = field(default_factory=dict)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _median(hist: deque[float]) -> float:
    if not hist:
        return 0.0
    return float(np.median(np.asarray(hist, dtype=np.float32)))


def run_pipeline(roi_bgr: np.ndarray, state: PipelineState, cfg: AppConfig) -> PipelineOutput:
    """Process one ROI frame and update pipeline state."""
    hsv = to_hsv(roi_bgr)
    masks = build_masks(hsv, cfg)
    heading_mask = masks[state.path_mask_key]

    raw_heading, area_used, line_error_x, hough_detected, heading_debug, centerline_next_x = extract_hough_heading(
        path_mask=heading_mask,
        prev_heading=state.p_prev,
        cfg=cfg.hough,
        lock_segment=state.lock_segment,
        lock_missed_frames=state.lock_missed_frames,
        lock_frames=cfg.smoothing.segment_lock_frames,
        lock_score_margin=cfg.smoothing.segment_switch_score_margin,
        lock_max_missed_frames=cfg.smoothing.segment_max_missed_frames,
        centerline_lookahead_ratio=cfg.smoothing.centerline_lookahead_ratio,
        centerline_prev_x=state.centerline_prev_x,
        centerline_ema_alpha=cfg.smoothing.centerline_ema_alpha,
    )
    state.centerline_prev_x = centerline_next_x
    sel = heading_debug.get("selected_segment")
    if isinstance(sel, list) and len(sel) == 4:
        state.lock_segment = tuple(int(v) for v in sel)
    state.lock_missed_frames = int(heading_debug.get("lock_missed_next", state.lock_missed_frames))
    path_detected = bool(hough_detected and heading_debug.get("fit_ok", False))

    # Keep running history for robust median + EMA smoothing.
    smooth_cfg = cfg.smoothing
    win = max(1, int(smooth_cfg.window_size))
    if state.path_x_hist.maxlen != win:
        state.path_x_hist = deque(list(state.path_x_hist)[-win:], maxlen=win)
    if state.path_y_hist.maxlen != win:
        state.path_y_hist = deque(list(state.path_y_hist)[-win:], maxlen=win)
    if state.err_hist.maxlen != win:
        state.err_hist = deque(list(state.err_hist)[-win:], maxlen=win)

    raw_path = unit(raw_heading)
    state.path_x_hist.append(float(raw_path[0]))
    state.path_y_hist.append(float(raw_path[1]))
    state.err_hist.append(float(line_error_x))

    med_path = unit([_median(state.path_x_hist), _median(state.path_y_hist)])
    if float(np.linalg.norm(med_path)) < 1e-9:
        med_path = unit(state.p_prev)
    med_err = _median(state.err_hist)

    if path_detected:
        state.missed_path_frames = 0
    else:
        state.missed_path_frames += 1

    freeze_after = max(0, int(smooth_cfg.freeze_on_miss_frames))
    should_update = path_detected or state.missed_path_frames <= freeze_after
    if should_update:
        a_path = _clamp(float(smooth_cfg.ema_alpha_path), 0.0, 0.999)
        a_err = _clamp(float(smooth_cfg.ema_alpha_error), 0.0, 0.999)

        p_filt = unit(a_path * unit(state.p_prev) + (1.0 - a_path) * med_path)
        if float(np.linalg.norm(p_filt)) < 1e-9:
            p_filt = unit([0.0, -1.0])

        err_ema = a_err * float(state.err_prev) + (1.0 - a_err) * float(med_err)
        max_step = max(0.0, float(smooth_cfg.max_error_step))
        if max_step > 0.0:
            err_delta = err_ema - float(state.err_prev)
            err_ema = float(state.err_prev) + _clamp(err_delta, -max_step, max_step)
        err_filt = _clamp(float(err_ema), -1.0, 1.0)

        state.p_prev = p_filt
        state.err_prev = err_filt
    else:
        p_filt = unit(state.p_prev)
        err_filt = float(state.err_prev)

    zone, zone_debug = classify_zone(
        masks=masks,
        zone_cfg=cfg.zones,
        path_mask_key=state.path_mask_key,
    )
    gamma = compute_gamma(area_used, cfg.confidence.expected_area)

    # target_detected: True when blue circular blob found (TARGET zone), False otherwise
    target_found = zone_debug.get("target_found", False)
    target_detected = bool(target_found)
    target_px, target_py = 0.0, 0.0
    if target_detected:
        tb = zone_debug.get("target_best", {})
        cx = tb.get("cx", 0.0)
        cy = tb.get("cy", 0.0)
        h, w = masks["blue"].shape[:2]
        if w > 0 and h > 0:
            dx = (cx - w / 2.0) / (w / 2.0)
            dy = (h / 2.0 - cy) / (h / 2.0)
            norm = float(np.linalg.norm([dx, dy]))
            if norm > 1e-9:
                target_px = float(dx / norm)
                target_py = float(dy / norm)

    return PipelineOutput(
        px=float(p_filt[0]),
        py=float(p_filt[1]),
        line_error_x=float(err_filt),
        zone=zone,
        gamma=float(gamma),
        path_detected=path_detected,
        path_mask_key=state.path_mask_key,
        target_detected=target_detected,
        target_px=target_px,
        target_py=target_py,
        debug_artifacts={
            "masks": masks,
            "raw_heading": raw_heading,
            "path_area_used": area_used,
            "path_mask_key": state.path_mask_key,
            "heading_debug": heading_debug,
            "smoothing_debug": {
                "line_error_raw": float(line_error_x),
                "line_error_median": float(med_err),
                "line_error_filtered": float(err_filt),
                "path_raw": [float(raw_path[0]), float(raw_path[1])],
                "path_median": [float(med_path[0]), float(med_path[1])],
                "path_filtered": [float(p_filt[0]), float(p_filt[1])],
                "missed_path_frames": int(state.missed_path_frames),
                "window_size": win,
            },
            "zone_debug": zone_debug,
        },
    )
