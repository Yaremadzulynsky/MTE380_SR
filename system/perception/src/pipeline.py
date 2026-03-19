"""Shared perception pipeline used by runtime and tools."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np

from src.config import AppConfig
from src.vision.confidence import compute_gamma
from src.vision.heading import extract_path_observation
from src.vision.masks import build_masks, resolve_roi_y_start, to_hsv
from src.vision.zones import classify_zone


@dataclass
class PipelineState:
    """Stateful pipeline fields carried frame-to-frame."""

    lateral_prev: float = 0.0
    path_mask_key: str = "red"


@dataclass
class PipelineOutput:
    px: float
    py: float
    zone: str
    gamma: float
    path_detected: bool
    path_mask_key: str
    target_detected: bool = False
    target_px: float = 0.0
    target_py: float = 0.0
    debug_artifacts: dict[str, Any] = field(default_factory=dict)


def _roi_area_scale(roi_shape: tuple[int, int], cfg: AppConfig) -> float:
    roi_h, roi_w = roi_shape
    base_h = max(1, int(cfg.camera.height))
    base_w = max(1, int(cfg.camera.width))
    base_y = resolve_roi_y_start(base_h, cfg.roi_y_start, cfg.roi_y_start_ratio)
    base_roi_h = max(1, base_h - base_y)
    base_area = float(base_w * base_roi_h)
    actual_area = float(max(1, roi_w) * max(1, roi_h))
    return actual_area / max(base_area, 1.0)


def run_pipeline(roi_bgr: np.ndarray, state: PipelineState, cfg: AppConfig) -> PipelineOutput:
    """Process one ROI frame and update pipeline state."""
    hsv = to_hsv(roi_bgr)
    masks = build_masks(hsv, cfg)
    path_mask = masks[state.path_mask_key]
    area_scale = _roi_area_scale(path_mask.shape[:2], cfg)
    heading_cfg = replace(cfg.heading, min_area=cfg.heading.min_area * area_scale)
    zone_cfg = replace(
        cfg.zones,
        target_min_area=cfg.zones.target_min_area * area_scale,
        danger_area_thresh=cfg.zones.danger_area_thresh * area_scale,
        path_area_thresh=cfg.zones.path_area_thresh * area_scale,
    )
    confidence_cfg = replace(
        cfg.confidence,
        expected_area=cfg.confidence.expected_area * area_scale,
    )

    raw_vector, tangent_vector, area_used, accepted_path_contours, heading_debug = extract_path_observation(
        path_mask=path_mask,
        min_area=heading_cfg.min_area,
        use_centerline=heading_cfg.use_centerline,
        probe_x_frac=heading_cfg.probe_x_frac,
        probe_y_frac=heading_cfg.probe_y_frac,
        lookahead_y_frac=heading_cfg.lookahead_y_frac,
        forward_bias=heading_cfg.forward_bias,
        lateral_gain=heading_cfg.lateral_gain,
        max_lateral_abs=heading_cfg.max_lateral_abs,
    )

    path_detected = heading_debug.get("fit_ok", False)
    if path_detected:
        lateral_filt = float(cfg.alpha * state.lateral_prev + (1.0 - cfg.alpha) * float(raw_vector[0]))
        p_filt = np.asarray([lateral_filt, float(raw_vector[1])], dtype=np.float32)
    else:
        p_filt = np.zeros(2, dtype=np.float32)
        lateral_filt = 0.0
    state.lateral_prev = float(np.clip(lateral_filt, -1.0, 1.0))
    p_filt[0] = state.lateral_prev

    zone, zone_debug = classify_zone(
        masks=masks,
        zone_cfg=zone_cfg,
        path_mask_key=state.path_mask_key,
    )
    gamma = compute_gamma(area_used, confidence_cfg.expected_area)

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
        zone=zone,
        gamma=float(gamma),
        path_detected=path_detected,
        path_mask_key=state.path_mask_key,
        target_detected=target_detected,
        target_px=target_px,
        target_py=target_py,
        debug_artifacts={
            "masks": masks,
            "raw_vector": raw_vector,
            "tangent_vector": tangent_vector,
            "path_area_used": area_used,
            "path_mask_key": state.path_mask_key,
            "accepted_path_contours": accepted_path_contours,
            "heading_debug": heading_debug,
            "zone_debug": zone_debug,
            "area_scale": area_scale,
        },
    )
