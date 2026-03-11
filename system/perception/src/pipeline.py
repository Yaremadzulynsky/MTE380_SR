"""Shared perception pipeline used by runtime and tools."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from src.config import AppConfig
from src.utils.math2d import unit
from src.vision.confidence import compute_gamma
from src.vision.heading import extract_heading
from src.vision.masks import build_masks, to_hsv
from src.vision.zones import classify_zone


@dataclass
class PipelineState:
    """Stateful pipeline fields carried frame-to-frame."""

    p_prev: np.ndarray = field(default_factory=lambda: unit([0.0, -1.0]))
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


def run_pipeline(roi_bgr: np.ndarray, state: PipelineState, cfg: AppConfig) -> PipelineOutput:
    """Process one ROI frame and update pipeline state."""
    hsv = to_hsv(roi_bgr)
    masks = build_masks(hsv, cfg)
    heading_mask = masks[state.path_mask_key]

    raw_heading, area_used, accepted_path_contours, heading_debug = extract_heading(
        red_mask=heading_mask,
        prev_heading=state.p_prev,
        min_area=cfg.heading.min_area,
        use_centerline=cfg.heading.use_centerline,
    )

    p_filt = unit(cfg.alpha * unit(state.p_prev) + (1.0 - cfg.alpha) * unit(raw_heading))
    if float(np.linalg.norm(p_filt)) < 1e-9:
        p_filt = unit([0.0, -1.0])
    state.p_prev = p_filt

    zone, zone_debug = classify_zone(
        masks=masks,
        zone_cfg=cfg.zones,
        path_mask_key=state.path_mask_key,
    )
    gamma = compute_gamma(area_used, cfg.confidence.expected_area)

    path_detected = heading_debug.get("fit_ok", False)

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
            "raw_heading": raw_heading,
            "path_area_used": area_used,
            "path_mask_key": state.path_mask_key,
            "accepted_path_contours": accepted_path_contours,
            "heading_debug": heading_debug,
            "zone_debug": zone_debug,
        },
    )
