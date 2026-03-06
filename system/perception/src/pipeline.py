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
    debug_artifacts: dict[str, Any]


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

    return PipelineOutput(
        px=float(p_filt[0]),
        py=float(p_filt[1]),
        zone=zone,
        gamma=float(gamma),
        path_detected=path_detected,
        path_mask_key=state.path_mask_key,
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
