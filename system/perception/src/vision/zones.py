"""Zone classification with priority TARGET > DANGER > PATH > SAFE."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from src.config import ZoneConfig
from src.utils.math2d import clamp01
from src.utils.math2d import circularity
from src.vision.masks import mask_ratio


def _contour_areas(mask: np.ndarray) -> tuple[list[np.ndarray], list[float]]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    areas = [float(cv2.contourArea(c)) for c in contours]
    return contours, areas


def classify_zone(
    masks: dict[str, np.ndarray],
    zone_cfg: ZoneConfig,
    path_mask_key: str = "red",
) -> tuple[str, dict[str, Any]]:
    """Classify zone according to configured thresholds and fixed priority."""
    path_mask = masks[path_mask_key]
    green = masks["green"]
    blue = masks["blue"]
    danger = masks["danger"]

    path_ratio = mask_ratio(path_mask)
    green_ratio = mask_ratio(green)
    danger_ratio = mask_ratio(danger)

    path_contours, path_areas = _contour_areas(path_mask)
    path_area_total = float(sum(path_areas))

    danger_contours, danger_areas = _contour_areas(danger)
    danger_area_largest = max(danger_areas, default=0.0)

    # TARGET: blue circular blobs
    target_found = False
    target_best = {"area": 0.0, "circularity": 0.0}
    blue_contours, _ = _contour_areas(blue)
    for c in blue_contours:
        area = float(cv2.contourArea(c))
        if area < zone_cfg.target_min_area:
            continue
        peri = float(cv2.arcLength(c, True))
        circ = circularity(area, peri)
        if circ >= zone_cfg.target_min_circularity:
            target_found = True
            if area > target_best["area"]:
                target_best = {"area": area, "circularity": circ}

    danger_by_ratio = danger_ratio >= zone_cfg.danger_ratio_thresh
    danger_by_area = danger_area_largest >= zone_cfg.danger_area_thresh
    if zone_cfg.danger_mode == "ratio":
        danger_found = danger_by_ratio
    elif zone_cfg.danger_mode == "area":
        danger_found = danger_by_area
    else:
        danger_found = danger_by_ratio or danger_by_area

    path_found = (path_ratio >= zone_cfg.path_ratio_thresh) or (
        path_area_total >= zone_cfg.path_area_thresh
    )

    path_conf_ratio = clamp01(path_ratio / max(zone_cfg.path_ratio_thresh, 1e-9))
    path_conf_area = clamp01(path_area_total / max(zone_cfg.path_area_thresh, 1e-9))
    conf_path = max(path_conf_ratio, path_conf_area)

    danger_conf_ratio = clamp01(danger_ratio / max(zone_cfg.danger_ratio_thresh, 1e-9))
    danger_conf_area = clamp01(danger_area_largest / max(zone_cfg.danger_area_thresh, 1e-9))
    if zone_cfg.danger_mode == "ratio":
        conf_danger = danger_conf_ratio
    elif zone_cfg.danger_mode == "area":
        conf_danger = danger_conf_area
    else:
        conf_danger = max(danger_conf_ratio, danger_conf_area)

    conf_target = 0.0
    if target_best["area"] > 0.0:
        conf_target_area = clamp01(target_best["area"] / max(zone_cfg.target_min_area, 1e-9))
        conf_target_circ = clamp01(
            target_best["circularity"] / max(zone_cfg.target_min_circularity, 1e-9)
        )
        conf_target = min(conf_target_area, conf_target_circ)

    # SAFE confidence: how strongly we are "not path/danger/target", optionally boosted by green.
    conf_not_hazard = clamp01(1.0 - max(conf_path, conf_danger, conf_target))
    conf_green = clamp01(green_ratio / max(zone_cfg.green_ratio_thresh, 1e-9))
    if zone_cfg.safe_green_required:
        conf_safe = min(conf_not_hazard, conf_green)
    else:
        conf_safe = max(conf_not_hazard, 0.35 * conf_green)

    if target_found:
        zone = "TARGET"
    elif danger_found:
        zone = "DANGER"
    elif path_found:
        zone = "PATH"
    else:
        # SAFE can optionally require visible green ratio, but defaults to SAFE.
        if zone_cfg.safe_green_required and green_ratio < zone_cfg.green_ratio_thresh:
            zone = "SAFE"
        else:
            zone = "SAFE"

    return zone, {
        "path_mask_key": path_mask_key,
        "path_ratio": path_ratio,
        "green_ratio": green_ratio,
        "danger_ratio": danger_ratio,
        "path_area_total": path_area_total,
        "danger_area_largest": danger_area_largest,
        "target_found": target_found,
        "target_best": target_best,
        "zone_confidences": {
            "SAFE": float(conf_safe),
            "PATH": float(conf_path),
            "DANGER": float(conf_danger),
            "TARGET": float(conf_target),
        },
        "path_contours": path_contours,
        "danger_contours": danger_contours,
    }
