"""Debug overlays for ROI and masks."""

from __future__ import annotations

import cv2
import numpy as np


def draw_overlay(
    roi_bgr: np.ndarray,
    line_vector: np.ndarray,
    zone: str,
    gamma: float,
    path_detected: bool = True,
    target_detected: bool = False,
    target_heading: np.ndarray | None = None,
    tangent_vector: np.ndarray | None = None,
    probe_point: tuple[float, float] | None = None,
    target_point: tuple[float, float] | None = None,
) -> np.ndarray:
    """Draw predicted vectors, zone label, and confidence on ROI frame."""
    out = roi_bgr.copy()
    h, w = out.shape[:2]
    probe_ok = probe_point is not None and len(probe_point) >= 2
    target_ok = target_point is not None and len(target_point) >= 2
    origin = (
        int(round(probe_point[0])) if probe_ok else w // 2,
        int(round(probe_point[1])) if probe_ok else h - 10,
    )
    line_tip = (
        int(origin[0] + float(line_vector[0]) * max(w / 2.0, 1.0)),
        int(origin[1] + float(line_vector[1]) * max(h - 1, 1.0)),
    )
    tangent_scale = int(min(h, w) * 0.25)
    vector_color = (255, 0, 0) if path_detected else (96, 96, 96)
    cv2.circle(out, origin, 5, (255, 255, 255), -1)
    if target_ok:
        target = (int(round(target_point[0])), int(round(target_point[1])))
        cv2.circle(out, target, 5, (0, 255, 0), -1)
        cv2.arrowedLine(out, origin, target, (0, 255, 0), 2, tipLength=0.18)
        cv2.putText(
            out,
            "lookahead offset",
            (target[0] + 6, target[1] - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            1,
        )
    cv2.arrowedLine(out, origin, line_tip, vector_color, 2, tipLength=0.2)
    cv2.putText(out, "line error", (line_tip[0] + 6, line_tip[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, vector_color, 1)
    if tangent_vector is not None:
        tangent_origin = (
            int(round(target_point[0])) if target_ok else origin[0],
            int(round(target_point[1])) if target_ok else origin[1],
        )
        tangent_tip = (
            int(tangent_origin[0] + float(tangent_vector[0]) * tangent_scale),
            int(tangent_origin[1] + float(tangent_vector[1]) * tangent_scale),
        )
        cv2.arrowedLine(out, tangent_origin, tangent_tip, (255, 255, 0), 2, tipLength=0.2)
        cv2.putText(
            out,
            "line tangent",
            (tangent_tip[0] + 6, tangent_tip[1] + 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 0),
            1,
        )
    if target_detected and target_heading is not None:
        scale = int(min(h, w) * 0.25)
        target_tip = (
            int(origin[0] + float(target_heading[0]) * scale),
            int(origin[1] + float(target_heading[1]) * scale),
        )
        cv2.arrowedLine(out, origin, target_tip, (0, 255, 255), 2, tipLength=0.2)
        cv2.putText(
            out,
            "target vector",
            (target_tip[0] + 6, target_tip[1] + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
        )
    cv2.putText(out, f"zone={zone}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(out, f"gamma={gamma:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(
        out,
        f"line=({float(line_vector[0]):.2f},{float(line_vector[1]):.2f})",
        (10, 76),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    if tangent_vector is not None:
        cv2.putText(
            out,
            f"tan=({float(tangent_vector[0]):.2f},{float(tangent_vector[1]):.2f})",
            (10, 102),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
    if target_ok:
        cv2.putText(
            out,
            f"lookahead=({float(target_point[0]):.0f},{float(target_point[1]):.0f})",
            (10, 128 if tangent_vector is not None else 102),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
    if target_detected and target_heading is not None:
        cv2.putText(
            out,
            f"target=({float(target_heading[0]):.2f},{float(target_heading[1]):.2f})",
            (10, 154 if tangent_vector is not None or target_ok else 102),
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
    black = cv2.cvtColor(masks["black"], cv2.COLOR_GRAY2BGR)
    danger = cv2.cvtColor(masks["danger"], cv2.COLOR_GRAY2BGR)
    blank = np.zeros_like(red)
    top = np.hstack([red, green, blue])
    bot = np.hstack([black, danger, blank])
    preview = np.vstack([top, bot])
    cv2.putText(preview, "RED", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(preview, "GREEN", (red.shape[1] + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(
        preview,
        "BLUE",
        (red.shape[1] * 2 + 10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 0, 0),
        2,
    )
    cv2.putText(preview, "BLACK", (10, red.shape[0] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
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
