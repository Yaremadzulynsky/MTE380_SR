from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from src.config import AppConfig


def _unit(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-9:
        return np.zeros_like(vec, dtype=np.float32)
    return (vec / norm).astype(np.float32)


def _crop_roi(frame: np.ndarray, start_ratio: float) -> np.ndarray:
    h = frame.shape[0]
    y0 = int(max(0, min(h - 1, round(h * start_ratio))))
    return frame[y0:, :]


def _centerline_points(mask: np.ndarray) -> np.ndarray:
    ys, xs = np.where(mask > 0)
    if ys.size == 0:
        return np.empty((0, 2), dtype=np.float32)
    h = mask.shape[0]
    counts = np.bincount(ys, minlength=h).astype(np.float32)
    sums = np.bincount(ys, weights=xs.astype(np.float32), minlength=h).astype(np.float32)
    valid = counts > 0
    y_vals = np.nonzero(valid)[0].astype(np.float32)
    x_vals = sums[valid] / counts[valid]
    return np.column_stack((x_vals, y_vals)).astype(np.float32)


def _select_lookahead_point(points: np.ndarray, probe: np.ndarray, target_y: float) -> np.ndarray:
    if points.shape[0] == 0:
        return probe.astype(np.float32)

    ahead = points[points[:, 1] <= probe[1]]
    candidates = ahead if ahead.shape[0] > 0 else points
    order = np.argsort(candidates[:, 1])
    candidates = candidates[order]

    if candidates.shape[0] == 1:
        return np.asarray([candidates[0, 0], target_y], dtype=np.float32)

    y_vals = candidates[:, 1].astype(np.float32)
    x_vals = candidates[:, 0].astype(np.float32)
    target_y = float(target_y)

    if target_y <= float(y_vals[0]):
        return np.asarray([x_vals[0], target_y], dtype=np.float32)
    if target_y >= float(y_vals[-1]):
        return np.asarray([x_vals[-1], target_y], dtype=np.float32)

    upper = int(np.searchsorted(y_vals, target_y, side="left"))
    lower = max(0, upper - 1)
    y0 = float(y_vals[lower])
    y1 = float(y_vals[upper])
    x0 = float(x_vals[lower])
    x1 = float(x_vals[upper])
    if abs(y1 - y0) <= 1e-6:
        x = 0.5 * (x0 + x1)
    else:
        t = (target_y - y0) / (y1 - y0)
        x = x0 + t * (x1 - x0)
    return np.asarray([x, target_y], dtype=np.float32)


def _build_mask(roi_bgr: np.ndarray, cfg: AppConfig) -> np.ndarray:
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    if cfg.path_mask_key == "red":
        mask = cv2.inRange(hsv, np.array(cfg.red1.lo), np.array(cfg.red1.hi))
        mask |= cv2.inRange(hsv, np.array(cfg.red2.lo), np.array(cfg.red2.hi))
    elif cfg.path_mask_key == "blue":
        mask = cv2.inRange(hsv, np.array(cfg.blue.lo), np.array(cfg.blue.hi))
    else:
        mask = cv2.inRange(hsv, np.array(cfg.black.lo), np.array(cfg.black.hi))

    kernel = np.ones((cfg.morph.kernel_size, cfg.morph.kernel_size), dtype=np.uint8)
    if cfg.morph.open_iters > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=cfg.morph.open_iters)
    if cfg.morph.close_iters > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=cfg.morph.close_iters)
    return mask


def _to_robot_frame_clamped(vec_image: np.ndarray) -> np.ndarray:
    px = float(vec_image[0])
    py = float(-vec_image[1])
    norm = float(np.hypot(px, py))
    if norm < 1e-9:
        return np.zeros(2, dtype=np.float32)
    if norm > 1.0:
        px /= norm
        py /= norm
    return np.asarray([px, py], dtype=np.float32)


def _normalize_geometry_vector_px(vec_px: np.ndarray, width: int, height: int) -> np.ndarray:
    return np.asarray(
        [
            float(vec_px[0]) / max(width / 2.0, 1.0),
            float(vec_px[1]) / max(height - 1, 1.0),
        ],
        dtype=np.float32,
    )


@dataclass
class PipelineState:
    pid_integral: float = 0.0
    pid_prev_error: float = 0.0


@dataclass
class PipelineOutput:
    roi_bgr: np.ndarray
    mask: np.ndarray
    path_detected: bool
    image_vector: np.ndarray
    robot_vector: np.ndarray
    lookahead_image_vector: np.ndarray
    lookahead_robot_vector: np.ndarray
    tangent_vector: np.ndarray
    zone: str
    gamma: float
    debug: dict[str, Any] = field(default_factory=dict)


def run_pipeline(frame_bgr: np.ndarray, state: PipelineState, cfg: AppConfig) -> PipelineOutput:
    roi = _crop_roi(frame_bgr, cfg.roi_y_start_ratio)
    mask = _build_mask(roi, cfg)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    accepted = [c for c in contours if cv2.contourArea(c) >= cfg.heading.min_area]
    area = float(sum(cv2.contourArea(c) for c in accepted))

    if not accepted:
        return PipelineOutput(
            roi_bgr=roi,
            mask=mask,
            path_detected=False,
            image_vector=np.zeros(2, dtype=np.float32),
            robot_vector=np.zeros(2, dtype=np.float32),
            lookahead_image_vector=np.zeros(2, dtype=np.float32),
            lookahead_robot_vector=np.zeros(2, dtype=np.float32),
            tangent_vector=np.zeros(2, dtype=np.float32),
            zone="SAFE",
            gamma=0.0,
            debug={"fit_ok": False},
        )

    filled = np.zeros_like(mask)
    cv2.drawContours(filled, accepted, -1, 255, thickness=cv2.FILLED)
    points = _centerline_points(filled)
    if points.shape[0] < 2:
        return PipelineOutput(
            roi_bgr=roi,
            mask=mask,
            path_detected=False,
            image_vector=np.zeros(2, dtype=np.float32),
            robot_vector=np.zeros(2, dtype=np.float32),
            lookahead_image_vector=np.zeros(2, dtype=np.float32),
            lookahead_robot_vector=np.zeros(2, dtype=np.float32),
            tangent_vector=np.zeros(2, dtype=np.float32),
            zone="SAFE",
            gamma=0.0,
            debug={"fit_ok": False},
        )

    line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
    vx, vy = float(line[0][0]), float(line[1][0])
    if vy > 0.0:
        vx, vy = -vx, -vy
    tangent = _unit(np.asarray([vx, vy], dtype=np.float32))

    h, w = mask.shape[:2]
    probe = np.asarray(
        [
            float((w - 1) * cfg.heading.probe_x_frac),
            float((h - 1) * cfg.heading.probe_y_frac),
        ],
        dtype=np.float32,
    )
    lookahead_y = min(float(probe[1]), float((h - 1) * cfg.heading.lookahead_y_frac))
    target_point = _select_lookahead_point(points, probe, lookahead_y)
    geometry_vector = target_point - probe
    lateral_target_point = np.asarray([target_point[0], probe[1]], dtype=np.float32)
    raw_lateral = float(geometry_vector[0]) / max((w - 1) / 2.0, 1.0)
    lateral = float(
        np.clip(
            raw_lateral * cfg.heading.lateral_gain,
            -cfg.heading.max_lateral_abs,
            cfg.heading.max_lateral_abs,
        )
    )
    error = 0.0 if abs(lateral) < cfg.heading.pid_deadband else lateral
    derivative = error - state.pid_prev_error
    state.pid_prev_error = error
    state.pid_integral = float(
        np.clip(
            state.pid_integral + error,
            -cfg.heading.pid_i_max,
            cfg.heading.pid_i_max,
        )
    )
    turn = (
        cfg.heading.pid_kp * error
        + cfg.heading.pid_ki * state.pid_integral
        + cfg.heading.pid_kd * derivative
    )
    turn = float(np.clip(turn, -1.0, 1.0))
    speed = (
        cfg.heading.base_speed * cfg.heading.forward_bias
        - cfg.heading.error_slowdown * abs(error)
    )
    speed = float(np.clip(speed, cfg.heading.min_speed, cfg.heading.max_speed))

    robot_vector = np.asarray([turn, speed], dtype=np.float32)
    image_vector = np.asarray([turn, -speed], dtype=np.float32)
    lookahead_image_vector = np.clip(_normalize_geometry_vector_px(geometry_vector, w, h), -1.0, 1.0)
    lookahead_robot_vector = _to_robot_frame_clamped(lookahead_image_vector)
    gamma = float(min(area / max(cfg.expected_area, 1.0), 1.0))

    return PipelineOutput(
        roi_bgr=roi,
        mask=mask,
        path_detected=True,
        image_vector=image_vector,
        robot_vector=robot_vector,
        lookahead_image_vector=lookahead_image_vector,
        lookahead_robot_vector=lookahead_robot_vector,
        tangent_vector=tangent,
        zone="PATH",
        gamma=gamma,
        debug={
            "fit_ok": True,
            "probe_point": probe.tolist(),
            "target_point": target_point.tolist(),
            "lateral_target_point": lateral_target_point.tolist(),
            "geometry_vector_px": geometry_vector.tolist(),
            "raw_lateral_error": raw_lateral,
            "lateral_error": error,
            "turn_command": turn,
            "speed_command": speed,
            "lookahead_image_vector": lookahead_image_vector.tolist(),
            "area": area,
        },
    )


def draw_overlay(output: PipelineOutput) -> np.ndarray:
    frame = output.roi_bgr.copy()
    h, w = frame.shape[:2]
    probe = output.debug.get("probe_point")
    target = output.debug.get("target_point")
    lateral_target = output.debug.get("lateral_target_point")
    origin = (
        int(round(probe[0])) if probe else w // 2,
        int(round(probe[1])) if probe else h - 10,
    )
    line_tip = (
        int(origin[0] + float(output.image_vector[0]) * max(w / 2.0, 1.0)),
        int(origin[1] + float(output.image_vector[1]) * max(h - 1, 1.0)),
    )
    cv2.circle(frame, origin, 5, (255, 255, 255), -1)

    if target:
        target_pt = (int(round(target[0])), int(round(target[1])))
        cv2.circle(frame, target_pt, 5, (0, 255, 0), -1)
        cv2.arrowedLine(frame, origin, target_pt, (0, 255, 0), 2, tipLength=0.18)
        cv2.putText(
            frame,
            "lookahead offset",
            (target_pt[0] + 6, target_pt[1] - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            1,
        )
        tangent_origin = target_pt
    else:
        tangent_origin = origin

    if lateral_target:
        lateral_pt = (int(round(lateral_target[0])), int(round(lateral_target[1])))
        cv2.circle(frame, lateral_pt, 4, (255, 0, 255), -1)
        cv2.arrowedLine(frame, origin, lateral_pt, (255, 0, 255), 2, tipLength=0.18)
        cv2.putText(
            frame,
            "lateral error",
            (lateral_pt[0] + 6, lateral_pt[1] - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 0, 255),
            1,
        )

    cv2.arrowedLine(frame, origin, line_tip, (255, 0, 0), 2, tipLength=0.2)
    cv2.putText(frame, "control vector", (line_tip[0] + 6, line_tip[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)

    tangent_tip = (
        int(tangent_origin[0] + float(output.tangent_vector[0]) * min(h, w) * 0.25),
        int(tangent_origin[1] + float(output.tangent_vector[1]) * min(h, w) * 0.25),
    )
    cv2.arrowedLine(frame, tangent_origin, tangent_tip, (255, 255, 0), 2, tipLength=0.2)
    cv2.putText(
        frame,
        "line tangent",
        (tangent_tip[0] + 6, tangent_tip[1] + 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 0),
        1,
    )

    cv2.putText(frame, f"zone={output.zone}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"gamma={output.gamma:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(
        frame,
        f"line=({float(output.lookahead_image_vector[0]):.2f},{float(output.lookahead_image_vector[1]):.2f})",
        (10, 76),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        f"ctrl=({float(output.image_vector[0]):.2f},{float(output.image_vector[1]):.2f})",
        (10, 102),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        f"tan=({float(output.tangent_vector[0]):.2f},{float(output.tangent_vector[1]):.2f})",
        (10, 128),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        f"lat={float(output.debug.get('lateral_error', 0.0)):.2f}",
        (10, 154),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    return frame
