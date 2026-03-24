"""LINE_FOLLOW_REVERSED mode 4 — lateral + heading PID backwards, FIND_LINE_REVERSED on loss.

Lateral correction sign is unchanged (same as forward).
Heading correction is negated: when going backward a rightward line tangent
requires turning left to align, the opposite of forward travel.
Forward speed component is negated: left = -fwd + turn, right = -fwd - turn.
"""
from __future__ import annotations

from states import ControlOutput, State, scale_pair_to_max_speed, _clamp


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if not det.red_found:
        sm._enter(State.FIND_LINE_REVERSED, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    sm._last_curvature = det.curvature
    if det.tape_cx_px is not None:
        sm._find_line_turn_cw = det.tape_cx_px > sm.cfg.camera_width / 2.0

    lateral_turn = sm._steer_pid(-det.red_error)
    lateral_turn = _clamp(lateral_turn, -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)

    # Heading PID input is negated vs forward: curve_heading sign flips when reversing
    heading_turn = sm._heading_pid(det.curve_heading)
    heading_turn = _clamp(heading_turn, -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)

    sm._heading_lateral_turn = lateral_turn
    sm._heading_heading_turn = heading_turn
    turn = _clamp(lateral_turn + heading_turn, -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)

    curv_scale = _clamp(1.0 - abs(det.curvature) / sm.cfg.corner_curvature_thresh, 0.0, 1.0)
    fwd = sm.cfg.min_speed + (sm.cfg.base_speed - sm.cfg.min_speed) * curv_scale

    left  = _clamp(-fwd + turn, -1.0, 1.0)
    right = _clamp(-fwd - turn, -1.0, 1.0)
    left, right = scale_pair_to_max_speed(sm, left, right)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
