"""LINE_FOLLOW mode 4 — lateral + heading PID, FIND_LINE on loss.

Two PID loops run in parallel and their outputs are summed:
  _steer_pid   : corrects lateral offset (red_error)
  _heading_pid : corrects angle between line tangent and robot direction (curve_heading)

The heading PID grows the turn command as the robot becomes more misaligned with
the line's tangent, independent of lateral position. Forward speed scales down
linearly as heading error grows, reaching min_speed at heading_speed_thresh.

Transitions to FIND_LINE when the line is lost.
"""
from __future__ import annotations

from states import ControlOutput, State, scale_pair_to_max_speed, _clamp


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if not det.red_found:
        sm._enter(State.FIND_LINE, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    sm._last_curvature = det.curvature
    if det.tape_cx_px is not None:
        sm._find_line_turn_cw = det.tape_cx_px > sm.cfg.camera_width / 2.0

    # Lateral correction: steer PID on centroid offset
    lateral_turn = sm._steer_pid(-det.red_error)
    lateral_turn = _clamp(lateral_turn, -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)

    # Heading correction: heading PID on tangent misalignment.
    # In reverse the rear aligns with the tangent, so the error sign flips.
    rev = sm.cfg.reverse_line_follow
    heading_sign = 1.0 if rev else -1.0
    heading_turn = sm._heading_pid(heading_sign * det.curve_heading)
    heading_turn = _clamp(heading_turn, -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)

    sm._heading_lateral_turn = lateral_turn
    sm._heading_heading_turn = heading_turn
    turn = _clamp(lateral_turn + heading_turn, -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)

    curv_scale = _clamp(1.0 - abs(det.curve_heading) / sm.cfg.corner_curvature_thresh, 0.0, 1.0)
    fwd = sm.cfg.min_speed + (sm.cfg.base_speed - sm.cfg.min_speed) * curv_scale
    if rev:
        fwd = -fwd

    left  = _clamp(fwd + turn, -1.0, 1.0)
    right = _clamp(fwd - turn, -1.0, 1.0)
    left, right = scale_pair_to_max_speed(sm, left, right)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
