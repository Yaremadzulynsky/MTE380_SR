"""LINE_FOLLOW_REVERSED mode 1 — steer along line backwards, slow on curvature, FIND_LINE_REVERSED when lost.

Steering maths (backward drive):
  forward version:  left = +fwd + turn,  right = +fwd - turn
  reversed version: left = -fwd + turn,  right = -fwd - turn

The turn correction sign is unchanged — if the line is to the right the PID
still outputs a positive turn, which makes the left motor less negative and
the right motor more negative, arcing the robot's rear to the right. ✓
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

    curv_scale = _clamp(1.0 - abs(det.curvature) / sm.cfg.corner_curvature_thresh, 0.0, 1.0)
    fwd  = sm.cfg.min_speed + (sm.cfg.base_speed - sm.cfg.min_speed) * curv_scale
    turn = sm._steer_pid(-det.red_error)

    left  = _clamp(-fwd + turn, -1.0, 1.0)
    right = _clamp(-fwd - turn, -1.0, 1.0)
    left, right = scale_pair_to_max_speed(sm, left, right)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
