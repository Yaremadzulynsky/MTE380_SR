"""LINE_FOLLOW mode 5 — steer along line, recover when lost, ramp speed down over time."""
from __future__ import annotations

import time

from states import ControlOutput, State, steer, _clamp


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if det.blue_found:
        sm._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    if not det.red_found:
        sm._enter(State.FIND_LINE, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    sm._last_curvature = det.curvature
    if det.tape_cx_px is not None:
        sm._find_line_turn_cw = det.tape_cx_px > sm.cfg.camera_width / 2.0

    # Time-based speed ramp: after slow_down_after_s seconds reduce base_speed
    # linearly down to min_speed over slow_down_ramp_s seconds.
    elapsed = time.time() - sm._t0
    after   = sm.cfg.slow_down_after_s
    ramp    = max(sm.cfg.slow_down_ramp_s, 1e-6)
    if elapsed >= after:
        frac     = _clamp((elapsed - after) / ramp, 0.0, 1.0)
        adj_base = sm.cfg.base_speed + (sm.cfg.min_speed - sm.cfg.base_speed) * frac
    else:
        adj_base = sm.cfg.base_speed

    # Also scale for curvature (same as line_follow_find)
    curv_scale = _clamp(1.0 - abs(det.curvature) / sm.cfg.corner_curvature_thresh, 0.0, 1.0)
    adj_base   = sm.cfg.min_speed + (adj_base - sm.cfg.min_speed) * curv_scale

    left, right = steer(sm, det.red_error, base_speed=adj_base)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
