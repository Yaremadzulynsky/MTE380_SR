"""
CENTER_BLUE — stop and rotate in-place until the blue circle centroid is centred.

Uses blue_cx_norm ([-1, 1], positive = blue is right of frame centre) with a
proportional controller.  Once |error| <= blue_center_tolerance, transitions to
DRIVE_FORWARD to cover blue_approach_m metres to the pickup point.

If blue is lost for a tick the robot holds position and waits.
"""
from __future__ import annotations

from states import ControlOutput, State, _clamp


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if not det.blue_found or det.blue_cx_norm is None:
        # Blue temporarily lost — hold still, preserve integral
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    error = det.blue_cx_norm  # positive = blue is right of centre

    if abs(error) <= sm.cfg.blue_center_tolerance:
        sm._center_blue_integral   = 0.0
        sm._center_blue_prev_error = 0.0
        sm._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    sm._center_blue_integral = getattr(sm, "_center_blue_integral", 0.0) + error
    prev_error = getattr(sm, "_center_blue_prev_error", error)
    derivative = error - prev_error
    sm._center_blue_prev_error = error

    # Turn in-place toward the blue circle.
    # error > 0 → blue is right → spin CW: left fwd, right back.
    spd  = sm.cfg.blue_center_speed
    turn = _clamp(
        sm.cfg.blue_center_kp * error
        + sm.cfg.blue_center_ki * sm._center_blue_integral
        + sm.cfg.blue_center_kd * derivative,
        -spd, spd,
    )
    return ControlOutput(left=turn, right=-turn, claw=None, state=sm.state)
