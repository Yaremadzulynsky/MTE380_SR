"""
FIND_LINE state — entered from LINE_FOLLOW when the red line is lost.

Spins in-place toward the last known curvature direction until the line
is reacquired, then transitions back to LINE_FOLLOW.

Direction convention (same as corner_turn):
  _last_curvature >= 0  → line curved right → spin CW  (left=+v, right=−v)
  _last_curvature <  0  → line curved left  → spin CCW (left=−v, right=+v)
"""
from __future__ import annotations

from states import ControlOutput, State


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    # Line reacquired → back to LINE_FOLLOW
    if det.red_found:
        sm._enter(State.LINE_FOLLOW, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # Spin toward last known curvature direction
    v = sm.cfg.find_line_turn_speed
    if sm._last_curvature >= 0.0:
        left, right = v, -v   # CW
    else:
        left, right = -v, v   # CCW

    sm._brain.send_voltage(left, right)
    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
