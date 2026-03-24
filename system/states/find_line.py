"""
FIND_LINE state — entered from line_follow_find when the red line is lost.

Spins in-place based on which side of the frame had more red in the last
detection (stored as sm._find_line_turn_cw by line_follow_find):
  True  → centroid was right of centre → spin CW  (left=+v, right=−v)
  False → centroid was left of centre  → spin CCW (left=−v, right=+v)

Transitions back to LINE_FOLLOW once the line is reacquired.
"""
from __future__ import annotations

from states import ControlOutput, State


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    # Line reacquired → back to LINE_FOLLOW
    if det.red_found:
        sm._enter(State.LINE_FOLLOW, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # Spin toward the side that last had more red.
    # Use the speed controller PID so the integral term ramps voltage up if
    # the wheels stall against static friction.
    v = sm.cfg.find_line_turn_speed
    if sm._find_line_turn_cw:
        sm._brain._speed_ctrl.set_target(v, -v)   # CW
    else:
        sm._brain._speed_ctrl.set_target(-v, v)   # CCW
    sm._brain._speed_ctrl.step()
    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
