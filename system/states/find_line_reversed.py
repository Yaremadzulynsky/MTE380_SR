"""
FIND_LINE_REVERSED — recovery spin while in reversed line-follow mode.

Identical spin logic to FIND_LINE, but transitions back to LINE_FOLLOW_REVERSED
instead of LINE_FOLLOW when the line is reacquired.
"""
from __future__ import annotations

from states import ControlOutput, State


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if det.red_found:
        sm._enter(State.LINE_FOLLOW_REVERSED, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    v = sm.cfg.find_line_turn_speed
    if sm._find_line_turn_cw:
        sm._brain._speed_ctrl.set_target(v, -v)
    else:
        sm._brain._speed_ctrl.set_target(-v, v)
    sm._brain._speed_ctrl.step()
    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
