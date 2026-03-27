"""
LINE_FOLLOW dispatcher — selects behaviour based on cfg.line_follow_mode:
  1  find_line       steer + slow on curvature + FIND_LINE when lost
  4  find_heading    lateral PID + heading PID + FIND_LINE when lost

Transition logic:
  is_returning=False  blue_found    → CENTER_BLUE
"""
from __future__ import annotations

from states import ControlOutput, State
import states.line_follow_find         as _find
import states.line_follow_find_heading as _find_heading


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if not sm.is_returning:
        if det.blue_found:
            sm._enter(State.CENTER_BLUE, left_ticks, right_ticks)
            return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    mode = sm.cfg.line_follow_mode
    if mode == 4:
        return _find_heading.step(sm, det, left_ticks, right_ticks)
    return _find.step(sm, det, left_ticks, right_ticks)
