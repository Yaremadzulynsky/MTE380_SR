"""
LINE_FOLLOW dispatcher — selects behaviour based on cfg.line_follow_mode:
  1  find_line       steer + slow on curvature + FIND_LINE when lost
  4  find_heading    lateral PID + heading PID + FIND_LINE when lost

Transition logic:
  is_returning=False  blue_found                              → DRIVE_FORWARD
  is_returning=True   green_found for >= cfg.green_delay_s   → DROP_OFF
                      green lost before timer expires         → timer resets
"""
from __future__ import annotations

import time

from states import ControlOutput, State
import states.line_follow_find         as _find
import states.line_follow_find_heading as _find_heading


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if sm.is_returning:
        if det.green_found:
            if sm._green_seen_t is None:
                sm._green_seen_t = time.time()
            elif time.time() - sm._green_seen_t >= sm.cfg.green_delay_s:
                sm._green_seen_t  = None
                sm._dropoff_phase = None   # ensure DROP_OFF initialises cleanly
                sm._enter(State.DROP_OFF, left_ticks, right_ticks)
                return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)
        else:
            sm._green_seen_t = None   # reset timer if green disappears
    else:
        if det.blue_found:
            sm._enter(State.CENTER_BLUE, left_ticks, right_ticks)
            return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    mode = sm.cfg.line_follow_mode
    if mode == 4:
        return _find_heading.step(sm, det, left_ticks, right_ticks)
    return _find.step(sm, det, left_ticks, right_ticks)
