"""
LINE_FOLLOW dispatcher — selects behaviour based on cfg.line_follow_mode:
  0  simple          steer + slow on curvature + stop when lost
  1  find_line       steer + slow on curvature + FIND_LINE when lost
  2  pid_turn        steer + slow on curvature + PID_TURN on sharp corner
  3  adaptive        blend red_error with horizontal pixel balance
  4  find_heading    lateral PID + heading PID + FIND_LINE when lost
  5  slow_down       like find_line but ramps speed from base to min after a time threshold
"""
from __future__ import annotations

from states import ControlOutput
import states.line_follow_simple          as _simple
import states.line_follow_find            as _find
import states.line_follow_pid_turn        as _pid_turn
import states.line_follow_adaptive        as _adaptive
import states.line_follow_find_heading    as _find_heading
import states.line_follow_find_slow_down  as _slow_down


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    mode = sm.cfg.line_follow_mode
    if mode == 1:
        return _find.step(sm, det, left_ticks, right_ticks)
    if mode == 2:
        return _pid_turn.step(sm, det, left_ticks, right_ticks)
    if mode == 3:
        return _adaptive.step(sm, det, left_ticks, right_ticks)
    if mode == 4:
        return _find_heading.step(sm, det, left_ticks, right_ticks)
    if mode == 5:
        return _slow_down.step(sm, det, left_ticks, right_ticks)
    return _simple.step(sm, det, left_ticks, right_ticks)
