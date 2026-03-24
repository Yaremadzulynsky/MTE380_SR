"""
LINE_FOLLOW_REVERSED dispatcher — follows the red line driving backwards.

Selects sub-behaviour based on cfg.line_follow_mode:
  1  find_line_reversed       steer + slow on curvature + FIND_LINE_REVERSED when lost
  4  find_heading_reversed    lateral PID + heading PID (inverted) + FIND_LINE_REVERSED when lost
"""
from __future__ import annotations

from states import ControlOutput
import states.line_follow_find_reversed         as _find_rev
import states.line_follow_find_heading_reversed as _find_heading_rev


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    mode = sm.cfg.line_follow_mode
    if mode == 4:
        return _find_heading_rev.step(sm, det, left_ticks, right_ticks)
    return _find_rev.step(sm, det, left_ticks, right_ticks)
