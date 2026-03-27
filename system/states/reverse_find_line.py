"""
REVERSE_FIND_LINE — two phases:
  1. Drive straight backward at reverse_find_speed until the red line is detected.
  2. Continue reversing for reverse_line_drive_s seconds, then transition to TURN_180.
"""
from __future__ import annotations

import time

from states import ControlOutput, State


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    spd = -abs(sm.cfg.reverse_find_speed)

    # Phase 1: reverse until the red line is detected
    if getattr(sm, "_rfl_found_t", None) is None:
        if det.red_found:
            sm._rfl_found_t = time.monotonic()
        return ControlOutput(left=-spd, right=-spd, claw=None, state=sm.state,
                             direct_voltage=True)

    # Phase 2: line found — continue reversing for reverse_line_drive_s then → TURN_180
    if time.monotonic() - sm._rfl_found_t < sm.cfg.reverse_line_drive_s:
        return ControlOutput(left=-spd, right=-spd, claw=None, state=sm.state,
                             direct_voltage=True)

    sm._rfl_found_t = None
    sm._enter(State.TURN_180, left_ticks, right_ticks)
    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)
