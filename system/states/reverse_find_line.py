"""
REVERSE_FIND_LINE — entered after PICKUP when the robot needs to back up onto the red line.

Two phases:
  1. Reverse at reverse_find_speed (direct voltage, no steering) until red is detected.
     sm._rfl_found_t is None during this phase.
  2. Continue reversing for reverse_line_drive_s seconds so the robot is fully over
     the line before stopping, then transition to TURN_180.
     sm._rfl_found_t holds the time.monotonic() timestamp when the line was first seen.
"""
from __future__ import annotations

import time

from states import ControlOutput, State


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    spd = -abs(sm.cfg.reverse_find_speed)

    # Phase 1: reverse until red line is visible.
    # _rfl_found_t is None until the first frame where red is detected.
    if getattr(sm, "_rfl_found_t", None) is None:
        if det.red_found:
            sm._rfl_found_t = time.monotonic()
        return ControlOutput(left=-spd, right=-spd, claw=None, state=sm.state,
                             direct_voltage=True)

    # Phase 2: line found — continue reversing for reverse_line_drive_s so the
    # robot is well clear of the pickup zone before spinning, then → TURN_180.
    if time.monotonic() - sm._rfl_found_t < sm.cfg.reverse_line_drive_s:
        return ControlOutput(left=-spd, right=-spd, claw=None, state=sm.state,
                             direct_voltage=True)

    sm._rfl_found_t = None
    sm._enter(State.TURN_180, left_ticks, right_ticks)
    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)
