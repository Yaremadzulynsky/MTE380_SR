from __future__ import annotations

import time

from states import ControlOutput, State


def step(sm, left_ticks: int, right_ticks: int) -> ControlOutput:
    pos = (left_ticks + right_ticks) / 2.0
    if abs(sm._pos_pid.setpoint - pos) <= sm.cfg.pos_tolerance:
        sm._last_curve_coast_t = time.time()
        sm._enter(State.LINE_FOLLOW)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)
    spd = sm._pos_pid(pos)
    return ControlOutput(left=spd, right=spd, claw=None, state=sm.state, direct_voltage=True)
