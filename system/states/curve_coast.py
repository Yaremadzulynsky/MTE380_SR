from __future__ import annotations

import time

from states import ControlOutput, State


def step(sm, left_ticks: int, right_ticks: int) -> ControlOutput:
    sm._last_curve_coast_t = time.time()
    sm._enter(State.LINE_FOLLOW)
    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)
