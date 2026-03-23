from __future__ import annotations

import time

from states import ControlOutput, State


def step(sm) -> ControlOutput:
    if time.time() - sm._t0 >= sm.cfg.turn_duration_s:
        sm._enter(State.RETURN)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)
    spd = min(sm.cfg.turn_speed, sm.cfg.max_speed)
    return ControlOutput(left=spd, right=-spd, claw=None, state=sm.state)
