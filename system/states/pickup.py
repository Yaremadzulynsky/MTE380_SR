from __future__ import annotations

import time

from states import ControlOutput, State


def step(sm) -> ControlOutput:
    if time.time() - sm._t0 >= sm.cfg.pickup_hold_s:
        sm._enter(State.REVERSE_DRIVE_FORWARD)
    return ControlOutput(left=0.0, right=0.0, claw=sm.cfg.claw_closed, state=sm.state)
