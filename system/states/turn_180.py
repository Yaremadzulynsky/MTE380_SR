from __future__ import annotations

import time

from states import ControlOutput, State


def step(sm) -> ControlOutput:
    ControlOutput(left=0, right=0)
    # Initialise timer on first tick after entering this state
    # if not hasattr(sm, "_turn180_t") or sm._turn180_t is None:
    #     sm._turn180_t = time.monotonic()

    # elapsed = time.monotonic() - sm._turn180_t
    # if elapsed >= sm.cfg.turn180_duration_s:
    #     sm._turn180_t = None
    #     next_state = getattr(sm, "_turn180_next", None)
    #     if next_state == State.DRIVE_FORWARD:
    #         sm._turn180_next = None
    #         sm._enter(State.DRIVE_FORWARD)
    #     else:
    #         sm.is_returning = True
    #         sm._enter(State.LINE_FOLLOW)
    #     return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # spd = sm.cfg.turn180_speed
    # return ControlOutput(left=-spd, right=spd, claw=None, state=sm.state, direct_voltage=True)
