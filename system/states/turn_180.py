from __future__ import annotations

import time

from control.rotation_controller import RotationController
from states import ControlOutput, State


_WAIT_S = 1  # motors-off pause before spinning


def step(sm) -> ControlOutput:
    # Initialise timer on first tick after entering this state
    if not hasattr(sm, "_turn180_t") or sm._turn180_t is None:
        sm._turn180_t = time.monotonic()

    elapsed = time.monotonic() - sm._turn180_t

    # Wait phase — hold still before spinning
    if elapsed < _WAIT_S:
        sm._turn180_ctrl = None
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, direct_voltage=True)

    # Spin phase — PID rotation controller
    if sm._turn180_ctrl is None:
        sm._turn180_ctrl = RotationController(sm._brain, 180)

    ctrl = sm._turn180_ctrl
    ctrl.step()

    if ctrl.done:
        sm._turn180_t = None
        sm._turn180_ctrl = None
        next_state = getattr(sm, "_turn180_next", None)
        if next_state == State.DRIVE_FORWARD:
            sm._turn180_next = None
            sm._enter(State.DRIVE_FORWARD)
        else:
            sm.is_returning = True
            sm._enter(State.LINE_FOLLOW)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
