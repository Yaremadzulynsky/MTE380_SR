from __future__ import annotations

from control.rotation_controller import RotationController
from states import ControlOutput, State


def step(sm) -> ControlOutput:
    # Initialise controller on first tick after entering this state
    if not hasattr(sm, "_turn180_ctrl") or sm._turn180_ctrl is None:
        sm._turn180_ctrl = RotationController(sm._brain, 180.0)

    ctrl = sm._turn180_ctrl
    ctrl.step()

    if ctrl.done:
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
