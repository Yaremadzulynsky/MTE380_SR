from __future__ import annotations

from control.position_controller import PositionController
from states import ControlOutput, State


def step(sm, left_ticks: int, right_ticks: int) -> ControlOutput:
    # Initialise controller on first tick after entering this state
    if not hasattr(sm, "_fwd_ctrl") or sm._fwd_ctrl is None:
        sm._fwd_ctrl = PositionController(sm._brain, sm.cfg.forward_distance_m)

    ctrl = sm._fwd_ctrl
    ctrl.step()

    if ctrl.done:
        sm._fwd_ctrl = None
        sm._enter(State.PICKUP)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
