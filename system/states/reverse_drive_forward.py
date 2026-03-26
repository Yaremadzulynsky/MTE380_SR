"""
REVERSE_DRIVE_FORWARD — drive backward the same distance as DRIVE_FORWARD.

Uses the position controller with -forward_drive_m (opposite sign to drive_forward).
Transitions to PICKUP when complete.
"""
from __future__ import annotations

from control.position_controller import PositionController
from states import ControlOutput, State


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if getattr(sm, "_rev_fwd_ctrl", None) is None:
        sm._rev_fwd_ctrl = PositionController(sm._brain, sm.cfg.forward_drive_m)

    sm._rev_fwd_ctrl.step()

    if sm._rev_fwd_ctrl.done:
        sm._rev_fwd_ctrl = None
        sm._enter(State.TURN_180)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
