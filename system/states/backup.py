"""
BACKUP — drive forward/backward a fixed distance before TURN_180.

Uses the position controller (cfg.pre_turn180_backup_m metres; positive = forward).
Exits to TURN_180 with _turn180_next=DRIVE_FORWARD when the move is complete.
"""
from __future__ import annotations

from control.position_controller import PositionController
from states import ControlOutput, State


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if not hasattr(sm, "_backup_ctrl") or sm._backup_ctrl is None:
        sm._backup_ctrl = PositionController(sm._brain, sm.cfg.pre_turn180_backup_m)

    sm._backup_ctrl.step()

    if sm._backup_ctrl.done:
        sm._backup_ctrl = None
        sm._turn180_next = State.DRIVE_FORWARD
        sm._enter(State.TURN_180, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
