"""
DRIVE_FORWARD — drives forward_distance_m while steering to keep the blue
target centred in the frame.

The PositionController handles distance tracking and forward speed via its PID.
A proportional correction (align_kp * blue_cx_norm) is passed as the steer
argument each tick: left = fwd + steer, right = fwd - steer.
If blue is not visible the robot still drives straight (steer = 0).
"""
from __future__ import annotations

from control.position_controller import PositionController
from states import ControlOutput, State, _clamp


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if sm._fwd_ctrl is None:
        sm._fwd_ctrl = PositionController(sm._brain, sm.cfg.forward_distance_m)

    cx = det.blue_cx_norm  # None or float in [-1, 1], positive = blue is right
    steer = _clamp(sm.cfg.align_kp * cx, -1.0, 1.0) if cx is not None else 0.0

    sm._fwd_ctrl.step(steer=steer)

    if sm._fwd_ctrl.done:
        sm._fwd_ctrl = None
        sm._enter(State.PICKUP)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
