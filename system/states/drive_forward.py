"""
DRIVE_FORWARD — aligns with the red line using the heading PID then drives
forward_distance_m using the position controller.

The heading PID (curve_heading) steers the robot to stay aligned with the
line tangent while driving.  If red is not visible the robot drives straight.
"""
from __future__ import annotations

from control.position_controller import PositionController
from states import ControlOutput, State, _clamp


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if not hasattr(sm, "_fwd_ctrl") or sm._fwd_ctrl is None:
        sm._fwd_ctrl = PositionController(sm._brain, sm.cfg.forward_distance_m)

    if det.red_found:
        steer = _clamp(sm._heading_pid(-det.curve_heading), -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)
    else:
        steer = 0.0

    sm._fwd_ctrl.step(steer=steer)

    if sm._fwd_ctrl.done:
        sm._fwd_ctrl = None
        sm._enter(State.PICKUP)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
