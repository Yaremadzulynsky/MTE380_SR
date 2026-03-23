from __future__ import annotations

from states import ControlOutput, State, steer
import states.corner_turn as _corner_turn


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    # Corner detected → drive forward then turn 90°
    # if (det.red_found
    #         and abs(det.curvature) >= sm.cfg.corner_curvature_thresh
    #         and det.curve_conf >= sm.cfg.corner_curve_conf_min):
    #     _corner_turn.on_enter(sm, det, left_ticks, right_ticks)
    #     sm._enter(State.CORNER_TURN, left_ticks, right_ticks)
    #     return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # Blue target visible → stop steering and drive straight to pickup point
    if det.blue_found:
        sm._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # No line — transition to FIND_LINE to spin toward last known curvature
    # if not det.red_found:
    #     sm._enter(State.FIND_LINE, left_ticks, right_ticks)
    #     return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # Line found — store curvature for FIND_LINE recovery, then steer
    sm._last_curvature = det.curvature
    left, right = steer(sm, det.red_error)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
