from __future__ import annotations

from states import ControlOutput, State, steer, _clamp
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

    # No line — stop and wait
    if not det.red_found:
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # Line found — store curvature for FIND_LINE recovery, then steer
    sm._last_curvature = det.curvature

    # Reduce base speed as curvature grows: min_speed at corner_curvature_thresh, base_speed at 0
    curv_scale = _clamp(1.0 - abs(det.curvature) / sm.cfg.corner_curvature_thresh, 0.0, 1.0)
    adj_base = sm.cfg.min_speed + (sm.cfg.base_speed - sm.cfg.min_speed) * curv_scale

    left, right = steer(sm, det.red_error, base_speed=adj_base)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
