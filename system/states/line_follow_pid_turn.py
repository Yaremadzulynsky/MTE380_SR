"""LINE_FOLLOW mode 2 — steer along line, slow on curvature, enter PID_TURN on sharp corner."""
from __future__ import annotations

from states import ControlOutput, State, steer, _clamp
import states.pid_turn as _pid_turn


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    # Sharp corner detected → forward + 90° rotation via PID controllers
    if (det.red_found
            and abs(det.curvature) >= sm.cfg.corner_curvature_thresh
            and det.curve_conf >= sm.cfg.corner_curve_conf_min):
        _pid_turn.on_enter(sm, det, left_ticks, right_ticks)
        sm._enter(State.PID_TURN, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    if det.blue_found:
        sm._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    if not det.red_found:
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    sm._last_curvature = det.curvature
    curv_scale = _clamp(1.0 - abs(det.curvature) / sm.cfg.corner_curvature_thresh, 0.0, 1.0)
    adj_base   = sm.cfg.min_speed + (sm.cfg.base_speed - sm.cfg.min_speed) * curv_scale
    left, right = steer(sm, det.red_error, base_speed=adj_base)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
