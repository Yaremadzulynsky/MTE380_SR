"""LINE_FOLLOW mode 3 — adaptive blend between line error and horizontal bias.

adaptive_curv_a is a 0–1 tradeoff between forward driving and lateral recovery:
  a=0  full forward at base_speed, steer purely on red_error
  a=1  forward=0, steer purely on coverage_loss * horiz_bias (spin toward more-red side)

  forward   = base_speed * (1 - a)
  steer_err = (1-a) * red_error  +  a * coverage_loss * horiz_bias

coverage_loss = 1 - curve_conf  (0 = all strips see red, 1 = no strips see red)
horiz_bias    = +1 if tape centroid is right of frame centre, -1 if left
"""
from __future__ import annotations

from states import ControlOutput, State, scale_pair_to_max_speed, _clamp


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if det.blue_found:
        sm._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    if not det.red_found:
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    sm._last_curvature = det.curvature

    if det.tape_cx_px is not None:
        sm._find_line_turn_cw = det.tape_cx_px > sm.cfg.camera_width / 2.0

    # coverage_loss: 0 = full vertical coverage, 1 = no strips detected
    coverage_loss = 1.0 - _clamp(det.curve_conf, 0.0, 1.0)

    # horiz_bias: +1 right, -1 left
    if det.tape_cx_px is not None:
        horiz_bias = 1.0 if det.tape_cx_px > sm.cfg.camera_width / 2.0 else -1.0
    else:
        horiz_bias = 0.0

    a = _clamp(sm.cfg.adaptive_curv_a, 0.0, 1.0)

    # Blend error: red_error at a=0, horizontal bias at a=1
    blended = _clamp(
        (1.0 - a) * det.red_error + a * coverage_loss * horiz_bias,
        -1.0, 1.0,
    )

    # Steering PID on blended error
    turn = sm._steer_pid(-blended)
    turn = _clamp(turn, -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)

    # Forward speed: base_speed at a=0, 0 at a=1 (bypass steer() to avoid min_speed floor)
    fwd = sm.cfg.base_speed * (1.0 - a)

    left  = _clamp(fwd + turn, -1.0, 1.0)
    right = _clamp(fwd - turn, -1.0, 1.0)
    left, right = scale_pair_to_max_speed(sm, left, right)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
