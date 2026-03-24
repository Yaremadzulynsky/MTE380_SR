"""LINE_FOLLOW mode 3 — adaptive blend between line error and horizontal bias.

adaptive_curv_a is a single gain that controls both:
  forward_speed = base_speed - a * |curvature|       (clamped to [min_speed, base_speed])
  blended_error = red_error + a * coverage_loss * horiz_bias

where coverage_loss = 1 - curve_conf (fraction of vertical strips without red).
horiz_bias = +1 if tape centroid is right of centre, -1 if left.
"""
from __future__ import annotations

from states import ControlOutput, State, steer, _clamp


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if det.blue_found:
        sm._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    if not det.red_found:
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    sm._last_curvature = det.curvature

    # Track horizontal side for FIND_LINE recovery if needed
    if det.tape_cx_px is not None:
        sm._find_line_turn_cw = det.tape_cx_px > sm.cfg.camera_width / 2.0

    # Vertical coverage loss: 0 = full coverage, 1 = no strips detected
    coverage_loss = 1.0 - _clamp(det.curve_conf, 0.0, 1.0)

    # Horizontal bias: +1 right, -1 left (same side decision as FIND_LINE)
    if det.tape_cx_px is not None:
        horiz_bias = 1.0 if det.tape_cx_px > sm.cfg.camera_width / 2.0 else -1.0
    else:
        horiz_bias = 0.0

    a = sm.cfg.adaptive_curv_a

    # Steering: red_error + a * coverage_loss * horiz_bias
    blended = _clamp(det.red_error + a * coverage_loss * horiz_bias, -1.0, 1.0)

    # Speed: forward_speed = base_speed - a * |curvature|
    adj_base = _clamp(sm.cfg.base_speed - a * abs(det.curvature), sm.cfg.min_speed, sm.cfg.base_speed)

    left, right = steer(sm, blended, base_speed=adj_base)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
