"""LINE_FOLLOW mode 3 — adaptive blend between line error and horizontal bias.

As vertical red coverage drops (fewer strips with red → curve_conf decreases)
the error blends from the normal centroid-based red_error toward a hard bias
that steers toward whichever horizontal side of the frame has more red.

  blended = curve_conf * red_error + (1 - curve_conf) * horiz_bias

where horiz_bias = +1 if tape centroid is right of frame centre, -1 if left
(same logic as FIND_LINE).  Speed is reduced on curvature as in line_follow_find.
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

    # Vertical coverage: fraction of strips that had red (0 = none, 1 = all)
    vert_frac = _clamp(det.curve_conf, 0.0, 1.0)

    # Horizontal bias: +1 right, -1 left (same side decision as FIND_LINE)
    if det.tape_cx_px is not None:
        horiz_bias = 1.0 if det.tape_cx_px > sm.cfg.camera_width / 2.0 else -1.0
    else:
        horiz_bias = 0.0

    # Blend: full red_error when line fills frame, full bias when line is thin/fading
    blended = _clamp(
        vert_frac * det.red_error + (1.0 - vert_frac) * sm.cfg.adaptive_horiz_weight * horiz_bias,
        -1.0, 1.0,
    )

    curv_scale = _clamp(1.0 - abs(det.curvature) / sm.cfg.corner_curvature_thresh, 0.0, 1.0)
    adj_base   = sm.cfg.min_speed + (sm.cfg.base_speed - sm.cfg.min_speed) * curv_scale
    left, right = steer(sm, blended, base_speed=adj_base)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
