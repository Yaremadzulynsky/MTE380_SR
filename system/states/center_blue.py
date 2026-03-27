"""
CENTER_BLUE — rotate and drive until the blue circle centroid is centred in both axes.

Two independent PIDs run simultaneously:
  X-axis (rotational): blue_cx_norm drives differential turn — spins the robot
    until the circle is horizontally centred (left/right).
  Y-axis (forward):    blue_cy_norm drives both wheels equally — drives the robot
    forward/backward until the circle is vertically centred (near/far).
    cy_norm > 0 → circle below centre → too close → back up.
    cy_norm < 0 → circle above centre → too far  → drive forward.

Transitions to DRIVE_FORWARD once both |cx_norm| <= blue_center_tolerance
and |cy_norm| <= blue_center_y_tolerance.

If blue is lost for a tick the robot holds position.
"""
from __future__ import annotations

from states import ControlOutput, State, _clamp


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if not det.blue_found or det.blue_cx_norm is None or det.blue_cy_norm is None:
        # Blue temporarily lost — hold still, preserve integrals
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    x_err = det.blue_cx_norm   # +ve = blue right of centre
    y_err = det.blue_cy_norm   # +ve = blue below centre = too close

    x_done = abs(x_err) <= sm.cfg.blue_center_tolerance
    y_done = abs(y_err) <= sm.cfg.blue_center_y_tolerance

    if x_done and y_done:
        sm._center_blue_integral     = 0.0
        sm._center_blue_prev_error   = 0.0
        sm._center_blue_y_integral   = 0.0
        sm._center_blue_y_prev_error = 0.0
        sm._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # ── X PID (rotational) ────────────────────────────────────────────────────
    sm._center_blue_integral   = getattr(sm, "_center_blue_integral",   0.0) + x_err
    x_prev                     = getattr(sm, "_center_blue_prev_error", x_err)
    sm._center_blue_prev_error = x_err
    turn = _clamp(
        sm.cfg.blue_center_kp * x_err
        + sm.cfg.blue_center_ki * sm._center_blue_integral
        + sm.cfg.blue_center_kd * (x_err - x_prev),
        -sm.cfg.blue_center_speed, sm.cfg.blue_center_speed,
    )

    # ── Y PID (forward / backward) ────────────────────────────────────────────
    # Negate so that positive y_err (circle below = too close) → back up (negative fwd).
    sm._center_blue_y_integral   = getattr(sm, "_center_blue_y_integral",   0.0) + y_err
    y_prev                       = getattr(sm, "_center_blue_y_prev_error", y_err)
    sm._center_blue_y_prev_error = y_err
    fwd = _clamp(
        -(sm.cfg.blue_center_y_kp * y_err
          + sm.cfg.blue_center_y_ki * sm._center_blue_y_integral
          + sm.cfg.blue_center_y_kd * (y_err - y_prev)),
        -sm.cfg.blue_center_y_speed, sm.cfg.blue_center_y_speed,
    )

    # Combine: turn differentially, fwd equally to both wheels.
    spd   = max(sm.cfg.blue_center_speed, sm.cfg.blue_center_y_speed)
    left  = _clamp(fwd + turn, -spd, spd)
    right = _clamp(fwd - turn, -spd, spd)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
