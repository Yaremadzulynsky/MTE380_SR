"""
DRIVE_FORWARD — stops for 5 s on entry, then drives forward for
forward_drive_s seconds using the speed PID at base_speed.

The heading PID (curve_heading) steers the robot to stay aligned with the
red line tangent while driving.  If red is not visible the robot drives straight.
"""
from __future__ import annotations
import time

from states import ControlOutput, State, _clamp

_STOP_DURATION_S = 5.0


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    # ── Entry stop ────────────────────────────────────────────────────────────
    if not hasattr(sm, "_fwd_stop_until"):
        sm._fwd_stop_until  = time.monotonic() + _STOP_DURATION_S
        sm._fwd_drive_until = None

    if time.monotonic() < sm._fwd_stop_until:
        sm._brain.idle()
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

    # ── Timed drive ───────────────────────────────────────────────────────────
    if sm._fwd_drive_until is None:
        sm._fwd_drive_until = time.monotonic() + sm.cfg.forward_drive_s

    if time.monotonic() >= sm._fwd_drive_until:
        del sm._fwd_stop_until
        del sm._fwd_drive_until
        sm._enter(State.PICKUP)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    steer = 0.0
    if det.red_found:
        steer = _clamp(sm._heading_pid(-det.curve_heading), -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)

    fwd = sm.cfg.base_speed
    left  = _clamp(fwd + steer, -1.0, 1.0)
    right = _clamp(fwd - steer, -1.0, 1.0)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
