"""
DRIVE_FORWARD — three phases:
  1. Stop for 1 s (settle after blue detection) — speed PID at 0 to actively brake.
  2. Align in place using the heading PID until curve_heading < forward_align_thresh.
  3. Drive forward for forward_drive_s seconds at forward_drive_speed.
"""
from __future__ import annotations
import time

from states import ControlOutput, State, _clamp

_STOP_DURATION_S = 1


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    # ── Phase 1: entry stop (active brake via speed PID at 0) ─────────────────
    if not hasattr(sm, "_fwd_phase"):
        sm._fwd_phase       = "stop"
        sm._fwd_stop_until  = time.monotonic() + _STOP_DURATION_S
        sm._fwd_drive_until = None

    if sm._fwd_phase == "stop":
        if time.monotonic() >= sm._fwd_stop_until:
            sm._fwd_phase = "align"
            sm._heading_pid.reset()
        # Return 0 speed through the speed PID (active braking, not coast)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # ── Phase 2: align in place with line heading ─────────────────────────────
    if sm._fwd_phase == "align":
        # if not det.red_found or abs(det.curve_heading) < sm.cfg.forward_align_thresh:
        #     sm._fwd_phase       = "drive"
        #     sm._fwd_drive_until = time.monotonic() + sm.cfg.forward_drive_s
        # else:
        #     turn = _clamp(sm._heading_pid(-det.curve_heading), -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)
        #     return ControlOutput(left=turn, right=-turn, claw=None, state=sm.state)
        sm._fwd_phase       = "drive"
        sm._fwd_drive_until = time.monotonic() + sm.cfg.forward_drive_s

    # ── Phase 3: timed drive ──────────────────────────────────────────────────
    if time.monotonic() >= sm._fwd_drive_until:
        del sm._fwd_phase
        del sm._fwd_stop_until
        del sm._fwd_drive_until
        sm._enter(State.PICKUP)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    fwd = sm.cfg.forward_drive_speed
    return ControlOutput(left=fwd, right=fwd, claw=None, state=sm.state)
