"""
TURN_TO_LINE — vision-based turn to reacquire and align with the red line.

Three phases:
  1. losing  — spin until red is no longer seen (line has passed out of view)
  2. finding — keep spinning until red is seen again (line reacquired)
  3. aligning — use heading PID to zero out curve_heading

Spin direction is CW by default; set sm._turn_to_line_cw = False before
entering to spin CCW.

Transitions to DRIVE_FORWARD when aligned.
"""
from __future__ import annotations

from states import ControlOutput, State, _clamp


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if not hasattr(sm, "_ttl_phase"):
        sm._ttl_phase = "losing"

    v   = sm.cfg.find_line_turn_speed
    cw  = getattr(sm, "_turn_to_line_cw", True)
    lv  =  v if cw else -v
    rv  = -v if cw else  v

    # ── Phase 1: spin until red is gone ──────────────────────────────────────
    if sm._ttl_phase == "losing":
        if not det.red_found:
            sm._ttl_phase = "finding"
        sm._brain._speed_ctrl.set_target(lv, rv)
        sm._brain._speed_ctrl.step()
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

    # ── Phase 2: spin until red is found again ────────────────────────────────
    if sm._ttl_phase == "finding":
        if det.red_found and det.red_blob_area >= sm.cfg.find_line_min_blob_px:
            sm._ttl_phase = "aligning"
            sm._steer_pid.reset()
            sm._heading_pid.reset()
        else:
            sm._brain._speed_ctrl.set_target(lv, rv)
            sm._brain._speed_ctrl.step()
            return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

    # ── Phase 3: align with line heading ─────────────────────────────────────
    if not det.red_found:
        sm._ttl_phase = "finding"
        sm._brain._speed_ctrl.set_target(lv, rv)
        sm._brain._speed_ctrl.step()
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

    if abs(det.curve_heading) < sm.cfg.forward_align_thresh:
        del sm._ttl_phase
        sm._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    turn = _clamp(sm._heading_pid(-det.curve_heading), -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)
    sm._brain._speed_ctrl.set_target(turn, -turn)
    sm._brain._speed_ctrl.step()
    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
