"""
TURN_180 — spin in place until the red line is reacquired, then align.

Three phases:

  1. spinning — RotationController spins CCW with a large (-720°) target so it
     never self-terminates.  The line-finder is inactive for the first
     find_line_min_angle_deg degrees to avoid re-acquiring the departure line.
     Once past that guard and det.red_found is True, motors are hard-stopped
     and the state moves to heading alignment.

  2. heading align — heading PID zeroes curve_heading (line tangent vs robot
     direction) with in-place differential drive.  When |curve_heading| drops
     below rot_tolerance (converted to radians) the state moves to lateral
     alignment.

  3. lateral align — drives forward slowly at min_speed with steer PID on
     red_error.  When |red_error| <= turn180_lateral_tolerance the state
     transitions to LINE_FOLLOW (is_returning=True).  If the line is lost at
     any point during phases 2 or 3, the state exits immediately.
"""
from __future__ import annotations

import math

from control.rotation_controller import RotationController
from states import ControlOutput, State, _clamp

_SPIN_TARGET_DEG = -720.0   # large CCW target so the controller never self-terminates


def _enter_next(sm, left_ticks: int, right_ticks: int) -> ControlOutput:
    """Transition out of TURN_180 into the next state."""
    next_state = getattr(sm, "_turn180_next", None)
    if next_state == State.DRIVE_FORWARD:
        sm._turn180_next = None
        sm._enter(State.DRIVE_FORWARD)
    else:
        sm.is_returning = True
        sm._enter(State.LINE_FOLLOW)
    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    phase = getattr(sm, "_turn180_phase", "spinning")

    # ── Phase 3: lateral alignment — drive forward slowly, steer to red_error=0
    if phase == "lateral":
        if not det.red_found:
            print("[turn_180] line lost during lateral align — exiting", flush=True)
            sm._turn180_phase = "spinning"
            return _enter_next(sm, left_ticks, right_ticks)

        if abs(det.red_error) <= sm.cfg.turn180_lateral_tolerance:
            print(f"[turn_180] lateral aligned (red_error={det.red_error:+.3f}) — exiting", flush=True)
            sm._turn180_phase = "spinning"
            return _enter_next(sm, left_ticks, right_ticks)

        spd  = sm.cfg.min_speed
        turn = _clamp(sm._steer_pid(det.red_error), -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)
        left  = _clamp(spd + turn, -1.0, 1.0)
        right = _clamp(spd - turn, -1.0, 1.0)
        sm._brain._speed_ctrl.set_target(left, right)
        sm._brain._speed_ctrl.step()
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

    # ── Phase 2: heading alignment — in-place rotation to zero curve_heading ──
    if phase == "heading":
        tol_rad = math.radians(sm.cfg.rot_tolerance)

        if not det.red_found:
            print("[turn_180] line lost during heading align — exiting", flush=True)
            sm._turn180_phase = "spinning"
            return _enter_next(sm, left_ticks, right_ticks)

        if abs(det.curve_heading) < tol_rad:
            print(f"[turn_180] heading aligned ({math.degrees(det.curve_heading):.1f}°) — lateral align", flush=True)
            sm._turn180_phase = "lateral"
            sm._steer_pid.reset()
            return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

        turn = _clamp(sm._heading_pid(-det.curve_heading),
                      -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)
        sm._brain._speed_ctrl.set_target(turn, -turn)
        sm._brain._speed_ctrl.step()
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

    # ── Phase 1: spin until line is found past the minimum angle ─────────────
    if getattr(sm, "_turn180_ctrl", None) is None:
        sm._turn180_ctrl = RotationController(sm._brain, _SPIN_TARGET_DEG)

    ctrl = sm._turn180_ctrl
    ctrl.step()

    degrees_turned, _ = ctrl.progress()
    if degrees_turned >= sm.cfg.find_line_min_angle_deg and det.red_found:
        print(f"[turn_180] red line found at {degrees_turned:.1f}° — heading align", flush=True)
        sm._turn180_ctrl  = None
        sm._turn180_phase = "heading"
        sm._brain.send_voltage(0.0, 0.0)   # hard-stop before handing off to PIDs
        sm._brain._speed_ctrl.reset()
        sm._heading_pid.reset()
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
