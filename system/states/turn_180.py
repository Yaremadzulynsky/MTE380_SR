"""
TURN_180 — spin in place until the red line is reacquired.

Behaviour:
  - Spins CCW (negative = CCW in RotationController convention).  A large
    target (-720°) is used so the controller never naturally terminates.
  - The line-finder is inactive for the first find_line_min_angle_deg degrees
    to avoid re-acquiring the line the robot just reversed off of.
  - Once past that guard, the first frame where det.red_found is True stops
    the spin and transitions to LINE_FOLLOW (is_returning=True).
"""
from __future__ import annotations

from control.rotation_controller import RotationController
from states import ControlOutput, State

_SPIN_TARGET_DEG = -720.0   # large CCW target so the controller never self-terminates


def _finish(sm, left_ticks: int, right_ticks: int) -> ControlOutput:
    """Stop speed controller, clear controller, enter next state."""
    sm._turn180_ctrl = None
    sm._brain._speed_ctrl.reset()
    next_state = getattr(sm, "_turn180_next", None)
    if next_state == State.DRIVE_FORWARD:
        sm._turn180_next = None
        sm._enter(State.DRIVE_FORWARD)
    else:
        sm.is_returning = True
        sm._enter(State.LINE_FOLLOW)
    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if getattr(sm, "_turn180_ctrl", None) is None:
        sm._turn180_ctrl = RotationController(sm._brain, _SPIN_TARGET_DEG)

    ctrl = sm._turn180_ctrl
    ctrl.step()

    degrees_turned, _ = ctrl.progress()
    if degrees_turned >= sm.cfg.find_line_min_angle_deg and det.red_found:
        print(f"[turn_180] red line found at {degrees_turned:.1f}°", flush=True)
        return _finish(sm, left_ticks, right_ticks)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
