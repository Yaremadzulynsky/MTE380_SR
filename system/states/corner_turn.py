"""
CORNER_TURN state — triggered when curvature exceeds the configured threshold.

Two internal phases
───────────────────
  forward  PositionController drives straight corner_forward_m metres, then →
  rotate   RotationController turns 90° toward the detected curvature direction

Both controllers call brain.send_voltage() directly; the state returns
ControlOutput(skip=True) so _mission_tick does not overwrite those voltages.
After the rotation reaches tolerance the state transitions back to LINE_FOLLOW.
"""
from __future__ import annotations

from states import ControlOutput, State


def on_enter(sm, det, left_ticks: int, right_ticks: int) -> None:
    """Stash corner-specific state on the SM before the generic _enter() call."""
    from control.position_controller import PositionController
    from control.rotation_controller import RotationController

    # +90° = CW (curves right);  −90° = CCW (curves left)
    degrees = 90.0 if det.curvature >= 0.0 else -90.0

    sm._corner_phase    = "forward"
    sm._corner_pos_ctrl = PositionController(sm._brain, sm.cfg.corner_forward_m)
    sm._corner_rot_ctrl = RotationController(sm._brain, degrees)


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    if sm._corner_phase == "forward":
        return _step_forward(sm, left_ticks, right_ticks)
    return _step_rotate(sm, left_ticks, right_ticks)


def _step_forward(sm, left_ticks: int, right_ticks: int) -> ControlOutput:
    ctrl = sm._corner_pos_ctrl
    ctrl.step()  # sends voltage directly via brain

    if ctrl.done:
        sm._corner_phase = "rotate"
        traveled, target = ctrl.progress()
        print(
            f"[corner] forward done  traveled={traveled:.3f}m  target={target:.3f}m",
            flush=True,
        )
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)


def _step_rotate(sm, left_ticks: int, right_ticks: int) -> ControlOutput:
    ctrl = sm._corner_rot_ctrl
    ctrl.step()  # sends voltage directly via brain

    if ctrl.done:
        traveled, target = ctrl.progress()
        print(
            f"[corner] rotation done  traveled={traveled:.1f}°  target={target:.1f}°",
            flush=True,
        )
        sm._enter(State.LINE_FOLLOW, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
