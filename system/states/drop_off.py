"""
DROP_OFF state — three-phase sequence after the return leg:

  Phase 0  Turn 180° (RotationController)
  Phase 1  Drive forward dropoff_distance_m (PositionController)
  Phase 2  Open claw (one tick) → DONE

Reset _dropoff_phase = None before calling _enter(State.DROP_OFF) to guarantee
a clean start when re-entering the state.
"""
from __future__ import annotations

from control.rotation_controller import RotationController
from control.position_controller import PositionController
from states import ControlOutput, State


def step(sm) -> ControlOutput:
    # Initialise phase on first tick after _enter
    if getattr(sm, "_dropoff_phase", None) is None:
        sm._dropoff_phase = 0
        sm._dropoff_ctrl  = None

    phase = sm._dropoff_phase

    # ── Phase 0: turn 180° ────────────────────────────────────────────────────
    if phase == 0:
        if sm._dropoff_ctrl is None:
            sm._dropoff_ctrl = RotationController(sm._brain, 180.0)
        sm._dropoff_ctrl.step()
        if sm._dropoff_ctrl.done:
            sm._dropoff_ctrl  = None
            sm._dropoff_phase = 1
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

    # ── Phase 1: drive forward ────────────────────────────────────────────────
    if phase == 1:
        if sm._dropoff_ctrl is None:
            sm._dropoff_ctrl = PositionController(sm._brain, sm.cfg.dropoff_distance_m)
        sm._dropoff_ctrl.step()
        if sm._dropoff_ctrl.done:
            sm._dropoff_ctrl  = None
            sm._dropoff_phase = 2
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)

    # ── Phase 2: open claw → DONE ─────────────────────────────────────────────
    sm._dropoff_phase = None
    sm._enter(State.DONE)
    return ControlOutput(left=0.0, right=0.0, claw=sm.cfg.claw_open, state=sm.state)
