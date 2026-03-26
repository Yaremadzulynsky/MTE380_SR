"""
DRIVE_FORWARD — three phases:
  1. Stop for 1 s (settle after blue detection) — speed PID at 0 to actively brake.
  2. Align in place using the heading PID until curve_heading < forward_align_thresh.
  3. Drive forward forward_drive_m metres using PositionController.
"""
from __future__ import annotations
import time

import copy

from control.position_controller import PositionController
from states import ControlOutput, State, _clamp

_STOP_DURATION_S = 1


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    # ── Phase 1: entry stop (active brake via speed PID at 0) ─────────────────
    if not hasattr(sm, "_fwd_phase"):
        sm._fwd_phase      = "stop"
        sm._fwd_stop_until = time.monotonic() + _STOP_DURATION_S
        sm._fwd_pos_ctrl   = None

    if sm._fwd_phase == "stop":
        if time.monotonic() >= sm._fwd_stop_until:
            sm._fwd_phase = "align"
            sm._heading_pid.reset()
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # ── Phase 2: align in place with line heading ─────────────────────────────
    if sm._fwd_phase == "align":
        sm._fwd_phase    = "drive"
        approach_cfg = copy.copy(sm.cfg)
        approach_cfg.pos_max_speed = sm.cfg.blue_approach_speed
        sm._fwd_pos_ctrl = PositionController(sm._brain, -sm.cfg.blue_approach_m, cfg=approach_cfg)

    # ── Phase 3: position-controlled drive ────────────────────────────────────
    ctrl = sm._fwd_pos_ctrl
    ctrl.step()

    if ctrl.done:
        del sm._fwd_phase
        del sm._fwd_stop_until
        del sm._fwd_pos_ctrl
        sm._enter(State.PICKUP)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
