from __future__ import annotations

import time

from control.rotation_controller import RotationController
from states import ControlOutput, State, _clamp, scale_pair_to_max_speed


_WAIT_S = 1  # motors-off pause before spinning


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    # Phase 0: Line-follow until curve_heading is within threshold
    if not getattr(sm, "_turn180_aligned", False):
        if det is not None and det.red_found and abs(det.curve_heading) > sm.cfg.turn180_align_thresh:
            lateral_turn = sm._steer_pid(-det.red_error)
            lateral_turn = _clamp(lateral_turn, -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)
            fwd = sm.cfg.base_speed
            left  = _clamp(-fwd + lateral_turn, -1.0, 1.0)
            right = _clamp(-fwd - lateral_turn, -1.0, 1.0)
            left, right = scale_pair_to_max_speed(sm, left, right)
            return ControlOutput(left=left, right=right, claw=None, state=sm.state)
        sm._turn180_aligned = True
        sm._turn180_t = None
        sm._turn180_ctrl = None

    # Phase 1: Initialise wait timer
    if sm._turn180_t is None:
        sm._turn180_t = time.monotonic()

    elapsed = time.monotonic() - sm._turn180_t

    # Wait phase — hold still before spinning
    if elapsed < _WAIT_S:
        sm._turn180_ctrl = None
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, direct_voltage=True)

    # Phase 2: Spin — PID rotation controller
    if sm._turn180_ctrl is None:
        sm._turn180_ctrl = RotationController(sm._brain, 180)

    ctrl = sm._turn180_ctrl
    ctrl.step()

    if ctrl.done:
        sm._turn180_t = None
        sm._turn180_ctrl = None
        sm._turn180_aligned = False
        next_state = getattr(sm, "_turn180_next", None)
        if next_state == State.DRIVE_FORWARD:
            sm._turn180_next = None
            sm._enter(State.DRIVE_FORWARD)
        else:
            sm.is_returning = True
            sm._enter(State.LINE_FOLLOW)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state, skip=True)
