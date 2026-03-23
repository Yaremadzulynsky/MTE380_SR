from __future__ import annotations

from states import ControlOutput, State, _clamp, clamp_search_turn, search_spin_pair, steer


def step(sm, det) -> ControlOutput:
    if det.t_junction:
        sm._enter(State.DONE)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    if not det.red_found:
        sm._consecutive_lost += 1
        n = max(1, sm.cfg.lost_frames_before_search)
        if sm._consecutive_lost < n:
            c = _clamp(max(0.0, sm.cfg.lost_line_coast_speed), 0.0, sm.cfg.max_speed)
            return ControlOutput(left=c, right=c, claw=None, state=sm.state)
        sm._steer_pid.reset()
        st = clamp_search_turn(sm)
        left, right = search_spin_pair(sm, st)
        return ControlOutput(left=left, right=right, claw=None, state=sm.state)

    sm._consecutive_lost = 0
    sm._last_red_error = det.red_error
    left, right = steer(sm, det.red_error)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
