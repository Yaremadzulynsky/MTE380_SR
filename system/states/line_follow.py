from __future__ import annotations

from states import ControlOutput, State, clamp_search_turn, search_spin_pair, steer


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    # Blue target visible → stop steering and drive straight to pickup point
    if det.blue_found:
        sm._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    if not det.red_found:
        sm._consecutive_lost += 1
        n = max(1, sm.cfg.lost_frames_before_search)

        if sm._consecutive_lost < n:
            # Soft loss: brief flicker or tape gap — coast straight at min_speed
            # and keep the steering PID warm so it's ready when the line reappears
            return ControlOutput(left=sm.cfg.min_speed, right=sm.cfg.min_speed, claw=None, state=sm.state)

        # Hard loss: line gone for too many frames — spin toward the side it was
        # last seen on (sign of _last_red_error) to try to reacquire it
        sm._steer_pid.reset()
        st = clamp_search_turn(sm)
        left, right = search_spin_pair(sm, st)
        return ControlOutput(left=left, right=right, claw=None, state=sm.state)

    # Line found — reset loss counter and record error for use during search spin
    sm._consecutive_lost = 0
    sm._last_red_error = det.red_error

    # Run steering PID: error → differential turn correction + forward speed scaling
    left, right = steer(sm, det.red_error)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
