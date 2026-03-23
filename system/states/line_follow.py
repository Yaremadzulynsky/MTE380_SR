from __future__ import annotations

from states import ControlOutput, State, _clamp, clamp_search_turn, search_spin_pair, steer
import states.corner_turn as _corner_turn


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    # Corner detected → drive forward then turn 90°
    if (det.red_found
            and abs(det.curvature) >= sm.cfg.corner_curvature_thresh
            and det.curve_conf >= sm.cfg.corner_curve_conf_min):
        _corner_turn.on_enter(sm, det, left_ticks, right_ticks)
        sm._enter(State.CORNER_TURN, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # Blue target visible → stop steering and drive straight to pickup point
    if det.blue_found:
        sm._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    if not det.red_found:
        sm._consecutive_lost += 1
        n = max(1, sm.cfg.lost_frames_before_search)

        if sm._consecutive_lost < n:
            # Soft loss: coast at lost_line_coast_speed (0 = stop in place until search spin)
            c = _clamp(max(0.0, sm.cfg.lost_line_coast_speed), 0.0, sm.cfg.max_speed)
            return ControlOutput(left=c, right=c, claw=None, state=sm.state)

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
