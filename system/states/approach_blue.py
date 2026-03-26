"""
APPROACH_BLUE — drive toward the blue circle until it is large enough to stop at.

  • Drives forward at cfg.approach_blue_speed.
  • Steers using the heading PID on the blue circle centroid (blue_cx_norm).
  • Exits to TURN_180 (with _turn180_next=DRIVE_FORWARD) once the fitted
    circle radius reaches cfg.approach_blue_min_r_px.
  • If blue is momentarily lost the robot keeps its last steering and continues.
"""
from __future__ import annotations

from states import ControlOutput, State, _clamp


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    fwd = sm.cfg.approach_blue_speed

    # ── Exit condition ────────────────────────────────────────────────────────
    if det.blue_found and det.blue_circle_r is not None:
        if det.blue_circle_r >= sm.cfg.approach_blue_min_r_px:
            sm._enter(State.BACKUP, left_ticks, right_ticks)
            return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    # ── Heading correction toward blue centroid ───────────────────────────────
    if det.blue_found and det.blue_cx_norm is not None:
        turn = sm._heading_pid(-det.blue_cx_norm)
        turn = _clamp(turn, -sm.cfg.steer_out_limit, sm.cfg.steer_out_limit)
    else:
        turn = 0.0

    left  = _clamp(-fwd + turn, -1.0, 1.0)
    right = _clamp(-fwd - turn, -1.0, 1.0)
    return ControlOutput(left=left, right=right, claw=None, state=sm.state)
