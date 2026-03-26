"""
DROP_OFF state — keep line-following while counting encoder distance.

As soon as green is detected (transition from LINE_FOLLOW), the robot
continues line-following using the normal steering logic.  Encoder
displacement is measured from the moment DROP_OFF was entered.  Once
dropoff_distance_m is reached, the claw opens and the mission ends.

If the line is briefly lost, the steering output from the sub-module is
still used but any state transition it attempts is suppressed so the
encoder baseline is preserved.
"""
from __future__ import annotations

import math

from control.brain import TICKS_PER_REV
from states import ControlOutput, State
import states.line_follow_find         as _find
import states.line_follow_find_heading as _find_heading


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    wheel_circ  = math.pi * sm.cfg.wheel_diameter_m
    dl = (left_ticks  - sm._enc0_left)  * wheel_circ / TICKS_PER_REV
    dr = (right_ticks - sm._enc0_right) * wheel_circ / TICKS_PER_REV
    displacement = (dl + dr) / 2.0

    if displacement >= sm.cfg.dropoff_distance_m:
        sm._enter(State.DONE)
        return ControlOutput(left=0.0, right=0.0, claw=sm.cfg.claw_open, state=sm.state)

    # Keep line-following for steering; suppress any state transitions the
    # sub-module tries to make (e.g. FIND_LINE on brief loss) so that our
    # encoder baseline (_enc0_*) is not reset by a spurious _enter call.
    if sm.cfg.line_follow_mode == 4:
        out = _find_heading.step(sm, det, left_ticks, right_ticks)
    else:
        out = _find.step(sm, det, left_ticks, right_ticks)

    if sm.state != State.DROP_OFF:
        sm.state = State.DROP_OFF   # revert without resetting encoder baseline

    return out
