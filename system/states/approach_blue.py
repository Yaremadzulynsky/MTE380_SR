"""
APPROACH_BLUE — line-follow until the blue circle centroid enters the bottom half of the frame.

Delegates entirely to line_follow_find_heading each tick, but intercepts the exit
condition: once det.blue_circle_cy > half the processing frame height, transitions
to DRIVE_FORWARD.
"""
from __future__ import annotations

from states import ControlOutput, State
import states.line_follow_find_heading as _find_heading

_FRAME_MID_Y = 70  # half of the 140 px processing height


def step(sm, det, left_ticks: int, right_ticks: int) -> ControlOutput:
    # Exit when blue circle centroid is in the bottom half of the frame
    if det.blue_found and det.blue_circle_cy is not None:
        if det.blue_circle_cy > _FRAME_MID_Y:
            sm._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
            return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)

    return _find_heading.step(sm, det, left_ticks, right_ticks)
