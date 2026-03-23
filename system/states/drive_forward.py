from __future__ import annotations

from states import ControlOutput, State


def step(sm, left_ticks: int, right_ticks: int) -> ControlOutput:
    avg_delta = ((left_ticks - sm._enc0_left) + (right_ticks - sm._enc0_right)) // 2
    if abs(avg_delta) >= sm.cfg.forward_ticks:
        sm._enter(State.PICKUP)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=sm.state)
    spd = min(sm.cfg.forward_speed, sm.cfg.max_speed)
    return ControlOutput(left=spd, right=spd, claw=None, state=sm.state)
