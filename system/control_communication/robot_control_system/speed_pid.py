"""
Forward speed control using simple-pid (linear velocity in m/s).

setpoint: desired speed
measurement: current speed from encoders

Output is a normalized motor command in [-1, 1] with a small deadband so tiny commands
don’t chatter the H-bridge.

Docs: https://simple-pid.readthedocs.io/en/latest/user_guide.html
"""

from __future__ import annotations

import math

from simple_pid import PID

from .heading_pid import MOTOR_DEADBAND

SPEED_KP = 1.0
SPEED_KI = 0.0
SPEED_KD = 0.0

SPEED_DEADBAND = 0.02  # m/s — treat as “close enough”, hold zero output


class SpeedPID:
    """PI(D) on forward speed; output is linear_x before mixing to left/right wheels."""

    def __init__(self) -> None:
        self._pid = PID(
            SPEED_KP,
            SPEED_KI,
            SPEED_KD,
            setpoint=0.0,
            output_limits=(-1.0, 1.0),
            sample_time=None,
        )
        self._pid.differential_on_measurement = False

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self._pid.tunings = (kp, ki, kd)
        self._pid.reset()

    def reset(self) -> None:
        self._pid.reset()

    def update(self, setpoint: float, current_speed: float) -> float:
        err = setpoint - current_speed
        if abs(err) < SPEED_DEADBAND:
            self._pid.reset()
            return 0.0

        self._pid.setpoint = setpoint
        raw = self._pid(current_speed)
        if abs(raw) < 1e-6:
            return 0.0
        return math.copysign(max(abs(raw), MOTOR_DEADBAND), raw)
