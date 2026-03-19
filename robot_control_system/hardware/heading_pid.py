
"""Heading PID — wraps the generic PID with angle wrapping and deadbands."""

import math

from .pid import PID

HEADING_KP, HEADING_KI, HEADING_KD = 0.25, 0, 0 #0.5, 0.3 #0.5, 0.1
MOTOR_DEADBAND   = 0.015  # minimum output to actually move the motors
HEADING_DEADBAND = 0.005  # heading error (rad) below which correction stops (~0.3°)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi

def _apply_deadband(value: float) -> float:
    if abs(value) < 1e-6:
        return 0.0
    return math.copysign(max(abs(value), MOTOR_DEADBAND), value)


class HeadingPID:

    def __init__(self):
        self._pid = PID(HEADING_KP, HEADING_KI, HEADING_KD)

    def set_gains(self, kp: float, ki: float, kd: float):
        self._pid.kp = kp
        self._pid.ki = ki
        self._pid.kd = kd
        self._pid.reset()

    def reset(self):
        self._pid.reset()

    def update(self, desired_heading: float, current_heading: float, scale: float = 1.0) -> float:
        """Return angular_z given desired and current heading in radians."""
        error = _wrap(desired_heading - current_heading)
        if abs(error) < HEADING_DEADBAND:
            self._pid.reset()
            return 0.0
        return _apply_deadband(self._pid.update(error, 0.0) * scale)
