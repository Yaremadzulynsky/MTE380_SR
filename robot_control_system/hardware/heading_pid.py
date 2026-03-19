
"""Heading PID — wraps the generic PID with angle wrapping and deadbands."""

import math
import sys as _sys, pathlib as _pathlib
_sys.path.insert(0, str(_pathlib.Path(__file__).parent.parent))
import config as _config

from .pid import PID

_c = _config.get()['heading_pid']
HEADING_KP       = _c['kp']
HEADING_KI       = _c['ki']
HEADING_KD       = _c['kd']
MOTOR_DEADBAND   = _c['motor_deadband']
HEADING_DEADBAND = _c['heading_deadband']


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
