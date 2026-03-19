"""Speed PID — wraps the generic PID for forward/backward velocity control."""

import sys as _sys, pathlib as _pathlib
_sys.path.insert(0, str(_pathlib.Path(__file__).parent.parent))
import config as _config

from .pid import PID

_c = _config.get()['speed_pid']
SPEED_KP       = _c['kp']
SPEED_KI       = _c['ki']
SPEED_KD       = _c['kd']
MOTOR_DEADBAND = _c['motor_deadband']
SPEED_DEADBAND = _c['speed_deadband']


class SpeedPID:

    def __init__(self):
        self._pid = PID(SPEED_KP, SPEED_KI, SPEED_KD)

    def set_gains(self, kp: float, ki: float, kd: float):
        self._pid.kp = kp
        self._pid.ki = ki
        self._pid.kd = kd
        self._pid.reset()

    def reset(self):
        self._pid.reset()

    def update(self, setpoint: float, current_speed: float) -> float:
        """Return linear_x given setpoint and current speed in m/s."""
        error = setpoint - current_speed
        if abs(error) < SPEED_DEADBAND:
            self._pid.reset()
            return 0.0
        raw = self._pid.update(setpoint, current_speed)
        if abs(raw) < 1e-6:
            return 0.0
        import math
        return math.copysign(max(abs(raw), MOTOR_DEADBAND), raw)
