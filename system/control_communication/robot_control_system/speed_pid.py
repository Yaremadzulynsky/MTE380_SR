"""Speed PID — wraps the generic PID for forward/backward velocity control."""

from .pid import PID

SPEED_KP = 1
SPEED_KI = 0.0
SPEED_KD = 0.0

MOTOR_DEADBAND = 0.015   # minimum output to actually move the motors
SPEED_DEADBAND = 0.02    # speed error (m/s) below which correction stops


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
