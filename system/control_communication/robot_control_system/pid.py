"""Discrete PID controller with anti-windup integral clamping."""

import time

import simple_pid

class PID:

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float = -1.0, out_max: float = 1.0,
                 integral_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral_limit = integral_limit

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time: float | None = None

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def update(self, setpoint: float, measurement: float) -> float:
        now   = time.monotonic()
        error = setpoint - measurement
        dt    = (now - self._prev_time) if self._prev_time is not None else 0.0

        if dt > 0:
            self._integral += error * dt
            self._integral  = max(-self.integral_limit,
                                  min(self.integral_limit, self._integral))

        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0

        self._prev_error = error
        self._prev_time  = now

        raw = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, raw))
