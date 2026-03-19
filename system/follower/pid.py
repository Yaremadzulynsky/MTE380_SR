"""Thread-safe PID controller."""

import threading
import time


class PID:
    def __init__(self, kp: float, ki: float, kd: float, i_max: float, out_max: float):
        self.kp      = kp
        self.ki      = ki
        self.kd      = kd
        self.i_max   = i_max
        self.out_max = out_max

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time: float | None = None
        self._lock = threading.Lock()

    def update(self, error: float) -> float:
        with self._lock:
            now = time.monotonic()
            if self._prev_time is None:
                dt = 1.0 / 30.0   # assume 30 Hz on first call
            else:
                dt = now - self._prev_time
                dt = max(0.001, min(0.5, dt))
            self._prev_time = now

            self._integral += error * dt
            self._integral = max(-self.i_max, min(self.i_max, self._integral))

            derivative = (error - self._prev_error) / dt
            self._prev_error = error

            raw = self.kp * error + self.ki * self._integral + self.kd * derivative
            return max(-self.out_max, min(self.out_max, raw))

    def reset(self):
        with self._lock:
            self._integral   = 0.0
            self._prev_error = 0.0
            self._prev_time  = None

    def set_gains(self, kp=None, ki=None, kd=None, i_max=None, out_max=None):
        with self._lock:
            if kp      is not None: self.kp      = float(kp)
            if ki      is not None: self.ki      = float(ki)
            if kd      is not None: self.kd      = float(kd)
            if i_max   is not None: self.i_max   = float(i_max)
            if out_max is not None: self.out_max = float(out_max)
            # Reset integral so stale windup doesn't kick in with new gains
            self._integral = 0.0
