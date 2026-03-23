"""
RotationController — heading PID.

Rotates the robot in place by a fixed number of degrees using encoder
feedback and differential-drive kinematics.  Shares the step() / done /
progress() interface with SpeedController so either can drive the same
control loop.

    ctrl = RotationController(brain, degrees=90)
    while not ctrl.done:
        ctrl.step()
    brain.idle()
"""
from __future__ import annotations

import math
import time

import config as _config_module
from control.brain import RobotBrain, TICKS_PER_REV


# ── Discrete PID ──────────────────────────────────────────────────────────────

class _DiscretePID:
    """Discrete PID with real dt, derivative-on-error, and integral clamping."""

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float = -1.0, out_max: float = 1.0,
                 integral_limit: float = 1.0) -> None:
        self.kp = kp; self.ki = ki; self.kd = kd
        self.out_min = out_min; self.out_max = out_max
        self.integral_limit = integral_limit
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time: float | None = None

    def reset(self) -> None:
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def update(self, setpoint: float, measurement: float) -> float:
        now   = time.monotonic()
        error = setpoint - measurement
        dt    = (now - self._prev_time) if self._prev_time is not None else 0.0
        if dt > 0:
            self._integral = max(-self.integral_limit,
                                 min(self.integral_limit, self._integral + error * dt))
        deriv = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        self._prev_time  = now
        raw = self.kp * error + self.ki * self._integral + self.kd * deriv
        return max(self.out_min, min(self.out_max, raw))


# ── Rotation controller ───────────────────────────────────────────────────────

class RotationController:
    """
    Heading PID.  Integrates encoder deltas into a heading estimate and drives
    the robot to a target angle.

    Positive degrees = clockwise (left wheel forward, right backward).
    Negative degrees = counter-clockwise.
    """

    @staticmethod
    def _wrap(angle: float) -> float:
        """Wrap angle to (-π, π]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def _apply_deadband(value: float, deadband: float) -> float:
        if abs(value) < 1e-6:
            return 0.0
        return math.copysign(max(abs(value), deadband), value)

    def __init__(
        self,
        brain:     RobotBrain,
        degrees:   float,
        tolerance: float | None = None,
        cfg:       _config_module.Config | None = None,
    ) -> None:
        c = cfg if cfg is not None else _config_module.get()
        tol_deg = tolerance if tolerance is not None else c.rot_tolerance
        l, r = brain.encoder_ticks
        self._brain          = brain
        self._prev_left      = l
        self._prev_right     = r
        self._wheel_circ     = math.pi * c.wheel_diameter_m
        self._wheelbase      = c.wheelbase_m
        self._heading        = 0.0                   # integrated, radians; positive = CW
        self._target         = math.radians(degrees) # positive = CW
        self._tol_rad        = math.radians(abs(tol_deg))
        self._motor_deadband = c.rot_motor_deadband
        self.done            = False
        self._pid = _DiscretePID(c.rot_kp, c.rot_ki, c.rot_kd)

    # ── Control interface ─────────────────────────────────────────────────────

    def step(self) -> None:
        """Integrate heading, run PID, send voltage to brain."""
        if self.done:
            return

        l, r = self._brain.encoder_ticks
        dl = (l - self._prev_left)  * self._wheel_circ / TICKS_PER_REV
        dr = (r - self._prev_right) * self._wheel_circ / TICKS_PER_REV
        self._heading   += (dl - dr) / self._wheelbase
        self._prev_left  = l
        self._prev_right = r

        error = self._wrap(self._target - self._heading)

        if abs(error) <= self._tol_rad:
            self._pid.reset()
            self.done = True
            return

        output = self._pid.update(error, 0.0)
        output = self._apply_deadband(output, self._motor_deadband)

        # Positive = CW (left fwd, right rev); negative = CCW.
        self._brain.send_voltage(output, -output)

    def progress(self) -> tuple[float, float]:
        """Returns (abs_degrees_traveled, abs_degrees_target)."""
        return abs(math.degrees(self._heading)), abs(math.degrees(self._target))

    def error_deg(self) -> float:
        """Signed error in degrees: positive = not yet reached, negative = overshot."""
        return math.degrees(self._target - self._heading)
