"""
PositionController — distance PID.

Drives the robot straight forward or backward by a fixed distance in metres
using encoder feedback.  Both wheels are commanded equally.

    ctrl = PositionController(brain, meters=0.5)
    while not ctrl.done:
        ctrl.step()
    brain.idle()

Positive metres = forward, negative = backward.
"""
from __future__ import annotations

import math
import time

import config as _config_module
from control.brain import RobotBrain, TICKS_PER_REV


class PositionController:
    """
    Distance PID.  Integrates average encoder displacement into metres and
    drives both wheels equally to reach the target distance.
    """

    @staticmethod
    def _apply_deadband(value: float, deadband: float) -> float:
        if abs(value) < 1e-6:
            return 0.0
        return math.copysign(max(abs(value), deadband), value)

    def __init__(
        self,
        brain:     RobotBrain,
        meters:    float,
        tolerance: float | None = None,
        cfg:       _config_module.Config | None = None,
    ) -> None:
        c = cfg if cfg is not None else _config_module.get()
        tol = tolerance if tolerance is not None else c.pos_tolerance_m
        l, r = brain.encoder_ticks
        self._brain          = brain
        self._wheel_circ     = math.pi * c.wheel_diameter_m
        self._target         = float(meters)
        self._tol            = abs(tol)
        self._motor_deadband = c.pos_motor_deadband
        self._displacement   = 0.0   # metres travelled so far (signed)
        self._prev_left      = l
        self._prev_right     = r
        self.done            = False

        from control.rotation_controller import _DiscretePID
        self._pid = _DiscretePID(c.pos_kp, c.pos_ki, c.pos_kd)

    def step(self) -> None:
        """Integrate displacement, run PID, send equal voltage to both wheels."""
        if self.done:
            return

        l, r = self._brain.encoder_ticks
        dl = (l - self._prev_left)  * self._wheel_circ / TICKS_PER_REV
        dr = (r - self._prev_right) * self._wheel_circ / TICKS_PER_REV
        self._displacement += (dl + dr) / 2.0
        self._prev_left  = l
        self._prev_right = r

        error = self._target - self._displacement

        if abs(error) <= self._tol:
            self._pid.reset()
            self.done = True
            return

        output = self._pid.update(error, 0.0)
        output = self._apply_deadband(output, self._motor_deadband)
        self._brain.send_voltage(output, output)

    def progress(self) -> tuple[float, float]:
        """Returns (abs_metres_traveled, abs_metres_target)."""
        return abs(self._displacement), abs(self._target)

    def error_m(self) -> float:
        """Signed error in metres: positive = not yet reached, negative = overshot."""
        return self._target - self._displacement
