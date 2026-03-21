"""
Heading hold / track using simple-pid.

We want zero *wrapped* angle error:

    e = wrap(desired_heading − current_heading) ∈ (−π, π]

The library computes error as (setpoint − measurement). With setpoint fixed at 0 and
measurement = −e, the PID sees error = e. No error_map needed, so I/D terms stay
consistent on the wrapped error (see simple-pid notes on error_map + derivative).

Docs: https://simple-pid.readthedocs.io/en/latest/user_guide.html
"""

from __future__ import annotations

import math

from simple_pid import PID

# Default tunings (radians)
HEADING_KP, HEADING_KI, HEADING_KD = 2.0, 0.1, 0.0

# Smallest drive command that actually moves the motors (same scale as linear_x / angular mix)
MOTOR_DEADBAND = 0.015
# Below this |wrapped error|, treat as on-heading and reset the controller
HEADING_DEADBAND = 0.005  # ~0.3°


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _apply_motor_deadband(value: float) -> float:
    if abs(value) < 1e-6:
        return 0.0
    return math.copysign(max(abs(value), MOTOR_DEADBAND), value)


class HeadingPID:
    """PID on wrapped heading error; output is the differential (angular_z) term before wheel mix."""

    def __init__(self) -> None:
        # setpoint=0, call with measurement = −wrapped_error  →  error = wrapped_error
        self._pid = PID(
            HEADING_KP,
            HEADING_KI,
            HEADING_KD,
            setpoint=0.0,
            output_limits=(-1.0, 1.0),
            sample_time=None,  # compute every __call__ (control loop sets the rate)
        )
        self._pid.differential_on_measurement = False  # D on error, like the old discrete PID

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self._pid.tunings = (kp, ki, kd)
        self._pid.reset()

    def reset(self) -> None:
        self._pid.reset()

    def update(
        self,
        desired_heading: float,
        current_heading: float,
        scale: float = 1.0,
    ) -> float:
        """
        Return angular_z contribution in [-1, 1] (then typically multiplied by rotation_scale).

        ``scale`` is applied to the PID output (same as before): rotation intensity knob.
        """
        wrapped = _wrap_pi(desired_heading - current_heading)
        if abs(wrapped) < HEADING_DEADBAND:
            self._pid.reset()
            return 0.0

        raw = self._pid(-wrapped) * scale
        return _apply_motor_deadband(raw)
