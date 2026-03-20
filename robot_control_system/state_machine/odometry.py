"""
state_machine/odometry.py

Dead-reckoning pose estimator driven by encoder ticks.
Owned and updated by StateMachine; passed to states each tick.
"""

from __future__ import annotations
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
import config as _config


class Odometry:
    """
    Integrates differential-drive encoder ticks into (x, y, heading).

    Call update(left_ticks, right_ticks) each tick.
    Origin and heading are reset to zero at construction or via reset().
    """

    def __init__(self):
        cfg = _config.get()
        w = cfg['wheel']
        self._ticks_per_rev  = w['ticks_per_rev']
        self._wheel_circ_m   = math.pi * w['diameter_m']
        self._wheel_base_m   = w['base_m']

        self._x       = 0.0
        self._y       = 0.0
        self._heading = math.pi / 2.0  # radians; π/2 = facing world +y (up on screen)
        self._prev_left:  int | None = None
        self._prev_right: int | None = None

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self, left: int, right: int) -> None:
        """Integrate new absolute encoder tick counts into the pose."""
        if self._prev_left is None:
            self._prev_left  = left
            self._prev_right = right
            return

        dl = (left  - self._prev_left)  * self._wheel_circ_m / self._ticks_per_rev
        dr = (right - self._prev_right) * self._wheel_circ_m / self._ticks_per_rev

        ds        = (dl + dr) / 2.0
        dtheta    = (dr - dl) / self._wheel_base_m
        theta_mid = self._heading + dtheta / 2.0

        self._x       += ds * math.cos(theta_mid)
        self._y       += ds * math.sin(theta_mid)
        self._heading += dtheta

        self._prev_left  = left
        self._prev_right = right

    def reset(self) -> None:
        """Reset pose to origin."""
        self._x           = 0.0
        self._y           = 0.0
        self._heading     = math.pi / 2.0
        self._prev_left   = None
        self._prev_right  = None

    @property
    def x(self) -> float:
        """X position in metres from origin."""
        return self._x

    @property
    def y(self) -> float:
        """Y position in metres from origin."""
        return self._y

    @property
    def heading(self) -> float:
        """Current heading in radians."""
        return self._heading

    def pose(self) -> tuple[float, float, float]:
        """Return (x_m, y_m, heading_rad)."""
        return self._x, self._y, self._heading
