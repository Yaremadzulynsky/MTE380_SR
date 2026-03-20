"""
state_machine/states/find_line.py

Recovery state entered when line_follow loses the line.

Keeps driving forward but steers in the opposite direction the robot was
last turning — if it was curving right and lost the line it overshot right,
so sweep left to find it again.

The recovery turn direction is captured from robot.angular_z at entry time.
Returns to line_follow as soon as the line is redetected.
Falls back to idle after timeout_s if the line is never found.

Config (config.yaml → find_line):
    follow_speed  : forward speed while searching [0, 1]
    turn_x        : lateral component of add_direction while sweeping [0, 1]
    timeout_s     : give up and go idle after this many seconds
"""

from __future__ import annotations
import logging
import math
import time
from typing import TYPE_CHECKING, Optional

import sys as _sys, pathlib as _pathlib
_sys.path.insert(0, str(_pathlib.Path(__file__).parent.parent.parent))
import config as _config

from state_machine.state import State

if TYPE_CHECKING:
    from state_machine.hardware.robot       import Robot
    from state_machine.vision.line_detector import LineDetector
    from state_machine.odometry            import Odometry

log = logging.getLogger(__name__)


class FindLine(State):
    name = 'find_line'

    def __init__(self):
        _fl = _config.get().get('find_line', {})
        self._follow_speed  = float(_fl.get('follow_speed',  0.30))
        self._turn_x        = float(_fl.get('turn_x',        0.5))
        self._timeout_s     = float(_fl.get('timeout_s',     3.0))
        self._recovery_x:   float = 0.0
        self._start_time:   float = 0.0

    def enter(self, robot: 'Robot', detector: Optional['LineDetector'],
              odometry: Optional['Odometry'],
              target_heading: Optional[float] = None) -> None:
        self._start_time = time.monotonic()

        # Determine which way robot was last turning from angular_z.
        # angular_z > 0 → was turning left  → sweep right (recovery_x > 0)
        # angular_z < 0 → was turning right → sweep left  (recovery_x < 0)
        # angular_z == 0 → default sweep right
        az = 0.0
        if robot is not None:
            az = robot.get_debug_info().get('angular_z', 0.0)
        self._recovery_x = math.copysign(self._turn_x, az) if az != 0.0 else self._turn_x

        log.info('Entered FIND_LINE  recovery_x=%.2f  timeout=%.1fs',
                 self._recovery_x, self._timeout_s)

    def tick(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry'],
             target_heading: Optional[float] = None) -> Optional[str]:
        if robot is None:
            return None

        if target_heading is not None:
            log.info('Line reacquired — returning to line_follow')
            return 'line_follow'

        if time.monotonic() - self._start_time > self._timeout_s:
            log.warning('Line not found after %.1fs — going idle', self._timeout_s)
            return 'idle'

        robot.add_direction(self._recovery_x, 1.0)
        robot.set_speed(self._follow_speed)
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry'],
             target_heading: Optional[float] = None) -> None:
        if robot is not None:
            robot.set_speed(0.0)
            robot.set_motors(0.0, 0.0)
        log.info('Leaving FIND_LINE')
