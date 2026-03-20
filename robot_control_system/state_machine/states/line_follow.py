"""
state_machine/states/line_follow.py

Follow a red line using the camera.

Each tick the state machine passes in `target_heading` — the world-frame heading
the robot should drive toward, already computed from line angle + lateral
correction.  This state converts that to a robot-frame direction and feeds it to
robot.add_direction().

If the line is lost the robot stops but stays in this state so it resumes when
the line reappears.

Config (config.yaml → line_follow):
    follow_speed : forward speed [0, 1] while line is visible
"""

from __future__ import annotations
import logging
import math
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


class LineFollow(State):
    name = 'line_follow'

    def __init__(self):
        self._follow_speed = float(_config.get()['line_follow'].get('follow_speed', 0.35))
        self._line_lost    = False

    def enter(self, robot: 'Robot', detector: Optional['LineDetector'],
              odometry: Optional['Odometry'],
              target_heading: Optional[float] = None) -> None:
        self._line_lost = False
        if robot is not None:
            robot.set_speed(0.0)
        log.info('Entered LINE_FOLLOW  speed=%.2f', self._follow_speed)

    def tick(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry'],
             target_heading: Optional[float] = None) -> Optional[str]:
        if robot is None:
            return None

        if target_heading is None:
            # No line detected
            if not self._line_lost:
                log.warning('Line lost — stopping')
                self._line_lost = True
            robot.set_speed(0.0)
            robot.set_motors(0.0, 0.0)
            return None

        if self._line_lost:
            log.info('Line reacquired')
            self._line_lost = False

        # Convert world-frame target heading to a robot-frame direction vector.
        # theta = how far right of current heading the target is.
        # robot.add_direction(sin θ, cos θ) steers the heading PID toward it.
        theta = odometry.heading - target_heading
        robot.add_direction(math.sin(theta), math.cos(theta))
        robot.set_speed(self._follow_speed)
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry'],
             target_heading: Optional[float] = None) -> None:
        if robot is not None:
            robot.set_speed(0.0)
            robot.set_motors(0.0, 0.0)
        log.info('Leaving LINE_FOLLOW')
