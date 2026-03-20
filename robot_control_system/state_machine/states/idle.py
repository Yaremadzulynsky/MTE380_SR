"""
state_machine/states/idle.py

Default startup state — motors off, waiting for a command.
"""

from __future__ import annotations
import logging
from typing import TYPE_CHECKING, Optional

from state_machine.state import State

if TYPE_CHECKING:
    from state_machine.hardware.robot       import Robot
    from state_machine.vision.line_detector import LineDetector
    from state_machine.odometry            import Odometry

log = logging.getLogger(__name__)


class Idle(State):
    name = 'idle'

    def enter(self, robot: 'Robot', detector: Optional['LineDetector'],
              odometry: Optional['Odometry'],
              target_heading: Optional[float] = None) -> None:
        if robot is not None:
            robot.set_speed(0.0)
            robot.set_motors(0.0, 0.0)
        log.info('Entered IDLE')

    def tick(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry'],
             target_heading: Optional[float] = None) -> Optional[str]:
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry'],
             target_heading: Optional[float] = None) -> None:
        log.info('Leaving IDLE')
