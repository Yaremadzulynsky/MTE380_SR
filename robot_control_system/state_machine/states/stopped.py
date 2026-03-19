"""
state_machine/states/stopped.py

Robot is stopped. Motors off, waiting for a command to transition out.
"""

from __future__ import annotations
import logging
from typing import TYPE_CHECKING, Optional

from state_machine.state import State

if TYPE_CHECKING:
    from hardware.robot       import Robot
    from vision.line_detector import LineDetector

log = logging.getLogger(__name__)


class Stopped(State):
    name = 'stopped'

    def enter(self, robot: 'Robot', detector: Optional['LineDetector']) -> None:
        robot.set_speed(0.0)
        robot.set_motors(0.0, 0.0)
        log.info('Entered STOPPED')

    def tick(self, robot: 'Robot', detector: Optional['LineDetector']) -> Optional[str]:
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector']) -> None:
        log.info('Leaving STOPPED')
