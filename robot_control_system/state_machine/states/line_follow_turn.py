"""
state_machine/states/line_follow_turn.py

Line-following state used when the detector reports significant curvature.
Identical to line_follow but uses a lower turn_speed.

Returns to 'line_follow' when curvature drops below the threshold.
Falls to 'find_line' if the line is lost.

Config (config.yaml → line_follow_turn):
    turn_speed              : forward speed while turning [0, 1]
    curvature_threshold_deg : abs curvature below this exits back to line_follow
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


class LineFollowTurn(State):
    name = 'line_follow_turn'

    def __init__(self):
        _lft = _config.get().get('line_follow_turn', {})
        self._turn_speed             = float(_lft.get('turn_speed',              0.20))
        self._curvature_threshold_deg = float(_lft.get('curvature_threshold_deg', 15.0))

    def enter(self, robot: 'Robot', detector: Optional['LineDetector'],
              odometry: Optional['Odometry'],
              target_heading: Optional[float] = None) -> None:
        if robot is not None:
            robot.set_speed(0.0)
        log.info('Entered LINE_FOLLOW_TURN  speed=%.2f  threshold=%.1f°',
                 self._turn_speed, self._curvature_threshold_deg)

    def tick(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry'],
             target_heading: Optional[float] = None) -> Optional[str]:
        if robot is None:
            return None

        if target_heading is None:
            log.warning('Line lost — switching to find_line')
            return 'find_line'

        # Return to normal follow once curvature clears.
        if detector is not None:
            result = detector.get_result()
            if result is None or result.curvature is None or \
                    abs(math.degrees(result.curvature)) < self._curvature_threshold_deg:
                log.info('Curvature cleared — returning to line_follow')
                return 'line_follow'

        theta = odometry.heading - target_heading
        robot.add_direction(math.sin(theta), math.cos(theta))
        robot.set_speed(self._turn_speed)
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry'],
             target_heading: Optional[float] = None) -> None:
        if robot is not None:
            robot.set_speed(0.0)
            robot.set_motors(0.0, 0.0)
        log.info('Leaving LINE_FOLLOW_TURN')
