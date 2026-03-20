"""
state_machine/states/line_follow_p.py

Simple proportional lateral-error line follower.

Assumes the robot starts centred on the line.  Each tick:
  1. Read lateral_distance_px from the detector.
  2. Compute a heading correction:  correction_x = lateral_px * KP
  3. Drive straight forward with that correction blended in.

The direction vector sent to the robot is:
    bx = clamp(lateral_px * KP, -1, 1)
    by = 1.0
which is then normalised.  When the robot is on the line (lateral ≈ 0) it
drives straight; when it drifts it steers back proportionally.
"""

from __future__ import annotations
import logging
import math
from typing import TYPE_CHECKING, Optional

from state_machine.state import State
import sys as _sys, pathlib as _pathlib
_sys.path.insert(0, str(_pathlib.Path(__file__).parent.parent.parent))
import config as _config

if TYPE_CHECKING:
    from hardware.robot          import Robot
    from vision.line_detector    import LineDetector
    from state_machine.odometry  import Odometry

log = logging.getLogger(__name__)

_lf = _config.get()['line_follow_p']
FOLLOW_SPEED: float = _lf['follow_speed']
KP:           float = _lf['kp']


class LineFollowP(State):
    name = 'line_follow_p'

    def enter(self, robot: 'Robot', detector: Optional['LineDetector'],
              odometry: Optional['Odometry']) -> None:
        robot.set_speed(0.0)
        log.info('Entered LINE_FOLLOW_P  speed=%.2f  kp=%.4f', FOLLOW_SPEED, KP)

    def tick(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry']) -> Optional[str]:
        if detector is None:
            log.warning('LINE_FOLLOW_P: no detector — stopping')
            return 'stopped'

        result = detector.get_result()
        if result is None:
            robot.set_speed(0.0)
            log.debug('LINE_FOLLOW_P: line lost')
            return 'stopped'

        lat_px = result.lateral_distance_px

        # Proportional correction: steer toward the line proportional to drift
        bx = max(-1.0, min(1.0, lat_px * KP))
        by = 1.0
        length = math.hypot(bx, by)
        bx /= length
        by /= length

        robot.add_direction(bx, by)
        robot.set_speed(FOLLOW_SPEED)

        log.debug('LINE_FOLLOW_P: lat=%.1fpx  cmd=(%.3f,%.3f)', lat_px, bx, by)
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry']) -> None:
        robot.set_speed(0.0)
        log.info('Leaving LINE_FOLLOW_P')
