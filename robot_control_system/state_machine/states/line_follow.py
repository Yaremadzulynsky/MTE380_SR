"""
state_machine/states/line_follow.py

Follow a red line using the camera.

Each tick:
  1. Read the latest LineResult from the detector.
  2. Compute target direction = line tangent + lateral correction toward the line
     (same formula used by the camera overlay and odometry canvas).
  3. Feed the direction to robot.add_direction() so the heading PID steers toward it.
  4. Drive forward at follow_speed.

If the line is lost the robot stops (motors to zero) but stays in this state so
it can resume automatically when the line reappears.

Config (config.yaml → line_follow):
    follow_speed   : forward speed [0, 1] while following
    lateral_deadzone_m : metres of lateral error ignored before correction kicks in
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
        _lf = _config.get()['line_follow']
        _vc = _config.get()['vision']
        self._follow_speed     = float(_lf.get('follow_speed',        0.35))
        self._deadzone_m       = float(_lf.get('lateral_deadzone_m',  0.01))
        self._cam_fwd_m        = float(_vc.get('camera_forward_m',    0.15))
        self._line_lost        = False

    def enter(self, robot: 'Robot', detector: Optional['LineDetector'],
              odometry: Optional['Odometry']) -> None:
        self._line_lost = False
        if robot is not None:
            robot.set_speed(0.0)
        log.info('Entered LINE_FOLLOW  speed=%.2f  deadzone=%.3fm',
                 self._follow_speed, self._deadzone_m)

    def tick(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry']) -> Optional[str]:
        if robot is None or detector is None:
            return None

        result = detector.get_result()

        if result is None:
            if not self._line_lost:
                log.warning('Line lost — stopping')
                self._line_lost = True
            robot.set_speed(0.0)
            robot.set_motors(0.0, 0.0)
            return None

        if self._line_lost:
            log.info('Line reacquired')
            self._line_lost = False

        # Compute right-turn angle in robot frame (matches camera and canvas views):
        #   theta = line_angle + atan2(lat_m, cam_fwd_m)
        angle_rad = math.radians(result.angle_deg)
        lat_m     = result.lateral_distance_m

        if lat_m is not None and abs(lat_m) >= self._deadzone_m:
            lat_corr = math.atan2(lat_m, self._cam_fwd_m)
        else:
            lat_corr = 0.0

        theta = angle_rad + lat_corr

        robot.add_direction(math.sin(theta), math.cos(theta))
        robot.set_speed(self._follow_speed)
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry']) -> None:
        if robot is not None:
            robot.set_speed(0.0)
            robot.set_motors(0.0, 0.0)
        log.info('Leaving LINE_FOLLOW')
