"""
state_machine/states/line_follow_angle.py

Lateral-error-to-angle line follower.

Geometry
--------
The camera is mounted `camera_forward_m` ahead of the wheel-base centre.
If the line has a lateral offset of `lat_m` at the camera position, the
angle the robot must turn to aim at that point is:

    delta_angle = atan2(lat_m, camera_forward_m)

This is a pure geometric relationship — no pixel-space direction vectors,
no arbitrary gains.  The computed angle is passed to add_direction() and is
also stored on the instance so the web UI can display it.
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
    from hardware.robot       import Robot
    from vision.line_detector import LineDetector

log = logging.getLogger(__name__)

_lfa = _config.get()['line_follow_angle']
FOLLOW_SPEED: float = _lfa['follow_speed']


class LineFollowAngle(State):
    name = 'line_follow_angle'

    def __init__(self):
        # Last computed delta angle (radians) — read by web server for telemetry
        self.delta_angle_rad: float = 0.0

    def enter(self, robot: 'Robot', detector: Optional['LineDetector']) -> None:
        self.delta_angle_rad = 0.0
        robot.set_speed(0.0)
        log.info('Entered LINE_FOLLOW_ANGLE  speed=%.2f', FOLLOW_SPEED)

    def tick(self, robot: 'Robot', detector: Optional['LineDetector']) -> Optional[str]:
        if detector is None:
            log.warning('LINE_FOLLOW_ANGLE: no detector — stopping')
            return 'stopped'

        result = detector.get_result()
        if result is None:
            robot.set_speed(0.0)
            self.delta_angle_rad = 0.0
            log.debug('LINE_FOLLOW_ANGLE: line lost')
            return 'stopped'

        vc = _config.get()['vision']
        ppm       = vc.get('pixels_per_meter', 0)
        cam_fwd_m = vc.get('camera_forward_m', 0.0)

        lat_px = result.lateral_distance_px

        if ppm and cam_fwd_m:
            lat_m = lat_px / ppm
            delta = math.atan2(lat_m, cam_fwd_m)
        else:
            # Fallback: no calibration — treat lat_px as a unit-less offset
            delta = math.atan2(lat_px, 1.0)

        self.delta_angle_rad = delta

        # Convert the angle back to a direction vector for add_direction:
        #   add_direction(x, y) computes atan2(-x, abs(y)) internally,
        #   so pass x = +sin(delta), y = cos(delta) → effective heading change = -delta,
        #   which steers toward the line (same sign convention as line_follow.py).
        bx =  math.sin(delta)
        by =  math.cos(delta)
        robot.add_direction(bx, by)
        robot.set_speed(FOLLOW_SPEED)

        log.debug(
            'LINE_FOLLOW_ANGLE: lat=%.1fpx  lat=%.4fm  delta=%.2f°',
            lat_px, lat_px / ppm if ppm else float('nan'), math.degrees(delta),
        )
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector']) -> None:
        self.delta_angle_rad = 0.0
        robot.set_speed(0.0)
        log.info('Leaving LINE_FOLLOW_ANGLE')
