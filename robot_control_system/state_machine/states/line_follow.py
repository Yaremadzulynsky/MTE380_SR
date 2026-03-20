"""
state_machine/states/line_follow.py

Follow the detected line using the direction vector from LineDetector.

Control logic
-------------
Each tick:
  1. Read the latest LineResult from the detector.
  2. If no line is found, stop and transition to STOPPED.
  3. Compute a geometric look-ahead direction:
       - The rotation origin (wheel-base centre) is behind the camera by
         cam_fwd_m.  The direction from the origin to the line's observed
         position at the camera is (lat_px, d_px) where d_px = cam_fwd_m * ppm.
       - add_direction computes delta = atan2(lat_px, d_px), which naturally
         reduces angular sensitivity when d_px is large, avoiding over-steering.
       - A deadzone suppresses small lateral errors to prevent oscillation.
  4. Pass the direction to robot.add_direction() and drive forward.
"""

from __future__ import annotations
import logging
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

# ── Tuning (from config.yaml) ─────────────────────────────────────────────────
_lf = _config.get()['line_follow']
FOLLOW_SPEED: float = _lf['follow_speed']
DEADZONE_PX:  float = _lf['deadzone_px']


def compute_target_direction(result) -> tuple[float, float]:
    """Compute the geometric look-ahead direction from a LineResult.

    Returns the direction vector from the rotation origin (wheel-base centre)
    to the observed line position at the camera: (lat_px, d_px) where
    d_px = cam_fwd_m * ppm.  add_direction computes atan2(lat_px, d_px),
    so the large d_px naturally reduces angular sensitivity (the farther the
    camera is ahead of the origin, the smaller the steering angle for the same
    lateral error).

    Falls back to (lat_px, 1.0) if vision is not calibrated.
    """
    lat_px = result.lateral_distance_px

    # Deadzone: suppress tiny errors to prevent oscillation
    if abs(lat_px) < DEADZONE_PX:
        lat_px = 0.0

    _vc = _config.get()['vision']
    ppm       = _vc.get('pixels_per_meter')
    cam_fwd_m = _vc.get('camera_forward_m', 0.0)

    if ppm and cam_fwd_m:
        d_px = cam_fwd_m * ppm
        return lat_px, d_px

    # Fallback: not calibrated — proportional correction with unit forward
    return lat_px, 1.0


class LineFollow(State):
    name = 'line_follow'

    def enter(self, robot: 'Robot', detector: Optional['LineDetector'],
              odometry: Optional['Odometry']) -> None:
        robot.set_speed(0.0)
        log.info('Entered LINE_FOLLOW  speed=%.2f  deadzone=%dpx', FOLLOW_SPEED, int(DEADZONE_PX))

    def tick(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry']) -> Optional[str]:
        if detector is None:
            log.warning('LINE_FOLLOW: no detector — stopping')
            return 'stopped'

        result = detector.get_result()

        if result is None:
            # Line lost — stop and wait
            robot.set_speed(0.0)
            log.debug('LINE_FOLLOW: line lost')
            return 'stopped'

        bx, by = compute_target_direction(result)

        robot.add_direction(bx, by)
        robot.set_speed(FOLLOW_SPEED)

        log.debug(
            'LINE_FOLLOW: lat=%.1fpx dir=(%.1f,%.1f)',
            result.lateral_distance_px, bx, by,
        )
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry']) -> None:
        robot.set_speed(0.0)
        log.info('Leaving LINE_FOLLOW')
