"""
state_machine/states/line_follow.py

Follow the detected line using the direction vector from LineDetector.

Control logic
-------------
Each tick:
  1. Read the latest LineResult from the detector.
  2. If no line is found, stop and transition to STOPPED.
  3. Compute a blended direction:
       - Start with the line tangent direction (always points along the line).
       - Add a lateral correction component proportional to how far the robot
         has drifted from the line, using a deadzone to prevent oscillation
         when the robot is already on the line.
  4. Pass the blended direction to robot.add_direction() and drive forward.
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

# ── Tuning (from config.yaml) ─────────────────────────────────────────────────
_lf = _config.get()['line_follow']
FOLLOW_SPEED:        float = _lf['follow_speed']
DEADZONE_PX:         float = _lf['deadzone_px']
CORRECTION_SCALE_PX: float = _lf['correction_scale_px']


def compute_target_direction(result) -> tuple[float, float]:
    """Compute the blended direction vector from a LineResult (tangent + lateral correction).

    Accounts for the camera being mounted ahead of the wheel base: the lateral
    distance measured at the camera position is projected back to the wheel-base
    pivot using the tangent vector's lateral component.
    """
    tx, ty   = result.direction
    lat_px   = result.lateral_distance_px

    # Project lateral distance from camera plane back to wheel-base pivot
    _vc = _config.get()['vision']
    ppm = _vc.get('pixels_per_meter')          # None until calibrated
    cam_fwd_m = _vc.get('camera_forward_m', 0.0)
    if ppm and cam_fwd_m:
        # How far the line has shifted laterally over cam_fwd_m of forward distance
        lat_px = lat_px + cam_fwd_m * ppm * tx

    weight   = min(1.0, max(0.0, abs(lat_px) - DEADZONE_PX) / CORRECTION_SCALE_PX)
    corr_x   = math.copysign(weight, lat_px)
    bx, by   = tx + corr_x, ty
    length   = math.hypot(bx, by)
    if length > 1e-6:
        bx /= length
        by /= length
    return bx, by


class LineFollow(State):
    name = 'line_follow'

    def enter(self, robot: 'Robot', detector: Optional['LineDetector']) -> None:
        robot.set_speed(0.0)
        log.info('Entered LINE_FOLLOW  speed=%.2f  deadzone=%dpx', FOLLOW_SPEED, int(DEADZONE_PX))

    def tick(self, robot: 'Robot', detector: Optional['LineDetector']) -> Optional[str]:
        if detector is None:
            log.warning('LINE_FOLLOW: no detector — stopping')
            return 'stopped'

        result = detector.get_result()

        if result is None:
            # Line lost — stop and wait
            robot.set_speed(0.0)
            log.debug('LINE_FOLLOW: line lost')
            return 'stopped'

        tx, ty = result.direction
        bx, by = compute_target_direction(result)

        robot.add_direction(bx, by)
        robot.set_speed(FOLLOW_SPEED)

        log.debug(
            'LINE_FOLLOW: lat=%.1fpx tangent=(%.2f,%.2f) cmd=(%.2f,%.2f)',
            result.lateral_distance_px, tx, ty, bx, by,
        )
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector']) -> None:
        robot.set_speed(0.0)
        log.info('Leaving LINE_FOLLOW')
