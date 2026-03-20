"""
state_machine/main.py — run the robot brain.

Usage:
  python -m state_machine.main --port /dev/ttyACM0 --virtual-cam
  python -m state_machine.main --port COM3 --camera 0
  python -m state_machine.main --no-robot --virtual-cam   # vision only, no hardware
"""

import argparse
import logging
import signal
import sys
import time

sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent.parent))

from hardware.robot              import Robot, MAX_SPEED, MAX_ROT_SPEED
import hardware.robot as _robot_module
from vision.line_detector        import LineDetector
from state_machine.machine       import StateMachine
from state_machine.states        import Stopped, LineFollow, LineFollowP, LineFollowAngle
from web_server.server           import WebServer
from web_server.main             import _find_droidcam_index


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='MTE 380 robot brain')

    robot_group = parser.add_mutually_exclusive_group()
    robot_group.add_argument('--port', default='/dev/ttyACM0')
    robot_group.add_argument('--no-robot', action='store_true')
    parser.add_argument('--baud', type=int, default=115200)

    cam_group = parser.add_mutually_exclusive_group()
    cam_group.add_argument('--virtual-cam', action='store_true',
                           help='Auto-detect OBS/DroidCam virtual webcam')
    cam_group.add_argument('--camera',
                           help='Camera source: index, "pi", file, or URL')

    parser.add_argument('--max-speed',     type=float, default=None,
                        help='Override MAX_SPEED (0–1, default %.2f)' % MAX_SPEED)
    parser.add_argument('--max-rot-speed', type=float, default=None,
                        help='Override MAX_ROT_SPEED (0–1, default %.2f)' % MAX_ROT_SPEED)
    parser.add_argument('--web-port', type=int, default=8321)
    parser.add_argument('--initial-state', default='stopped',
                        choices=['stopped', 'line_follow', 'line_follow_p', 'line_follow_angle'])
    parser.add_argument('--debug', action='store_true')
    return parser


def main():
    args = build_parser().parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )
    log = logging.getLogger('state_machine.main')

    # ── Speed overrides ───────────────────────────────────────────────────────
    if args.max_speed is not None:
        _robot_module.MAX_SPEED = args.max_speed
        log.info('MAX_SPEED → %.2f', args.max_speed)
    if args.max_rot_speed is not None:
        _robot_module.MAX_ROT_SPEED = args.max_rot_speed
        log.info('MAX_ROT_SPEED → %.2f', args.max_rot_speed)

    # ── Robot ────────────────────────────────────────────────────────────────
    robot = None
    if not args.no_robot:
        robot = Robot(args.port, args.baud)
        robot.start()
        log.info('Robot connected on %s', args.port)

    # ── Detector ─────────────────────────────────────────────────────────────
    detector = None
    if args.virtual_cam:
        log.info('Auto-detecting virtual webcam…')
        source, backend = _find_droidcam_index()
        log.info('Virtual webcam found at index %d', source)
        detector = LineDetector(source=source, backend=backend, debug=True)
        detector.start()
    elif args.camera:
        source = args.camera
        if source == 'pi':
            source = None
        else:
            try:
                source = int(source)
            except ValueError:
                pass
        detector = LineDetector(source=source, debug=args.debug)
        detector.start()

    # ── State machine ─────────────────────────────────────────────────────────
    sm = (
        StateMachine(robot, detector)
        .register(Stopped())
        .register(LineFollow())
        .register(LineFollowP())
        .register(LineFollowAngle())
    )

    # ── Web server (debug dashboard) ──────────────────────────────────────────
    server = WebServer(robot=robot, detector=detector, state_machine=sm, port=args.web_port)
    server.start()
    log.info('Dashboard: http://localhost:%d', args.web_port)

    sm.start(args.initial_state)

    # ── Run until Ctrl-C ──────────────────────────────────────────────────────
    def _shutdown(sig, frame):
        log.info('Shutting down…')
        sm.stop()
        if detector:
            detector.stop()
        if robot:
            robot.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    while True:
        time.sleep(1)


if __name__ == '__main__':
    main()
