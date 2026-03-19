"""
web_server/main.py — start the MTE 380 debug web server.

Usage:
  # OBS Virtual Camera / DroidCam / any virtual webcam (auto-detected):
  python -m web_server.main --no-robot --virtual-cam
  python -m web_server.main --port COM3 --virtual-cam

  # Raspberry Pi camera
  python -m web_server.main --port /dev/ttyACM0 --camera pi

  # Local webcam by index
  python -m web_server.main --port COM3 --camera 0

  # Robot hardware only (no camera)
  python -m web_server.main --port COM3
"""

import argparse
import logging
import signal
import sys
import time

sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent.parent))

from hardware.robot       import Robot
from vision.line_detector import LineDetector
from web_server.server    import WebServer

def _find_droidcam_index(max_index: int = 10) -> tuple:
    """Scan webcam indices across backends and return (index, backend) for the
    first source that opens successfully.

    Tries CAP_DSHOW and CAP_MSMF (Windows virtual webcam backends) before the
    default backend.  Raises RuntimeError if nothing is found.
    """
    import cv2
    backends = []
    if hasattr(cv2, 'CAP_DSHOW'):
        backends.append(('DSHOW', cv2.CAP_DSHOW))
    if hasattr(cv2, 'CAP_MSMF'):
        backends.append(('MSMF', cv2.CAP_MSMF))
    backends.append(('default', 0))

    for i in range(max_index, -1, -1):
        for backend_name, backend in backends:
            cap = cv2.VideoCapture(i, backend) if backend else cv2.VideoCapture(i)
            if cap.isOpened():
                cap.release()
                return (i, backend)
            cap.release()

    raise RuntimeError(
        'Could not find any working camera source. '
        'Make sure OBS Virtual Camera is started (Tools → Virtual Camera → Start) '
        'or your virtual webcam software is running.'
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='MTE 380 debug web server',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Serial / robot
    robot_group = parser.add_mutually_exclusive_group()
    robot_group.add_argument('--port', default='/dev/ttyACM0',
                             help='Serial port for Arduino (default: /dev/ttyACM0)')
    robot_group.add_argument('--no-robot', action='store_true',
                             help='Skip serial connection (camera/UI only)')
    parser.add_argument('--baud', type=int, default=115200)

    # Camera source — mutually exclusive
    cam_group = parser.add_mutually_exclusive_group()
    cam_group.add_argument('--virtual-cam', action='store_true',
                           help='Auto-detect a virtual webcam (OBS Virtual Camera, DroidCam, etc.)')
    cam_group.add_argument('--camera',
                           help='Camera source: integer webcam index, "pi" for Picamera2, '
                                'or a file/URL path')

    # Web server
    parser.add_argument('--host', default='0.0.0.0',
                        help='Bind address (default: 0.0.0.0)')
    parser.add_argument('--web-port', type=int, default=5000,
                        help='HTTP port (default: 5000)')
    parser.add_argument('--debug', action='store_true',
                        help='Verbose logging + annotated camera overlay')

    return parser


def main():
    parser = build_parser()
    args   = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )
    log = logging.getLogger('web_server.main')

    # ── Robot ────────────────────────────────────────────────────────────────
    robot = None
    if not args.no_robot:
        log.info('Connecting to robot on %s @ %d baud', args.port, args.baud)
        robot = Robot(args.port, args.baud)
        robot.start()

    # ── Line detector ────────────────────────────────────────────────────────
    detector = None
    if args.virtual_cam:
        log.info('Auto-detecting virtual webcam…')
        source, backend = _find_droidcam_index()
        log.info('Camera source: virtual webcam found at index %d', source)
        detector = LineDetector(source=source, backend=backend, debug=True)
        detector.start()
    elif args.camera is not None:
        source = args.camera
        if source == 'pi':
            source = None   # LineDetector(source=None) → Picamera2
            log.info('Camera source: Picamera2')
        else:
            try:
                source = int(source)
                log.info('Camera source: webcam index %d', source)
            except ValueError:
                log.info('Camera source: %s', source)
        detector = LineDetector(source=source, debug=args.debug)
        detector.start()

    # ── Web server ───────────────────────────────────────────────────────────
    server = WebServer(robot=robot, detector=detector,
                       host=args.host, port=args.web_port)
    server.start()

    log.info('Dashboard: http://localhost:%d', args.web_port)

    # ── Run until Ctrl-C ─────────────────────────────────────────────────────
    def _shutdown(sig, frame):
        log.info('Shutting down…')
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
