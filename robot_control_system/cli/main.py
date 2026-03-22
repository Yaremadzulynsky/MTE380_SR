"""
MTE 380 robot CLI.

Start the robot (state machine + web server):
  python -m cli.main                              # uses default port /dev/ttyACM0
  python -m cli.main --port /dev/ttyACM0 --camera pi
  python -m cli.main --port COM3 --camera 0
  python -m cli.main --no-robot --virtual-cam     # vision only, no hardware

Low-level hardware debug commands (talk directly to Arduino over serial):
  python -m cli.main --port /dev/ttyACM0 encoders
  python -m cli.main --port /dev/ttyACM0 heading
  python -m cli.main --port /dev/ttyACM0 motors <left> <right>
  python -m cli.main --port /dev/ttyACM0 drive <left> <right>
  python -m cli.main --port /dev/ttyACM0 wheel <left|right> <ticks>
  python -m cli.main --port /dev/ttyACM0 servo <angle>
"""

import argparse
import cmd
import logging
import math
import signal
import sys
import threading
import time

sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent.parent))
from state_machine.hardware.robot import Robot


# ── Camera helper ─────────────────────────────────────────────────────────────

def _find_virtual_cam(max_index: int = 10) -> tuple:
    import cv2
    backends = []
    if hasattr(cv2, 'CAP_DSHOW'):
        backends.append(('DSHOW', cv2.CAP_DSHOW))
    if hasattr(cv2, 'CAP_MSMF'):
        backends.append(('MSMF', cv2.CAP_MSMF))
    backends.append(('default', 0))

    for i in range(max_index, -1, -1):
        for _name, backend in backends:
            cap = cv2.VideoCapture(i, backend) if backend else cv2.VideoCapture(i)
            if cap.isOpened():
                cap.release()
                return (i, backend)
            cap.release()

    raise RuntimeError(
        'Could not find any working camera source. '
        'Make sure OBS Virtual Camera or your virtual webcam software is running.'
    )


# ── Interactive shell ─────────────────────────────────────────────────────────

class RobotShell(cmd.Cmd):
    """Direct interactive shell — calls state machine / robot Python APIs, no HTTP."""

    intro  = "MTE 380 — type 'help' for commands, 'exit' to quit."
    prompt = '(robot) '

    _STATES = ['idle', 'line_follow']

    def __init__(self, sm, robot):
        super().__init__()
        self._sm    = sm
        self._robot = robot

    # ── State ──────────────────────────────────────────────────────────────────

    def do_state(self, line: str):
        """state [name]  — get current state, or transition to <name>."""
        name = line.strip()
        if name:
            self._sm.transition(name)
            print(f'  → {name}')
        else:
            print(f'  {self._sm.current_state}')

    def complete_state(self, text, *_):
        return [s for s in self._STATES if s.startswith(text)]

    def do_start(self, _):
        """start  — begin line following."""
        self.do_state('line_follow')

    def do_stop(self, _):
        """stop  — return to idle and halt motors."""
        self.do_state('idle')

    # ── Motion ─────────────────────────────────────────────────────────────────

    def do_speed(self, line: str):
        """speed <0-1>  — drive both motors straight at this power (0 = stop)."""
        try:
            val = max(-1.0, min(1.0, float(line.strip())))
        except ValueError:
            print('  usage: speed <0-1>')
            return
        if not self._robot:
            print('  no robot')
            return
        self._robot.set_motors(val, val)
        print(f'  motors → {val}')

    def do_direction(self, line: str):
        """direction <x> <y>  — set direction vector (x=lateral [-1,1], y=forward [-1,1])."""
        parts = line.split()
        try:
            x, y = float(parts[0]), float(parts[1])
        except (IndexError, ValueError):
            print('  usage: direction <x> <y>')
            return
        if self._robot:
            self._robot.add_direction(x, y)
            print(f'  direction → ({x}, {y})')
        else:
            print('  no robot')

    def do_motors(self, line: str):
        """motors <left> <right>  — set raw motor outputs [-1, 1], bypassing PID."""
        parts = line.split()
        try:
            l, r = float(parts[0]), float(parts[1])
        except (IndexError, ValueError):
            print('  usage: motors <left> <right>')
            return
        if self._robot:
            self._robot.set_motors(l, r)
            print(f'  motors → L={l}  R={r}')
        else:
            print('  no robot')

    def do_servo(self, line: str):
        """servo <angle>  — set claw servo angle (0–180)."""
        try:
            angle = float(line.strip())
        except ValueError:
            print('  usage: servo <angle>')
            return
        if self._robot:
            self._robot.set_claw(angle)
            print(f'  servo → {angle}°')
        else:
            print('  no robot')

    # ── Tuning ─────────────────────────────────────────────────────────────────

    def do_gains(self, line: str):
        """gains <heading|speed> <kp> <ki> <kd>  — set PID gains."""
        parts = line.split()
        try:
            target = parts[0]
            kp, ki, kd = float(parts[1]), float(parts[2]), float(parts[3])
        except (IndexError, ValueError):
            print('  usage: gains <heading|speed> <kp> <ki> <kd>')
            return
        if not self._robot:
            print('  no robot')
            return
        if target == 'heading':
            self._robot.set_gains(kp, ki, kd)
        elif target == 'speed':
            self._robot.set_speed_gains(kp, ki, kd)
        else:
            print(f'  unknown target: {target!r}')
            return
        print(f'  {target} gains → kp={kp}  ki={ki}  kd={kd}')

    def complete_gains(self, text, *_):
        return [t for t in ('heading', 'speed') if t.startswith(text)]

    # ── Status ─────────────────────────────────────────────────────────────────

    def do_status(self, _):
        """status  — print a snapshot of robot state."""
        print(f'  state:    {self._sm.current_state}')
        if self._robot:
            info = self._robot.get_debug_info()
            print(f'  heading:  {info["heading_current"]:+.1f}° → {info["heading_target"]:+.1f}°')
            print(f'  velocity: {info["linear_speed_mps"]:+.3f} m/s  '
                  f'(setpoint {info["speed_setpoint_mps"]:+.3f} m/s)')
            print(f'  drive:    L={info["last_drive"][0]:+.3f}  R={info["last_drive"][1]:+.3f}')
            enc = info['encoders']
            print(f'  encoders: L={enc[0]:+d}  R={enc[1]:+d}')
            hb = info['heartbeat_age_s']
            print(f'  hb:       {f"{hb:.1f}s ago" if hb is not None else "none"}')
        x, y, h = self._sm.odometry.pose()
        print(f'  pose:     x={x:.3f}m  y={y:.3f}m  hdg={math.degrees(h):.1f}°')

    # ── Misc ───────────────────────────────────────────────────────────────────

    def do_exit(self, _):
        """exit  — shut down and quit."""
        return True

    def do_quit(self, _):
        """quit  — shut down and quit."""
        return True

    def default(self, line: str):
        print(f'  unknown command: {line.split()[0]!r}')

    def emptyline(self):
        pass


# ── Main serve command ────────────────────────────────────────────────────────

def cmd_serve(args):
    """Start the robot: state machine + web server. Blocks until Ctrl-C."""
    import state_machine.hardware.robot as _robot_module
    from state_machine.hardware.robot       import Robot, MAX_SPEED, MAX_ROT_SPEED
    from state_machine.vision.line_detector import LineDetector
    from state_machine.machine              import StateMachine
    from state_machine.states               import Idle, LineFollow
    from web_server.server                  import WebServer

    log = logging.getLogger('cli')

    if args.max_speed is not None:
        _robot_module.MAX_SPEED = args.max_speed
    if args.max_rot_speed is not None:
        _robot_module.MAX_ROT_SPEED = args.max_rot_speed

    # ── Robot ─────────────────────────────────────────────────────────────────
    robot = None
    if not args.no_robot:
        robot = Robot(args.port, args.baud)
        robot.start()
        log.info('Robot connected on %s', args.port)

    # ── Detector ──────────────────────────────────────────────────────────────
    detector = None
    if args.virtual_cam:
        log.info('Auto-detecting virtual webcam…')
        source, backend = _find_virtual_cam()
        log.info('Virtual webcam found at index %d', source)
        detector = LineDetector(source=source, backend=backend, debug=args.debug)
        detector.start()
    elif args.camera is not None:
        source = None if args.camera == 'pi' else (
            int(args.camera) if args.camera.isdigit() else args.camera
        )
        log.info('Camera source: %s', 'Picamera2' if source is None else source)
        detector = LineDetector(source=source, debug=args.debug)
        detector.start()

    # ── State machine ─────────────────────────────────────────────────────────
    sm = (
        StateMachine(robot, detector)
        .register(Idle())
        .register(LineFollow())
    )

    # ── Web server ────────────────────────────────────────────────────────────
    server = WebServer(state_machine=sm, host=args.host, port=args.web_port)
    server.start()
    log.info('Dashboard: http://<host>:%d', args.web_port)

    sm.start(args.initial_state)

    def _shutdown():
        log.info('Shutting down…')
        sm.stop()
        if detector:
            detector.stop()
        if robot:
            robot.stop()

    signal.signal(signal.SIGTERM, lambda *_: (_shutdown(), sys.exit(0)))

    try:
        import readline
        readline.parse_and_bind('tab: complete')
    except ImportError:
        pass

    shell = RobotShell(sm, robot)
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        print()
    finally:
        _shutdown()


# ── Low-level hardware debug commands ─────────────────────────────────────────

def _make_robot(args) -> Robot:
    robot = Robot(args.port, args.baud)
    robot.start()
    return robot


def cmd_encoders(robot: Robot, _args):
    stop = threading.Event()
    def _stream():
        while not stop.is_set():
            l, r = robot.get_encoders()
            print(f'\r  left={l:8d}  right={r:8d}', end='', flush=True)
            time.sleep(0.05)
        print()
    t = threading.Thread(target=_stream, daemon=True)
    t.start()
    try:
        input()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        t.join()


def cmd_heading(robot: Robot, _args):
    stop = threading.Event()
    def _stream():
        while not stop.is_set():
            current, target = robot.get_heading()
            error = (target - current + 180) % 360 - 180
            print(f'\r  current={current:7.1f}°  target={target:7.1f}°  error={error:+7.1f}°',
                  end='', flush=True)
            time.sleep(0.05)
        print()
    t = threading.Thread(target=_stream, daemon=True)
    t.start()
    try:
        input()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        t.join()


def cmd_serial(robot: Robot, _args):
    def _on_raw(data: bytes):
        print(' '.join(f'{b:02x}' for b in data))
    robot._bridge.on_raw = _on_raw
    try:
        input()
    except KeyboardInterrupt:
        pass
    finally:
        robot._bridge.on_raw = None


def cmd_drive(robot: Robot, args):
    robot.drive_ticks(args.left, args.right,
                      kp=args.kp, kd=args.kd, deadband=args.deadband)


def cmd_wheel(robot: Robot, args):
    if args.side == 'left':
        robot.drive_ticks(args.ticks, 0)
    else:
        robot.drive_ticks(0, args.ticks)


def cmd_motors(robot: Robot, args):
    robot.set_motors(args.left, args.right)
    try:
        input('  Running — press Enter or Ctrl-C to stop\n')
    except KeyboardInterrupt:
        pass


def cmd_servo(robot: Robot, args):
    robot.set_claw(args.angle)


HARDWARE_COMMANDS = {
    'encoders': cmd_encoders,
    'heading':  cmd_heading,
    'serial':   cmd_serial,
    'drive':    cmd_drive,
    'wheel':    cmd_wheel,
    'motors':   cmd_motors,
    'servo':    cmd_servo,
}


# ── Argument parser ───────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='MTE 380 robot CLI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--port', default='/dev/ttyACM0',
                        help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--debug', action='store_true')

    sub = parser.add_subparsers(dest='command', metavar='command')

    # ── Serve (default) ───────────────────────────────────────────────────────
    p = sub.add_parser('serve', help='Start robot brain (default when no command given)')

    robot_group = p.add_mutually_exclusive_group()
    robot_group.add_argument('--port', dest='port', default='/dev/ttyACM0',
                             help='Serial port for Arduino (default: /dev/ttyACM0)')
    robot_group.add_argument('--no-robot', action='store_true',
                             help='Skip serial connection (camera/UI only)')
    p.add_argument('--baud', type=int, default=115200)

    cam_group = p.add_mutually_exclusive_group()
    cam_group.add_argument('--virtual-cam', action='store_true',
                           help='Auto-detect a virtual webcam')
    cam_group.add_argument('--camera',
                           help='Camera source: integer index, "pi" for Picamera2, or a file/URL')

    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--web-port', type=int, default=8321)
    p.add_argument('--initial-state', default='idle',
                   choices=['idle', 'line_follow'])
    p.add_argument('--max-speed', type=float, default=None)
    p.add_argument('--max-rot-speed', type=float, default=None)

    # ── Hardware debug commands ───────────────────────────────────────────────
    sub.add_parser('encoders', help='Stream encoder ticks (Ctrl-C to stop)')
    sub.add_parser('heading',  help='Stream heading live (Ctrl-C to stop)')
    sub.add_parser('serial',   help='Print raw serial bytes (Ctrl-C to stop)')

    p = sub.add_parser('drive', help='Move both wheels by N ticks')
    p.add_argument('left', type=int)
    p.add_argument('right', type=int)
    p.add_argument('--kp', type=float, default=0.0005)
    p.add_argument('--kd', type=float, default=0.0)
    p.add_argument('--deadband', type=int, default=10)

    p = sub.add_parser('wheel', help='Move one wheel by N ticks')
    p.add_argument('side', choices=['left', 'right'])
    p.add_argument('ticks', type=int)

    p = sub.add_parser('motors', help='Set left/right motor speeds (-1 to 1)')
    p.add_argument('left', type=float)
    p.add_argument('right', type=float)

    p = sub.add_parser('servo', help='Set claw servo angle (0-180)')
    p.add_argument('angle', type=float)

    return parser


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = build_parser()
    args   = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )

    # Default: start the robot
    if args.command is None or args.command == 'serve':
        # serve subparser sets its own --port; top-level --port is for hw commands
        if args.command is None:
            # propagate top-level --port/--baud into args for cmd_serve
            if not hasattr(args, 'no_robot'):
                args.no_robot = False
            if not hasattr(args, 'virtual_cam'):
                args.virtual_cam = False
            if not hasattr(args, 'camera'):
                args.camera = None
            if not hasattr(args, 'host'):
                args.host = '0.0.0.0'
            if not hasattr(args, 'web_port'):
                args.web_port = 8321
            if not hasattr(args, 'initial_state'):
                args.initial_state = 'idle'
            if not hasattr(args, 'max_speed'):
                args.max_speed = None
            if not hasattr(args, 'max_rot_speed'):
                args.max_rot_speed = None
        cmd_serve(args)
        return

    # Hardware debug commands — connect to Arduino directly
    robot = _make_robot(args)
    try:
        HARDWARE_COMMANDS[args.command](robot, args)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()


if __name__ == '__main__':
    main()
