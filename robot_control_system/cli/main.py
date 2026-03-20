"""
MTE 380 robot CLI.

Launch the robot brain (state machine + web server):
  python -m cli.main serve --port /dev/ttyACM0 --virtual-cam
  python -m cli.main serve --port COM3 --camera 0 --initial-state line_follow
  python -m cli.main serve --no-robot --virtual-cam   # vision only, no hardware

Interactive shell (embedded in serve, or connect to a remote process):
  python -m cli.main serve --port COM3     # starts robot + drops into shell
  python -m cli.main shell                 # attach to an already-running serve
  python -m cli.main shell --server http://pi.local:8321

Server commands (talk to a running 'serve' process):
  python -m cli.main start
  python -m cli.main stop
  python -m cli.main state get
  python -m cli.main state set <name>
  python -m cli.main status

  Use --server to point at a non-default host: --server http://pi.local:8321

Hardware commands (talk directly to Arduino over serial):
  python -m cli.main --port /dev/ttyACM0 <command> [args]
  python -m cli.main --port COM3         <command> [args]

  drive <left> <right>                — move both wheels by N ticks
  wheel <left|right> <ticks>          — move one wheel by N ticks
  motors <left> <right>               — set motor speeds directly (-1 to 1)
  servo <angle>                       — set claw angle (0-180)
  direction <x> <y>                   — set direction vector (-1 to 1 each)
  speed <value>                       — set forward/backward speed (-1 to 1)
  rotation <scale>                    — set rotation scale (0-1)
  gains heading <kp> <ki> <kd>        — tune heading PID
  gains speed <kp> <ki> <kd>          — tune speed PID
  gains wheel <kp> <kd> <deadband>    — tune wheel PD loop
  encoders                            — stream encoder ticks (Ctrl-C to stop)
  heading                             — stream heading live (Ctrl-C to stop)
  serial                              — print raw serial bytes (Ctrl-C to stop)
"""

import argparse
import cmd
import json
import logging
import signal
import sys
import threading
import time
import urllib.request
import urllib.error

sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent.parent))
from state_machine.hardware.robot import Robot

# Commands that talk to a running serve process over HTTP
_SERVER_COMMANDS = {'start', 'stop', 'state', 'status'}


# ── Camera helper ─────────────────────────────────────────────────────────────

def _find_virtual_cam(max_index: int = 10) -> tuple:
    """Scan webcam indices and return (index, backend) for the first that opens.

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
        for _name, backend in backends:
            cap = cv2.VideoCapture(i, backend) if backend else cv2.VideoCapture(i)
            if cap.isOpened():
                cap.release()
                return (i, backend)
            cap.release()

    raise RuntimeError(
        'Could not find any working camera source. '
        'Make sure OBS Virtual Camera is started or your virtual webcam software is running.'
    )


# ── HTTP helpers ──────────────────────────────────────────────────────────────

def _http_get(server: str, path: str) -> dict:
    url = server.rstrip('/') + path
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.loads(r.read())


def _http_post(server: str, path: str, body: dict) -> dict:
    url  = server.rstrip('/') + path
    data = json.dumps(body).encode()
    req  = urllib.request.Request(url, data=data,
                                  headers={'Content-Type': 'application/json'},
                                  method='POST')
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


# ── Server commands ───────────────────────────────────────────────────────────

def cmd_start(args):
    r = _http_post(args.server, '/api/state', {'state': 'line_follow'})
    print(f'  state → {r.get("state")}')


def cmd_stop(args):
    r = _http_post(args.server, '/api/state', {'state': 'stopped'})
    print(f'  state → {r.get("state")}')


def cmd_state(args):
    if args.state_action == 'get':
        r = _http_get(args.server, '/api/state')
        print(f'  current state: {r.get("state")}')
    elif args.state_action == 'set':
        r = _http_post(args.server, '/api/state', {'state': args.state_name})
        print(f'  state → {r.get("state")}')


def cmd_status(args):
    """Stream live status from the server's SSE endpoint and print to terminal."""
    url = args.server.rstrip('/') + '/events'
    print(f'  Streaming status from {url}  (Ctrl-C to stop)\n')
    try:
        req = urllib.request.urlopen(url, timeout=None)
        buf = ''
        while True:
            chunk = req.read(1).decode('utf-8', errors='replace')
            if not chunk:
                break
            buf += chunk
            if buf.endswith('\n\n'):
                for line in buf.strip().splitlines():
                    if line.startswith('data:'):
                        try:
                            d = json.loads(line[5:].strip())
                            _print_status(d)
                        except json.JSONDecodeError:
                            pass
                buf = ''
    except KeyboardInterrupt:
        print()


def _print_status(d: dict):
    parts = []

    state = d.get('state')
    if state:
        parts.append(f'state={state.upper():<12}')

    hw = d.get('hardware')
    if hw:
        hb  = hw.get('heartbeat_age_s')
        hdg = hw.get('heading', {})
        enc = hw.get('encoders', {})
        vel = hw.get('linear_speed_mps')
        parts.append(f'hb={hb:.1f}s' if hb is not None else 'hb=--')
        parts.append(f'hdg={hdg.get("current_deg", 0):+6.1f}°→{hdg.get("target_deg", 0):+6.1f}°')
        parts.append(f'enc=L{enc.get("left", 0):+7d} R{enc.get("right", 0):+7d}')
        if vel is not None:
            parts.append(f'vel={vel:+.3f}m/s')

    ln = d.get('line')
    if ln:
        if ln.get('found'):
            parts.append(f'line=FOUND  angle={ln.get("angle_deg", 0):+5.1f}°  '
                         f'lat={ln.get("lateral_distance_px", 0):+6.1f}px')
        else:
            parts.append('line=LOST')

    print('\r  ' + '   '.join(parts) + '   ', end='', flush=True)


def _print_status_snapshot(d: dict):
    """Pretty-print a single /api/status snapshot."""
    state = d.get('state') or '—'
    print(f'  state:     {state.upper()}')

    hw = d.get('hardware')
    if hw:
        hdg = hw.get('heading', {})
        enc = hw.get('encoders', {})
        drv = hw.get('last_drive', {})
        hb  = hw.get('heartbeat_age_s')
        print(f'  heading:   {hdg.get("current_deg", 0):+.1f}° → {hdg.get("target_deg", 0):+.1f}°')
        print(f'  encoders:  L={enc.get("left", 0):+d}  R={enc.get("right", 0):+d}')
        print(f'  speed:     {hw.get("speed_scale", 0):.3f}  ({hw.get("linear_speed_mps", 0):.3f} m/s)')
        print(f'  drive:     L={drv.get("left", 0):+.3f}  R={drv.get("right", 0):+.3f}')
        print(f'  heartbeat: {f"{hb:.1f}s ago" if hb is not None else "no data"}')

    od = d.get('odometry')
    if od:
        print(f'  pose:      x={od["x_m"]:.3f}m  y={od["y_m"]:.3f}m  '
              f'heading={od["heading_deg"]:.1f}°')

    ln = d.get('line')
    if ln:
        if ln.get('found'):
            print(f'  line:      FOUND  angle={ln.get("angle_deg", 0):+.1f}°  '
                  f'lat={ln.get("lateral_distance_px", 0):+.1f}px')
        else:
            print('  line:      LOST')


# ── Interactive shell ─────────────────────────────────────────────────────────

class RobotShell(cmd.Cmd):
    """Interactive shell for controlling the robot via the HTTP API."""

    intro  = "MTE 380 robot shell — type 'help' for commands, 'exit' to quit."
    prompt = '(robot) '

    _STATES = ['idle', 'stopped', 'line_follow', 'line_follow_p', 'line_follow_angle']

    def __init__(self, server: str):
        super().__init__()
        self._server = server

    # ── HTTP wrappers ──────────────────────────────────────────────────────────

    def _get(self, path: str) -> dict | None:
        try:
            return _http_get(self._server, path)
        except urllib.error.URLError as e:
            print(f'  error: cannot reach {self._server} — {e.reason}')
            return None

    def _post(self, path: str, body: dict) -> dict | None:
        try:
            return _http_post(self._server, path, body)
        except urllib.error.URLError as e:
            print(f'  error: cannot reach {self._server} — {e.reason}')
            return None

    # ── Commands ───────────────────────────────────────────────────────────────

    def do_state(self, line: str):
        """state [name]  — get current state, or transition to <name>."""
        name = line.strip()
        if name:
            r = self._post('/api/state', {'state': name})
            if r:
                print(f'  state → {r.get("state")}')
        else:
            r = self._get('/api/state')
            if r:
                print(f'  state: {r.get("state")}')

    def complete_state(self, text, line, begidx, endidx):
        return [s for s in self._STATES if s.startswith(text)]

    def do_start(self, _):
        """start  — transition to line_follow."""
        self.do_state('line_follow')

    def do_stop(self, _):
        """stop  — transition to stopped."""
        self.do_state('stopped')

    def do_speed(self, line: str):
        """speed <value>  — set forward speed (-1 to 1)."""
        try:
            val = float(line.strip())
        except ValueError:
            print('  usage: speed <value>')
            return
        r = self._post('/api/speed', {'speed': val})
        if r:
            print(f'  speed → {r.get("speed")}')

    def do_direction(self, line: str):
        """direction <x> <y>  — set direction vector (x=lateral, y=forward)."""
        parts = line.split()
        try:
            x, y = float(parts[0]), float(parts[1])
        except (IndexError, ValueError):
            print('  usage: direction <x> <y>')
            return
        r = self._post('/api/direction', {'x': x, 'y': y})
        if r:
            print(f'  direction → ({r.get("x")}, {r.get("y")})')

    def do_gains(self, line: str):
        """gains <heading|speed> <kp> <ki> <kd>  — set PID gains."""
        parts = line.split()
        try:
            pid = parts[0]
            kp, ki, kd = float(parts[1]), float(parts[2]), float(parts[3])
        except (IndexError, ValueError):
            print('  usage: gains <heading|speed> <kp> <ki> <kd>')
            return
        r = self._post('/api/gains', {'pid': pid, 'kp': kp, 'ki': ki, 'kd': kd})
        if r:
            g = r.get(pid, {})
            print(f'  {pid} gains → kp={g.get("kp")}  ki={g.get("ki")}  kd={g.get("kd")}')

    def complete_gains(self, text, line, begidx, endidx):
        return [t for t in ('heading', 'speed') if t.startswith(text)]

    def do_servo(self, line: str):
        """servo <angle>  — set claw servo angle (0–180)."""
        # servo isn't in the HTTP API yet — print a note
        print('  servo control is hardware-only; use: python -m cli.main servo <angle>')

    def do_status(self, _):
        """status  — print a snapshot of all robot state."""
        d = self._get('/api/status')
        if d:
            _print_status_snapshot(d)

    def do_pose(self, _):
        """pose  — print current odometry (x, y, heading)."""
        d = self._get('/api/status')
        if not d:
            return
        od = d.get('odometry')
        if od:
            print(f'  x={od["x_m"]:.4f}m  y={od["y_m"]:.4f}m  '
                  f'heading={od["heading_deg"]:.2f}°')
        else:
            print('  odometry not available')

    def do_exit(self, _):
        """exit  — quit the shell."""
        return True

    def do_quit(self, _):
        """quit  — quit the shell."""
        return True

    def default(self, line: str):
        print(f'  unknown command: {line.split()[0]!r}  — type "help" for commands')

    def emptyline(self):
        pass  # don't repeat the last command


def cmd_shell(args):
    """Start the interactive robot shell."""
    # Try to enable readline history (no-op on Windows without pyreadline)
    try:
        import readline
        readline.parse_and_bind('tab: complete')
    except ImportError:
        pass

    shell = RobotShell(args.server)
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        print()


# ── Serve command ─────────────────────────────────────────────────────────────

def cmd_serve(args):
    """Launch the robot brain: state machine + web server."""
    import state_machine.hardware.robot as _robot_module
    from state_machine.hardware.robot       import Robot, MAX_SPEED, MAX_ROT_SPEED
    from state_machine.vision.line_detector import LineDetector
    from state_machine.machine              import StateMachine
    from state_machine.states               import Idle, LineFollow, LineFollowTurn, FindLine
    from web_server.server                  import WebServer

    log = logging.getLogger('cli.serve')

    # ── Speed overrides ───────────────────────────────────────────────────────
    if args.max_speed is not None:
        _robot_module.MAX_SPEED = args.max_speed
        log.info('MAX_SPEED → %.2f', args.max_speed)
    if args.max_rot_speed is not None:
        _robot_module.MAX_ROT_SPEED = args.max_rot_speed
        log.info('MAX_ROT_SPEED → %.2f', args.max_rot_speed)

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
        detector = LineDetector(source=source, backend=backend, debug=True)
        detector.start()
    elif args.camera is not None:
        source = args.camera
        if source == 'pi':
            source = None
            log.info('Camera source: Picamera2')
        else:
            try:
                source = int(source)
                log.info('Camera source: webcam index %d', source)
            except ValueError:
                log.info('Camera source: %s', source)
        detector = LineDetector(source=source, debug=args.debug)
        detector.start()

    # ── State machine ─────────────────────────────────────────────────────────
    sm = (
        StateMachine(robot, detector)
        .register(Idle())
        .register(LineFollow())
        .register(LineFollowTurn())
        .register(FindLine())
    )

    # ── Web server ────────────────────────────────────────────────────────────
    server = WebServer(state_machine=sm, host=args.host, port=args.web_port)
    server.start()
    log.info('Dashboard: http://localhost:%d', args.web_port)

    sm.start(args.initial_state)

    def _shutdown():
        log.info('Shutting down…')
        sm.stop()
        if detector:
            detector.stop()
        if robot:
            robot.stop()

    signal.signal(signal.SIGTERM, lambda *_: (_shutdown(), sys.exit(0)))

    # ── Embedded interactive shell ─────────────────────────────────────────────
    shell = RobotShell(f'http://localhost:{args.web_port}')
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        print()
    finally:
        _shutdown()


# ── Hardware helpers ──────────────────────────────────────────────────────────

def _make_robot(args) -> Robot:
    robot = Robot(args.port, args.baud)
    robot.start()
    return robot


# ── Hardware commands ─────────────────────────────────────────────────────────

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


def cmd_direction(robot: Robot, args):
    robot.add_direction(args.x, args.y)
    try:
        input('  Holding direction — press Enter or Ctrl-C to stop\n')
    except KeyboardInterrupt:
        pass


def cmd_speed(robot: Robot, args):
    robot.set_speed(args.value)
    try:
        input('  Running — press Enter or Ctrl-C to stop\n')
    except KeyboardInterrupt:
        pass


def cmd_rotation(robot: Robot, args):
    robot.set_rotation_scale(args.scale)


def cmd_gains(robot: Robot, args):
    if args.gains_target == 'heading':
        robot.set_gains(args.kp, args.ki, args.kd)
    elif args.gains_target == 'speed':
        robot.set_speed_gains(args.kp, args.ki, args.kd)
    elif args.gains_target == 'wheel':
        print(f'  wheel gains → kp={args.kp}  kd={args.kd}  deadband={args.deadband}')


HARDWARE_COMMANDS = {
    'encoders':  cmd_encoders,
    'heading':   cmd_heading,
    'serial':    cmd_serial,
    'drive':     cmd_drive,
    'wheel':     cmd_wheel,
    'motors':    cmd_motors,
    'servo':     cmd_servo,
    'direction': cmd_direction,
    'speed':     cmd_speed,
    'rotation':  cmd_rotation,
    'gains':     cmd_gains,
}


# ── Parser ────────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='MTE 380 robot CLI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    # Hardware connection (used by hardware commands)
    parser.add_argument('--port', default='/dev/ttyACM0',
                        help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200)
    # Server connection (used by server commands)
    parser.add_argument('--server', default='http://localhost:8321',
                        help='Server URL for state/status commands (default: http://localhost:8321)')
    parser.add_argument('--debug', action='store_true')

    sub = parser.add_subparsers(dest='command', metavar='command')
    sub.required = False

    # ── Serve ─────────────────────────────────────────────────────────────────
    p = sub.add_parser('serve', help='Launch robot brain: state machine + web server')

    robot_group = p.add_mutually_exclusive_group()
    robot_group.add_argument('--port', default='/dev/ttyACM0',
                             dest='port',
                             help='Serial port for Arduino (default: /dev/ttyACM0)')
    robot_group.add_argument('--no-robot', action='store_true',
                             help='Skip serial connection (camera/UI only)')
    p.add_argument('--baud', type=int, default=115200)

    cam_group = p.add_mutually_exclusive_group()
    cam_group.add_argument('--virtual-cam', action='store_true',
                           help='Auto-detect a virtual webcam (OBS Virtual Camera, DroidCam, etc.)')
    cam_group.add_argument('--camera',
                           help='Camera source: integer index, "pi" for Picamera2, or a file/URL')

    p.add_argument('--host', default='0.0.0.0',
                   help='Web server bind address (default: 0.0.0.0)')
    p.add_argument('--web-port', type=int, default=8321,
                   help='Web server HTTP port (default: 8321)')
    p.add_argument('--initial-state', default='idle',
                   choices=['idle', 'stopped', 'line_follow', 'line_follow_p', 'line_follow_angle'],
                   help='Initial state machine state (default: idle)')
    p.add_argument('--max-speed', type=float, default=None,
                   help='Override MAX_SPEED (0–1)')
    p.add_argument('--max-rot-speed', type=float, default=None,
                   help='Override MAX_ROT_SPEED (0–1)')

    # ── Shell ─────────────────────────────────────────────────────────────────
    sub.add_parser('shell',  help='Interactive shell (connects to running serve process)')

    # ── Server commands ───────────────────────────────────────────────────────
    sub.add_parser('start',  help='Start line following (server)')
    sub.add_parser('stop',   help='Stop the robot (server)')
    sub.add_parser('status', help='Stream live status from server (Ctrl-C to stop)')

    p = sub.add_parser('state', help='Get or set state machine state (server)')
    state_sub = p.add_subparsers(dest='state_action', metavar='action')
    state_sub.required = True
    state_sub.add_parser('get', help='Print current state')
    sp = state_sub.add_parser('set', help='Transition to a state')
    sp.add_argument('state_name')

    # ── Hardware commands ─────────────────────────────────────────────────────
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

    p = sub.add_parser('direction', help='Set direction vector')
    p.add_argument('x', type=float, help='Lateral  [-1=left,  1=right]')
    p.add_argument('y', type=float, help='Forward  [-1=back,  1=fwd]')

    p = sub.add_parser('speed', help='Set forward/backward speed (-1 to 1)')
    p.add_argument('value', type=float)

    p = sub.add_parser('rotation', help='Set rotation scale (0-1)')
    p.add_argument('scale', type=float)

    p = sub.add_parser('gains', help='Tune PID gains')
    gains_sub = p.add_subparsers(dest='gains_target', metavar='target')
    gains_sub.required = True
    for target in ('heading', 'speed'):
        gp = gains_sub.add_parser(target)
        gp.add_argument('kp', type=float)
        gp.add_argument('ki', type=float)
        gp.add_argument('kd', type=float)
    gp = gains_sub.add_parser('wheel')
    gp.add_argument('kp', type=float)
    gp.add_argument('kd', type=float)
    gp.add_argument('deadband', type=int)

    return parser


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = build_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )

    if args.command is None or args.command == 'shell':
        cmd_shell(args)
        return

    if args.command == 'serve':
        cmd_serve(args)
        return

    if args.command in _SERVER_COMMANDS:
        try:
            {'start': cmd_start, 'stop': cmd_stop,
             'state': cmd_state, 'status': cmd_status}[args.command](args)
        except urllib.error.URLError as e:
            print(f'Cannot reach server at {args.server}: {e.reason}', file=sys.stderr)
            sys.exit(1)
        return

    # Hardware commands — connect to robot over serial
    robot = _make_robot(args)
    robot.set_speed(0.01)
    try:
        HARDWARE_COMMANDS[args.command](robot, args)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()


if __name__ == '__main__':
    main()
