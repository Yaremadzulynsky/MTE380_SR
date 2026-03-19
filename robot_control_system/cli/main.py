"""
MTE 380 robot CLI.

Hardware commands (talk directly to Arduino over serial):
  python -m cli.main --port /dev/ttyACM0 <command> [args]
  python -m cli.main --port COM3         <command> [args]

Server commands (talk to a running state_machine/main.py server):
  python -m cli.main start
  python -m cli.main stop
  python -m cli.main state get
  python -m cli.main state set <name>
  python -m cli.main status

  Use --server to point at a non-default host: --server http://pi.local:8321

Hardware commands:
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
import json
import logging
import sys
import threading
import time
import urllib.request
import urllib.error

sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent.parent))
from hardware.robot import Robot

# Commands that talk to the HTTP server instead of serial
_SERVER_COMMANDS = {'start', 'stop', 'state', 'status'}


# ── HTTP helpers ─────────────────────────────────────────────────────────────

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


# ── Hardware helpers ──────────────────────────────────────────────────────────

def _make_robot(args) -> Robot:
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )
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


def cmd_servo(robot: Robot, args):
    robot.set_claw(args.angle)


def cmd_direction(robot: Robot, args):
    robot.add_direction(args.x, args.y)


def cmd_speed(robot: Robot, args):
    robot.set_speed(args.value)


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
    # Hardware connection
    parser.add_argument('--port', default='/dev/ttyACM0',
                        help='Serial port for hardware commands (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200)
    # Server connection
    parser.add_argument('--server', default='http://localhost:8321',
                        help='Server URL for state/status commands (default: http://localhost:8321)')
    parser.add_argument('--debug', action='store_true')

    sub = parser.add_subparsers(dest='command', metavar='command')
    sub.required = True

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

    if args.command in _SERVER_COMMANDS:
        # Server commands — no serial connection needed
        try:
            {'start': cmd_start, 'stop': cmd_stop,
             'state': cmd_state, 'status': cmd_status}[args.command](args)
        except urllib.error.URLError as e:
            print(f'Cannot reach server at {args.server}: {e.reason}', file=sys.stderr)
            sys.exit(1)
        return

    # Hardware commands — connect to robot over serial
    robot = _make_robot(args)
    try:
        HARDWARE_COMMANDS[args.command](robot, args)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()


if __name__ == '__main__':
    main()
