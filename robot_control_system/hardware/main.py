"""
Robot control entry point.

Two PID loops run at RATE_HZ and write /cmd to the Arduino via SerialBridge:

  Heading PID
    setpoint  : desired heading = atan2(dir.x, dir.y)   (radians from forward)
    feedback  : set via robot.set_heading_feedback(radians)   (e.g. from IMU)
    output    : angular_z

  Speed PID
    setpoint  : dir.y * MAX_SPEED
    feedback  : set via robot.set_speed_feedback(m/s)         (e.g. from encoders)
    output    : linear_x

Direction input:
  Call robot.add_direction(x, y) from any thread.
  x : lateral  [-1, 1]   (left = -1, right = +1)
  y : forward  [-1, 1]   (back  = -1, fwd   = +1)

Usage:
  python main.py --port /dev/ttyACM0          # Linux / WSL
  python main.py --port COM3                  # Windows
  python main.py --port /dev/ttyACM0 --debug  # verbose logging
"""

import argparse
import logging
import threading
import time

from robot import Robot


def main():
    parser = argparse.ArgumentParser(description='MTE 380 robot controller')
    parser.add_argument('--port', default='/dev/ttyACM0',
                        help='Serial port (e.g. /dev/ttyACM0 or COM3)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )

    robot = Robot(args.port, args.baud)
    robot.start()

    wheel_kp       = 0.0005
    wheel_kd       = 0.0
    wheel_deadband = 10

    print("Commands:")
    print("  x y               — set direction (e.g. '0 1' = forward, '1 0' = rotate right)")
    print("  speed <-1 to 1>   — set forward/backward speed (e.g. 'speed 0.5')")
    print("  rotation <0-1>    — set rotation speed (e.g. 'rotation 0.5')")
    print("  encoders          — stream encoder ticks live (Enter to stop)")
    print("  heading           — stream current/target heading live (Enter to stop)")
    print("  serial            — print raw serial bytes from Arduino (Enter to stop)")
    print("  gains <kp> <ki> <kd>             — tune heading PID gains live")
    print("  gains speed <kp> <ki> <kd>       — tune speed PID gains live")
    print("  gains wheel <kp> <kd> <deadband> — tune wheel PD loop gains")
    print("  wheel <left|right> <ticks>       — move one wheel by N ticks")
    print("  drive <left> <right>             — move both wheels by N ticks")
    print("  motors <l> <r>    — set left/right motor speeds directly, e.g. 'motors 0.5 0.5'")
    print("  servo <angle>     — set claw servo angle 0-180, e.g. 'servo 90'")
    print("  0 0               — stop | Ctrl-C = quit\n")

    try:
        while True:
            try:
                raw = input('> ').strip()
                if not raw:
                    continue
                parts = raw.split()
                if parts[0] == 'gains':
                    if parts[1] == 'wheel':
                        if len(parts) != 5:
                            print('Usage: gains wheel <kp> <kd> <deadband>')
                            continue
                        wheel_kp       = float(parts[2])
                        wheel_kd       = float(parts[3])
                        wheel_deadband = int(parts[4])
                        print(f'  wheel gains → kp={wheel_kp}  kd={wheel_kd}  deadband={wheel_deadband} ticks')
                    elif parts[1] == 'speed':
                        if len(parts) != 5:
                            print('Usage: gains speed <kp> <ki> <kd>')
                            continue
                        robot.set_speed_gains(float(parts[2]), float(parts[3]), float(parts[4]))
                    elif len(parts) != 4:
                        print('Usage: gains <kp> <ki> <kd>')
                        continue
                    else:
                        robot.set_gains(float(parts[1]), float(parts[2]), float(parts[3]))
                elif parts[0] == 'drive':
                    if len(parts) != 3:
                        print('Usage: drive <left_ticks> <right_ticks>')
                        continue
                    robot.drive_ticks(int(parts[1]), int(parts[2]), kp=wheel_kp, kd=wheel_kd, deadband=wheel_deadband)
                elif parts[0] == 'servo':
                    if len(parts) != 2:
                        print('Usage: servo <angle>  (0-180)')
                        continue
                    robot.set_claw(float(parts[1]))
                elif parts[0] == 'motors':
                    if len(parts) != 3:
                        print('Usage: motors <left> <right>  (range -1 to 1)')
                        continue
                    robot.set_motors(float(parts[1]), float(parts[2]))
                elif parts[0] == 'serial':
                    def _on_raw(data: bytes):
                        print(' '.join(f'{b:02x}' for b in data))
                    robot._bridge.on_raw = _on_raw
                    input()
                    robot._bridge.on_raw = None
                elif parts[0] == 'heading':
                    stop_event = threading.Event()
                    def _stream_heading():
                        while not stop_event.is_set():
                            current, target = robot.get_heading()
                            error = target - current
                            # wrap error to [-180, 180]
                            error = (error + 180) % 360 - 180
                            print(f'\r  current={current:7.1f}°  target={target:7.1f}°  error={error:+7.1f}°', end='', flush=True)
                            time.sleep(0.05)
                        print()
                    t = threading.Thread(target=_stream_heading, daemon=True)
                    t.start()
                    input()
                    stop_event.set()
                    t.join()
                elif parts[0] == 'encoders':
                    stop_event = threading.Event()
                    def _stream():
                        while not stop_event.is_set():
                            l, r = robot.get_encoders()
                            print(f'\r  left={l:8d}  right={r:8d}', end='', flush=True)
                            time.sleep(0.05)
                        print()
                    t = threading.Thread(target=_stream, daemon=True)
                    t.start()
                    input()
                    stop_event.set()
                    t.join()
                elif parts[0] == 'speed':
                    if len(parts) != 2:
                        print('Usage: speed <-1 to 1>')
                        continue
                    robot.set_speed(float(parts[1]))
                elif parts[0] == 'rotation':
                    if len(parts) != 2:
                        print('Usage: rotation <0-1>')
                        continue
                    robot.set_rotation_scale(float(parts[1]))
                elif len(parts) == 2:
                    robot.add_direction(float(parts[0]), float(parts[1]))
                else:
                    print('Expected: x y  or  speed/rotation <0-1>')
            except ValueError:
                print('Invalid input')
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()


if __name__ == '__main__':
    main()
