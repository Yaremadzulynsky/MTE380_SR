#!/usr/bin/env python3
"""
Rotation PID test — standalone CLI.

Usage:
    python scripts/test_rot_pid.py --port /dev/ttyACM0
    python scripts/test_rot_pid.py --port COM3 --degrees 180
    python scripts/test_rot_pid.py --port COM3 --degrees 90 --kp 1.0 --ki 0.1 --kd 0.0
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import config as _config_module
from control import LocalMotorController, RotationController


def run(args: argparse.Namespace) -> None:
    cfg = _config_module.get()

    if args.kp is not None: cfg.rot_kp = args.kp
    if args.ki is not None: cfg.rot_ki = args.ki
    if args.kd is not None: cfg.rot_kd = args.kd
    if args.tol is not None: cfg.rot_tolerance = args.tol

    print(f"rot_kp={cfg.rot_kp}  rot_ki={cfg.rot_ki}  rot_kd={cfg.rot_kd}  "
          f"tol={cfg.rot_tolerance}°  target={args.degrees}°", flush=True)

    control = LocalMotorController(
        serial_port=args.port,
        baud=args.baud,
        cfg=cfg,
    )

    try:
        time.sleep(0.5)  # let serial settle

        ctrl = RotationController(control, args.degrees, cfg=cfg)
        interval = 0.005  # 200 Hz
        t_start = time.monotonic()

        while not ctrl.done:
            t0 = time.monotonic()
            ctrl.step()
            traveled, target = ctrl.progress()
            error = ctrl.error_deg()
            print(f"\r  traveled={traveled:6.1f}°  error={error:+7.2f}°  "
                  f"t={time.monotonic() - t_start:.2f}s",
                  end="", flush=True)
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))

        print(f"\nDone in {time.monotonic() - t_start:.2f}s", flush=True)

    except KeyboardInterrupt:
        print("\nAborted.", flush=True)
    finally:
        control.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="Rotation PID test")
    parser.add_argument("--port",    default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--baud",    type=int,   default=115200)
    parser.add_argument("--degrees", type=float, default=90.0,  help="Target rotation (+ = CW)")
    parser.add_argument("--kp",      type=float, default=None)
    parser.add_argument("--ki",      type=float, default=None)
    parser.add_argument("--kd",      type=float, default=None)
    parser.add_argument("--tol",     type=float, default=None,  help="Tolerance in degrees")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
