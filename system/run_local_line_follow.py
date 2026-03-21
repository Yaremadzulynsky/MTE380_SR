#!/usr/bin/env python3
"""
Main runner: perception → state machine → motor/servo control.

PID values are loaded from pid_config.json at startup. Edit them live via
the PID tuner web UI (pid_tuner/app.py) and restart the runner to apply.

CLI flags override the config file when provided explicitly.
Run:
    python run_local_line_follow.py [--dry-run] [--no-display]
    python run_local_line_follow.py --steer-kp 0.8   # override one value
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import time
from pathlib import Path

import cv2

from local.control import LocalMotorController, MotorCommand, MotorPIDConfig
from local.perception import Perception
from local.state_machine import Config, MissionStateMachine, State

# ── Config file ───────────────────────────────────────────────────────────────

_DEFAULT_CONFIG = Path(__file__).parent / "pid_config.json"

_FALLBACK: dict = {
    "steer_kp": 0.65,   "steer_ki": 0.04,    "steer_kd": 0.10,  "steer_out_limit": 0.80,
    "motor_kp": 0.003125, "motor_ki": 0.0005, "motor_kd": 0.0,
    "base_speed": 0.28, "min_speed": 0.16,   "max_speed": 0.45, "search_turn": 0.18,
    "forward_ticks": 800, "forward_speed": 0.25,
    "turn_speed": 0.30,   "turn_duration_s": 2.2,
    "claw_open": 0.0,     "claw_closed": 90.0, "pickup_hold_s": 0.8,
}


def load_pid_config(path: Path) -> dict:
    try:
        raw = json.loads(path.read_text())
        return {**_FALLBACK, **{k: v for k, v in raw.items() if k in _FALLBACK}}
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"[config] Could not read {path}: {e} — using built-in defaults.", flush=True)
        return dict(_FALLBACK)


# ── CLI ───────────────────────────────────────────────────────────────────────

def build_arg_parser(cfg: dict) -> argparse.ArgumentParser:
    """Build argument parser with config-file values as defaults."""
    p = argparse.ArgumentParser(
        description="MTE 380 mission runner.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Camera / runtime
    p.add_argument("--width",         type=int,   default=640)
    p.add_argument("--height",        type=int,   default=480)
    p.add_argument("--fps",           type=float, default=30.0)
    p.add_argument("--roi-top-ratio", type=float, default=0.5)
    p.add_argument("--serial-port",   default="/dev/ttyACM0")
    p.add_argument("--baud",          type=int,   default=115200)
    p.add_argument("--dry-run",       action="store_true")
    p.add_argument("--no-display",    action="store_true",
                   default=not bool(os.environ.get("DISPLAY", "")))
    p.add_argument("--pid-config",    default=str(_DEFAULT_CONFIG),
                   help="Path to pid_config.json")

    # ── All tunable values — defaults come from the config file ──────────────
    p.add_argument("--steer-kp",              type=float, default=cfg["steer_kp"])
    p.add_argument("--steer-ki",              type=float, default=cfg["steer_ki"])
    p.add_argument("--steer-kd",              type=float, default=cfg["steer_kd"])
    p.add_argument("--steer-out-limit",       type=float, default=cfg["steer_out_limit"])

    p.add_argument("--motor-kp",              type=float, default=cfg["motor_kp"])
    p.add_argument("--motor-ki",              type=float, default=cfg["motor_ki"])
    p.add_argument("--motor-kd",              type=float, default=cfg["motor_kd"])

    p.add_argument("--base-speed",            type=float, default=cfg["base_speed"])
    p.add_argument("--min-speed",             type=float, default=cfg["min_speed"])
    p.add_argument("--max-speed",             type=float, default=cfg["max_speed"])
    p.add_argument("--search-turn",           type=float, default=cfg["search_turn"])

    p.add_argument("--forward-ticks",         type=int,   default=cfg["forward_ticks"])
    p.add_argument("--forward-speed",         type=float, default=cfg["forward_speed"])

    p.add_argument("--turn-speed",            type=float, default=cfg["turn_speed"])
    p.add_argument("--turn-duration",         type=float, default=cfg["turn_duration_s"])

    p.add_argument("--claw-open",             type=float, default=cfg["claw_open"])
    p.add_argument("--claw-closed",           type=float, default=cfg["claw_closed"])
    p.add_argument("--pickup-hold",           type=float, default=cfg["pickup_hold_s"])
    return p


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    # Load config file first so it becomes the argparse defaults
    cfg_path = _DEFAULT_CONFIG
    # Pre-scan for --pid-config before full parse so the path is respected
    import sys
    for i, arg in enumerate(sys.argv[1:]):
        if arg.startswith("--pid-config") and "=" in arg:
            cfg_path = Path(arg.split("=", 1)[1])
        elif arg == "--pid-config" and i + 1 < len(sys.argv) - 1:
            cfg_path = Path(sys.argv[i + 2])

    cfg  = load_pid_config(cfg_path)
    args = build_arg_parser(cfg).parse_args()
    period = 1.0 / max(args.fps, 1.0)

    should_stop = False
    def _handle_stop(_sig, _frame):
        nonlocal should_stop
        should_stop = True
    signal.signal(signal.SIGINT,  _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    perception = Perception(
        width=args.width,
        height=args.height,
        roi_top_ratio=args.roi_top_ratio,
    )
    control = LocalMotorController(
        serial_port=args.serial_port,
        baud=args.baud,
        dry_run=args.dry_run,
        motor_pid_cfg=MotorPIDConfig(
            kp=args.motor_kp,
            ki=args.motor_ki,
            kd=args.motor_kd,
        ),
    )
    sm = MissionStateMachine(Config(
        steer_kp=args.steer_kp,
        steer_ki=args.steer_ki,
        steer_kd=args.steer_kd,
        steer_out_limit=args.steer_out_limit,
        base_speed=args.base_speed,
        min_speed=args.min_speed,
        max_speed=args.max_speed,
        search_turn=args.search_turn,
        forward_ticks=args.forward_ticks,
        forward_speed=args.forward_speed,
        turn_speed=args.turn_speed,
        turn_duration_s=args.turn_duration,
        claw_open=args.claw_open,
        claw_closed=args.claw_closed,
        pickup_hold_s=args.pickup_hold,
    ))

    control.send_claw(args.claw_open)

    print(
        f"Mission started — serial={args.serial_port}  dry_run={args.dry_run}\n"
        f"  config      : {cfg_path}\n"
        f"  steering PID: kp={args.steer_kp}  ki={args.steer_ki}  kd={args.steer_kd}\n"
        f"  motor    PID: kp={args.motor_kp:.6f}  ki={args.motor_ki}  kd={args.motor_kd}",
        flush=True,
    )

    try:
        while not should_stop:
            t0    = time.time()
            frame = perception.read_frame()
            if frame is None:
                print("Camera read failed — stopping.", flush=True)
                break

            det         = perception.detect(frame)
            left, right = control.encoder_ticks
            output      = sm.step(det, left, right)

            control.send_drive(MotorCommand(left=output.left, right=output.right))

            if output.claw is not None:
                control.send_claw(output.claw)

            if output.state == State.DONE:
                print("Mission complete.", flush=True)
                break

            rpm_l, rpm_r = control.measured_rpm
            if args.no_display:
                print(
                    f"state={output.state.value:14s}  "
                    f"err={det.red_error:+.2f}  "
                    f"blue={'Y' if det.blue_found else 'N'}  "
                    f"T={'Y' if det.t_junction else 'N'}  "
                    f"L={output.left:+.2f}({rpm_l:+.0f}rpm)  "
                    f"R={output.right:+.2f}({rpm_r:+.0f}rpm)",
                    flush=True,
                )
            else:
                overlay = perception.draw_debug(frame, det)
                cv2.putText(
                    overlay,
                    f"{output.state.value}  "
                    f"L={output.left:+.2f}({rpm_l:+.0f})  "
                    f"R={output.right:+.2f}({rpm_r:+.0f})",
                    (10, overlay.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2, cv2.LINE_AA,
                )
                cv2.imshow("mission", overlay)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    finally:
        control.stop()
        perception.release()
        cv2.destroyAllWindows()
        print("Stopped.", flush=True)


if __name__ == "__main__":
    main()
