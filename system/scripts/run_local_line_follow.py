#!/usr/bin/env python3
"""
Main runner: perception → state machine → motor/servo control.

All tunable values live in config.yaml and are loaded via config.py.
Press 'g' to reload config.yaml and (re)start the mission.

Run:
    python run_local_line_follow.py
    python run_local_line_follow.py --config /path/to/config.yaml
    python run_local_line_follow.py --dry-run --no-display
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import cv2

import config as _config_module
from config import Config
from control import LocalMotorController, MotorCommand
from perception import Perception
from state_machine import MissionStateMachine, State


# ── CLI ───────────────────────────────────────────────────────────────────────

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="MTE 380 mission runner.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--config",      default=str(_config_module._CONFIG_PATH),
                   help="Path to config.yaml")
    p.add_argument("--serial-port", default="/dev/ttyACM0")
    p.add_argument("--baud",        type=int, default=115200)
    p.add_argument("--dry-run",     action="store_true")
    p.add_argument("--no-display",  action="store_true",
                   default=not bool(os.environ.get("DISPLAY", "")))
    p.add_argument("--no-telemetry",     action="store_true")
    p.add_argument("--telemetry-every",  type=int,   default=1, metavar="N")
    p.add_argument("--telemetry-idle-s", type=float, default=0.5)
    return p


# ── Keyboard listener ─────────────────────────────────────────────────────────

class KeyboardListener:
    """Background thread that reads single keypresses without blocking the loop."""

    def __init__(self) -> None:
        self._ch: str | None = None
        self._lock = threading.Lock()
        self._old: list | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._old = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        self._thread.start()

    def stop(self) -> None:
        if self._old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)
            self._old = None

    def get(self) -> str | None:
        with self._lock:
            ch, self._ch = self._ch, None
            return ch

    def _run(self) -> None:
        while True:
            try:
                ch = sys.stdin.read(1)
            except Exception:
                break
            with self._lock:
                self._ch = ch


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    # Pre-scan for --config so set_path() is called before any config.get()
    for i, arg in enumerate(sys.argv[1:]):
        if arg.startswith("--config="):
            _config_module.set_path(arg.split("=", 1)[1])
        elif arg == "--config" and i + 1 < len(sys.argv) - 1:
            _config_module.set_path(sys.argv[i + 2])

    args   = build_arg_parser().parse_args()
    cfg    = _config_module.get()
    period = 1.0 / max(cfg.fps, 1.0)

    should_stop = False
    def _handle_stop(_sig, _frame):
        nonlocal should_stop
        should_stop = True
    signal.signal(signal.SIGINT,  _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    perception = Perception(cfg)
    control    = LocalMotorController(args.serial_port, args.baud, args.dry_run, cfg)

    def _make_sm() -> MissionStateMachine:
        return MissionStateMachine(_config_module.get())

    sm      = _make_sm()
    running = False

    control.send_claw(cfg.claw_open)

    def go_reload_and_start() -> None:
        nonlocal sm, running, period
        fresh  = _config_module.reload()
        period = 1.0 / max(fresh.fps, 1.0)
        control.set_motor_pid(fresh)
        perception.reconfigure(fresh)
        sm      = _make_sm()
        running = True
        print(
            f"[KEY] go — reloaded {_config_module._CONFIG_PATH}  "
            f"steer_kp={fresh.steer_kp}  max_speed={fresh.max_speed}  "
            f"motor_kp={fresh.motor_kp:.6f}  geom={fresh.geom_enable}",
            flush=True,
        )

    print(
        f"Ready — serial={args.serial_port}  dry_run={args.dry_run}\n"
        f"  config:       {_config_module._CONFIG_PATH}\n"
        f"  steering PID: kp={cfg.steer_kp}  ki={cfg.steer_ki}  kd={cfg.steer_kd}\n"
        f"  motor    PID: kp={cfg.motor_kp:.6f}  ki={cfg.motor_ki}  kd={cfg.motor_kd}\n"
        f"  Press 'g' to go (reloads config), 's' to stop, 'q' to quit.",
        flush=True,
    )

    kb = KeyboardListener()
    kb.start()

    telemetry     = not args.no_telemetry
    frame_n       = 0
    t_start       = time.monotonic()
    last_idle_log = 0.0

    if telemetry:
        print(
            "# telemetry: t_s | state | vision | enc_L/R | motor cmd/tgt_rpm/rpm/V | claw | iter_ms",
            flush=True,
        )

    try:
        while not should_stop:
            t0 = time.time()
            frame_n += 1

            # ── Keyboard ──────────────────────────────────────────────────────
            key = kb.get()
            if key == "g" and not running:
                go_reload_and_start()
            elif key == "s" and running:
                running = False
                control.idle()
                print("[KEY] stop (motors idle, serial still open)", flush=True)
            elif key in ("q", "\x03"):
                break

            # ── Vision ────────────────────────────────────────────────────────
            frame = perception.read_frame()
            if frame is None:
                print("Camera read failed — stopping.", flush=True)
                break

            det = perception.detect(frame)

            # ── Control ───────────────────────────────────────────────────────
            if running:
                enc_l, enc_r = control.encoder_ticks
                output       = sm.step(det, enc_l, enc_r)

                control.send_drive(MotorCommand(left=output.left, right=output.right))
                if output.claw is not None:
                    control.send_claw(output.claw)

                if output.state == State.DONE:
                    running = False
                    control.idle()
                    print("Mission complete. Press 'g' to run again.", flush=True)

                rpm_l, rpm_r = control.measured_rpm
                status_line = (
                    f"state={output.state.value:14s}  "
                    f"err={det.red_error:+.2f}  "
                    f"blue={'Y' if det.blue_found else 'N'}  "
                    f"T={'Y' if det.t_junction else 'N'}  "
                    f"L={output.left:+.2f}({rpm_l:+.0f}rpm)  "
                    f"R={output.right:+.2f}({rpm_r:+.0f}rpm)"
                )
            else:
                output    = None
                rpm_l = rpm_r = 0.0
                status_line = "IDLE — press 'g' to start"

            # ── Display ───────────────────────────────────────────────────────
            if args.no_display and not telemetry:
                print(status_line, flush=True)
            elif not args.no_display:
                overlay = perception.draw_debug(frame, det)
                label = (
                    f"{output.state.value}  "
                    f"L={output.left:+.2f}({rpm_l:+.0f})  "
                    f"R={output.right:+.2f}({rpm_r:+.0f})"
                    if output else "IDLE — press 'g'"
                )
                cv2.putText(overlay, label, (10, overlay.shape[0] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("mission", overlay)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                elif k == ord("g") and not running:
                    go_reload_and_start()
                elif k == ord("s") and running:
                    running = False
                    control.idle()
                    print("[KEY] stop (motors idle, serial still open)", flush=True)

            elapsed = time.time() - t0
            iter_ms = elapsed * 1000.0

            # ── Terminal telemetry ─────────────────────────────────────────────
            if telemetry:
                t_run = time.monotonic() - t_start
                if output is not None and (frame_n % max(1, args.telemetry_every) == 0):
                    enc_l, enc_r = control.encoder_ticks
                    mt     = control.motor_telemetry_line()
                    claw_s = f"{output.claw:.0f}°" if output.claw is not None else "--"
                    print(
                        f"{t_run:8.3f}s  {output.state.value:12s}  "
                        f"red={'Y' if det.red_found else 'N'} err={det.red_error:+.3f}  "
                        f"blue={'Y' if det.blue_found else 'N'} T={'Y' if det.t_junction else 'N'}  "
                        f"claw={claw_s}  "
                        f"enc_L={enc_l:6d} enc_R={enc_r:6d}  {mt}  "
                        f"iter_ms={iter_ms:5.1f}",
                        flush=True,
                    )
                elif output is None and (time.monotonic() - last_idle_log >= args.telemetry_idle_s):
                    last_idle_log = time.monotonic()
                    print(
                        f"{t_run:8.3f}s  IDLE  "
                        f"red={'Y' if det.red_found else 'N'} err={det.red_error:+.3f}  "
                        f"blue={'Y' if det.blue_found else 'N'} T={'Y' if det.t_junction else 'N'}  "
                        f"iter_ms={iter_ms:5.1f}",
                        flush=True,
                    )

            if elapsed < period:
                time.sleep(period - elapsed)

    finally:
        kb.stop()
        control.shutdown()
        perception.release()
        cv2.destroyAllWindows()
        print("Stopped.", flush=True)


if __name__ == "__main__":
    main()
