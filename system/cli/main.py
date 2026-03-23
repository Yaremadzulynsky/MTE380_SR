"""
MTE 380 system CLI.

Start the full mission (perception + state machine + motors):
  python -m cli.main
  python -m cli.main --serial-port /dev/ttyACM0 run
  python -m cli.main run --no-display --config my.yaml

PID tuner web UI:
  python -m cli.main serve
  python -m cli.main serve --web-port 8080 --config /path/to/config.yaml

Low-level hardware debug commands:
  python -m cli.main --serial-port /dev/ttyACM0 encoders
  python -m cli.main --serial-port /dev/ttyACM0 serial
  python -m cli.main --serial-port /dev/ttyACM0 motors 0.3 0.3
  python -m cli.main --serial-port /dev/ttyACM0 claw 90
"""
from __future__ import annotations

import argparse
import cmd
import logging
import os
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import config as _config_module
from config import Config


# ── Interactive shell ──────────────────────────────────────────────────────────

class MissionShell(cmd.Cmd):
    intro  = "MTE 380 — type 'help' for commands, 'quit' to exit."
    prompt = "(mission) "

    def __init__(self, brain):
        super().__init__()
        self._brain = brain

    def do_start(self, _: str):
        """start  — reload config.yaml and begin the mission."""
        self._brain.go()

    def do_go(self, line: str):
        """go  — alias for start."""
        self.do_start(line)

    def do_stop(self, _: str):
        """stop  — idle motors (serial stays open)."""
        self._brain.stop()

    def do_state(self, _: str):
        """state  — show current mission state."""
        print(f"  {self._brain.state}")

    def do_status(self, _: str):
        """status  — print a telemetry snapshot."""
        snap = self._brain.snapshot()
        print(f"  state:    {snap['sm_state']}")
        print(f"  running:  {snap['running']}")
        print(f"  encoders: L={snap['enc_l']:+d}  R={snap['enc_r']:+d}")
        print(f"  rpm:      L={snap['rpm_l']:+.1f}  R={snap['rpm_r']:+.1f}")
        det = snap["det"]
        if det is not None:
            print(f"  vision:   red={'Y' if det.red_found else 'N'}  "
                  f"err={det.red_error:+.3f}  "
                  f"blue={'Y' if det.blue_found else 'N'}  "
                  f"T={'Y' if det.t_junction else 'N'}")
        out = snap["output"]
        if out is not None:
            print(f"  drive:    L={out.left:+.3f}  R={out.right:+.3f}")
        print(f"  motors:   {snap['motor_telem']}")

    def do_gains(self, line: str):
        """gains <steer|motor> <kp> <ki> <kd>  — update gains live and save to config.yaml."""
        parts = line.split()
        try:
            target     = parts[0]
            kp, ki, kd = float(parts[1]), float(parts[2]), float(parts[3])
        except (IndexError, ValueError):
            print("  usage: gains <steer|motor> <kp> <ki> <kd>")
            return
        if target == "steer":
            self._brain.set_steer_gains(kp, ki, kd)
        elif target == "motor":
            self._brain.set_motor_gains(kp, ki, kd)
        else:
            print(f"  unknown target: {target!r}  (choose 'steer' or 'motor')")
            return
        print(f"  {target} gains → kp={kp}  ki={ki}  kd={kd}  (saved to config.yaml)")

    def complete_gains(self, text: str, *_):
        return [t for t in ("steer", "motor") if t.startswith(text)]

    def do_claw(self, line: str):
        """claw <angle>  — set servo angle (0–180°)."""
        try:
            angle = float(line.strip())
        except ValueError:
            print("  usage: claw <angle>")
            return
        self._brain.send_claw(angle)
        print(f"  claw → {angle}°")

    def do_motors(self, line: str):
        """motors <left> <right>  — speed fractions [-1, 1] via RPM PID."""
        parts = line.split()
        try:
            l, r = float(parts[0]), float(parts[1])
        except (IndexError, ValueError):
            print("  usage: motors <left> <right>")
            return
        self._brain.drive(l, r)
        print(f"  motors → L={l}  R={r}")

    def do_config(self, _: str):
        """config  — show all current config values."""
        cfg = _config_module.get()
        sections = [
            ("Steering PID",   ["steer_kp", "steer_ki", "steer_kd", "steer_out_limit"]),
            ("Motor PID",      ["motor_kp", "motor_ki", "motor_kd"]),
            ("Speed",          ["base_speed", "min_speed", "max_speed",
                                "search_turn", "search_turn_max", "lost_frames_before_search",
                                "lost_line_coast_speed"]),
            ("Mission",        ["forward_ticks", "forward_speed",
                                "turn_speed", "turn_duration_s"]),
            ("Claw",           ["claw_open", "claw_closed", "pickup_hold_s"]),
            ("Vision",         ["red_loss_debounce_frames", "red_error_ema_alpha",
                                "line_axle_extrap"]),
            ("Camera/runtime", ["camera_width", "camera_height", "fps", "roi_top_ratio"]),
        ]
        for section, keys in sections:
            print(f"  {section}:")
            for k in keys:
                print(f"    {k} = {getattr(cfg, k)}")

    def do_reload(self, _: str):
        """reload  — reload config.yaml without starting the mission."""
        fresh = _config_module.reload()
        self._brain.reconfigure(fresh)
        print(f"  Reloaded {_config_module._CONFIG_PATH} (mission not started)")

    def do_exit(self, _: str):
        """exit  — shut down and quit."""
        return True

    def do_quit(self, _: str):
        """quit  — shut down and quit."""
        return True

    def default(self, line: str):
        print(f"  unknown command: {line.split()[0]!r}")

    def emptyline(self):
        pass


# ── Run command ────────────────────────────────────────────────────────────────

def cmd_run(args: argparse.Namespace) -> None:
    from control import RobotBrain
    log = logging.getLogger("cli")

    cfg   = _config_module.get()
    brain = RobotBrain(args.serial_port, args.baud, args.dry_run, cfg)
    brain.start_loop(
        no_display       = args.no_display,
        no_telemetry     = args.no_telemetry,
        telemetry_every  = args.telemetry_every,
        telemetry_idle_s = args.telemetry_idle_s,
    )
    log.info("Mission loop started (camera live). Type 'start' to begin.")
    brain.send_claw(cfg.claw_open)

    signal.signal(signal.SIGTERM, lambda *_: (brain.shutdown(), sys.exit(0)))

    try:
        import readline
        readline.parse_and_bind("tab: complete")
    except ImportError:
        pass

    shell = MissionShell(brain)
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        print()
    finally:
        log.info("Shutting down…")
        brain.shutdown()


# ── Serve command (PID tuner web UI) ──────────────────────────────────────────

def cmd_serve(args: argparse.Namespace) -> None:
    import web_server.app as _server
    from control import RobotBrain

    cfg_path = _config_module._CONFIG_PATH
    if not cfg_path.exists():
        _config_module.save(Config(), cfg_path)
        print(f"[serve] Created default config at {cfg_path}", flush=True)

    cfg   = _config_module.get()
    brain = RobotBrain(args.serial_port, args.baud, args.dry_run, cfg)
    brain.start_loop(no_display=True, no_telemetry=True)
    brain.send_claw(cfg.claw_open)

    _server.set_runner(brain)

    print(f"[serve] Web UI → http://{args.host}:{args.web_port}", flush=True)
    print(f"[serve] Config file: {cfg_path}", flush=True)
    print("[serve] Press Ctrl-C to stop.", flush=True)

    flask_thread = threading.Thread(
        target=_server.app.run,
        kwargs=dict(host=args.host, port=args.web_port,
                    debug=False, threaded=True, use_reloader=False),
        daemon=True,
        name="flask",
    )
    flask_thread.start()

    try:
        while flask_thread.is_alive():
            flask_thread.join(timeout=1.0)
    except KeyboardInterrupt:
        print()
    finally:
        brain.shutdown()


# ── Hardware debug commands ────────────────────────────────────────────────────

def _make_brain(args: argparse.Namespace):
    from control import RobotBrain
    return RobotBrain(args.serial_port, args.baud, args.dry_run)


def cmd_encoders(brain, _args) -> None:
    stop = threading.Event()

    def _stream():
        while not stop.is_set():
            enc_l, enc_r = brain.encoder_ticks
            rpm_l, rpm_r = brain.measured_rpm
            print(
                f"\r  L={enc_l:8d} ({rpm_l:+6.1f} rpm)  R={enc_r:8d} ({rpm_r:+6.1f} rpm)",
                end="", flush=True,
            )
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


def cmd_serial(brain, _args) -> None:
    if brain.bridge is None:
        print("No serial bridge (dry-run mode?)")
        return

    def _on_raw(data: bytes):
        print(" ".join(f"{b:02x}" for b in data))

    brain.bridge.on_raw = _on_raw
    try:
        input("  Printing raw serial bytes — press Enter or Ctrl-C to stop\n")
    except KeyboardInterrupt:
        pass
    finally:
        brain.bridge.on_raw = None


def cmd_motors(brain, args) -> None:
    brain.drive(args.left, args.right)
    try:
        input(f"  Motors L={args.left}  R={args.right} — press Enter or Ctrl-C to stop\n")
    except KeyboardInterrupt:
        pass
    finally:
        brain.idle()


def cmd_claw(brain, args) -> None:
    brain.send_claw(args.angle)
    print(f"  Claw → {args.angle}°")


def cmd_voltage(brain, args) -> None:
    print(f"  Voltage L={args.left}  R={args.right} — Ctrl-C to stop", flush=True)
    try:
        while True:
            brain.send_voltage(args.left, args.right)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        brain.idle()


HARDWARE_COMMANDS = {
    "encoders": cmd_encoders,
    "serial":   cmd_serial,
    "motors":   cmd_motors,
    "voltage":  cmd_voltage,
    "claw":     cmd_claw,
}


# ── Argument parser ────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="MTE 380 system CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--serial-port", default="/dev/ttyACM0",
                        help="Arduino serial port")
    parser.add_argument("--baud",    type=int, default=115200)
    parser.add_argument("--dry-run", action="store_true",
                        help="Skip serial; print motor commands to stdout")
    parser.add_argument("--debug",   action="store_true")

    sub = parser.add_subparsers(dest="command", metavar="command")

    p = sub.add_parser("run", help="Start full mission (default)")
    p.add_argument("--config", default=str(_config_module._CONFIG_PATH),
                   help="Path to config.yaml")
    p.add_argument("--no-display", action="store_true",
                   default=not bool(os.environ.get("DISPLAY", "")))
    p.add_argument("--no-telemetry", action="store_true")
    p.add_argument("--telemetry-every",  type=int,   default=1, metavar="N")
    p.add_argument("--telemetry-idle-s", type=float, default=0.5)

    p = sub.add_parser("serve", help="Start the PID tuner web UI (http://<host>:5050)")
    p.add_argument("--config", default=str(_config_module._CONFIG_PATH),
                   help="Path to config.yaml (read/written by the UI)")
    p.add_argument("--host",     default="0.0.0.0")
    p.add_argument("--web-port", type=int, default=5050, metavar="PORT")

    sub.add_parser("encoders", help="Stream encoder ticks + RPM (Enter to stop)")
    sub.add_parser("serial",   help="Print raw serial bytes hex (Enter to stop)")

    p = sub.add_parser("motors", help="Set left/right speed fractions [-1, 1] via RPM PID")
    p.add_argument("left",  type=float)
    p.add_argument("right", type=float)

    p = sub.add_parser("voltage", help="Send direct voltage [-1, 1] bypassing RPM PID (Ctrl-C to stop)")
    p.add_argument("left",  type=float)
    p.add_argument("right", type=float)

    p = sub.add_parser("claw", help="Set claw servo angle (0–180°)")
    p.add_argument("angle", type=float)

    return parser


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    for i, arg in enumerate(sys.argv[1:]):
        if arg.startswith("--config="):
            _config_module.set_path(arg.split("=", 1)[1])
        elif arg == "--config" and i + 1 < len(sys.argv) - 1:
            _config_module.set_path(sys.argv[i + 2])

    parser = build_parser()
    args   = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    if args.command == "serve":
        cmd_serve(args)
        return

    if args.command is None or args.command == "run":
        if args.command is None:
            args.no_display        = not bool(os.environ.get("DISPLAY", ""))
            args.no_telemetry      = False
            args.telemetry_every   = 1
            args.telemetry_idle_s  = 0.5
        cmd_run(args)
        return

    brain = _make_brain(args)
    try:
        HARDWARE_COMMANDS[args.command](brain, args)
    except KeyboardInterrupt:
        pass
    finally:
        brain.shutdown()


if __name__ == "__main__":
    main()
