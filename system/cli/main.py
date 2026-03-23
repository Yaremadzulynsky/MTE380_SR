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
import collections
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


# ── Mission runner (background thread) ────────────────────────────────────────

class MissionRunner:
    """Runs the perception → state machine → motor control loop in a background thread."""

    def __init__(self, control, args: argparse.Namespace):
        from perception import Perception
        self._control = control
        self.control  = control   # public access for web server
        self._args    = args

        self._perception = Perception(_config_module.get())
        self._sm         = None
        self._running    = False
        self._quit       = threading.Event()
        self._lock       = threading.Lock()

        self._last_det    = None
        self._last_output = None
        self._active_ctrl      = None   # active PositionController or RotationController
        self._active_ctrl_type: str | None = None  # "position" or "rotation"
        # Circular telemetry history: each entry is (t_rel, red_error, rpm_l, rpm_r)
        self._telem_history: collections.deque = collections.deque(maxlen=300)
        self._frame_n     = 0
        self._t_start     = time.monotonic()
        self._period      = 1.0 / max(_config_module.get().fps, 1.0)

        self._thread = threading.Thread(target=self._loop, daemon=True, name="mission-loop")

    def start_loop(self) -> None:
        self._thread.start()

    def go(self) -> None:
        """Reload config.yaml and start the mission."""
        fresh = _config_module.reload()
        self._period = 1.0 / max(fresh.fps, 1.0)
        self._control.set_motor_pid(fresh)
        self._perception.reconfigure(fresh)

        from state_machine import MissionStateMachine
        with self._lock:
            self._sm      = MissionStateMachine(fresh)
            self._running = True

        print(
            f"[go] mission started — "
            f"steer_kp={fresh.steer_kp}  max_speed={fresh.max_speed}  "
            f"motor_kp={fresh.motor_kp:.6f}",
            flush=True,
        )

    def reconfigure(self, cfg) -> None:
        """Apply updated config to perception and live state machine without restarting."""
        self._perception.reconfigure(cfg)
        self._control.set_motor_pid(cfg)
        with self._lock:
            if self._sm is not None:
                self._sm.cfg = cfg
                self._sm._steer_pid.tunings = (cfg.steer_kp, cfg.steer_ki, cfg.steer_kd)
                self._sm._steer_pid.output_limits = (-cfg.steer_out_limit, cfg.steer_out_limit)

    def run_position(self, delta_ticks: int) -> None:
        """Stop the mission and drive delta_ticks in a background thread."""
        from control import PositionController
        with self._lock:
            self._running = False
        self._control.idle()

        def _move():
            ctrl = PositionController(self._control, delta_ticks)
            self._active_ctrl = ctrl
            self._active_ctrl_type = "position"
            while not ctrl.done:
                ctrl.step()
                time.sleep(0.02)
            self._active_ctrl = None
            self._active_ctrl_type = None
            self._control.idle()
            print(f"[pos-move] done  delta={delta_ticks} ticks", flush=True)

        threading.Thread(target=_move, daemon=True, name="pos-move").start()

    def run_rotation(self, degrees: float) -> None:
        """Stop the mission and run the rotation PID synchronously (blocks until done)."""
        from control import RotationController
        with self._lock:
            self._running = False
        self._control.idle()

        ctrl = RotationController(self._control, degrees)
        self._active_ctrl = ctrl
        self._active_ctrl_type = "rotation"
        interval = 0.005  # 200 Hz
        while not ctrl.done:
            t0 = time.monotonic()
            ctrl.step()
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))
        self._active_ctrl = None
        self._active_ctrl_type = None
        self._control.idle()
        print(f"[rot-move] done  degrees={degrees}", flush=True)

    def move_status(self) -> dict:
        """Return progress of the currently running position/rotation move, if any."""
        ctrl = self._active_ctrl
        if ctrl is None:
            return {"active": None}
        traveled, target = ctrl.progress()
        remaining = max(0.0, target - traveled)
        return {
            "active":    self._active_ctrl_type,
            "done":      ctrl.done,
            "traveled":  round(traveled,  1),
            "target":    round(target,    1),
            "remaining": round(remaining, 1),
        }

    def stop_move(self) -> None:
        """Cancel any active position/rotation move and idle motors immediately."""
        ctrl = self._active_ctrl
        if ctrl is not None:
            ctrl.done = True          # signals the move thread to exit its loop
        self._active_ctrl      = None
        self._active_ctrl_type = None
        self._control.idle()

    def stop(self) -> None:
        with self._lock:
            self._running = False
        self._control.idle()
        print("[stop] motors idle, serial open", flush=True)

    def shutdown(self) -> None:
        self._quit.set()
        with self._lock:
            self._running = False
        self._control.idle()
        self._thread.join(timeout=2.0)
        self._perception.release()
        self._control.shutdown()

    def set_steer_gains(self, kp: float, ki: float, kd: float) -> None:
        cfg = _config_module.get()
        cfg.steer_kp = kp
        cfg.steer_ki = ki
        cfg.steer_kd = kd
        _config_module.save(cfg)
        with self._lock:
            if self._sm is not None:
                self._sm.cfg.steer_kp = kp
                self._sm.cfg.steer_ki = ki
                self._sm.cfg.steer_kd = kd
                self._sm._steer_pid.tunings = (kp, ki, kd)
                self._sm._steer_pid.reset()

    def set_motor_gains(self, kp: float, ki: float, kd: float) -> None:
        cfg = _config_module.get()
        cfg.motor_kp = kp
        cfg.motor_ki = ki
        cfg.motor_kd = kd
        _config_module.save(cfg)
        self._control.set_motor_pid(cfg)

    @property
    def state(self) -> str:
        with self._lock:
            return self._sm.state.value if self._sm else "not started"

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def get_annotated_frame(self):
        """Return the latest frame with detection overlay (BGR ndarray), or None."""
        frame = self._perception.get_frame()
        if frame is None:
            return None
        with self._lock:
            det = self._last_det
        if det is not None:
            return self._perception.draw_debug(frame, det)
        return frame

    def get_mask_frame(self):
        """Return the latest red-mask frame (BGR ndarray), or None."""
        return self._perception.get_red_mask_frame()

    def telemetry_history(self) -> dict:
        """Return lists of time-series data for graphing."""
        rows = list(self._telem_history)
        if not rows:
            return {"t": [], "red_error": [], "rpm_l": [], "rpm_r": []}
        t, err, rpm_l, rpm_r = zip(*rows)
        return {"t": list(t), "red_error": list(err), "rpm_l": list(rpm_l), "rpm_r": list(rpm_r)}

    def snapshot(self) -> dict:
        with self._lock:
            det      = self._last_det
            output   = self._last_output
            sm_state = self._sm.state.value if self._sm else "N/A"
            running  = self._running
        enc_l, enc_r = self._control.encoder_ticks
        rpm_l, rpm_r = self._control.measured_rpm
        return {
            "sm_state":    sm_state,
            "running":     running,
            "enc_l":       enc_l,  "enc_r": enc_r,
            "rpm_l":       rpm_l,  "rpm_r": rpm_r,
            "det":         det,
            "output":      output,
            "motor_telem": self._control.motor_telemetry_line(),
        }

    def _loop(self) -> None:
        import cv2
        from state_machine import State
        from control import MotorCommand

        args        = self._args
        telemetry   = not getattr(args, "no_telemetry", False)
        no_display  = getattr(args, "no_display", True)
        telem_every = max(1, getattr(args, "telemetry_every", 1))
        telem_idle  = getattr(args, "telemetry_idle_s", 0.5)
        last_idle   = 0.0

        while not self._quit.is_set():
            t0 = time.time()
            self._frame_n += 1

            frame = self._perception.read_frame()
            if frame is None:
                print("Camera read failed — stopping loop.", flush=True)
                break

            det = self._perception.detect(frame)

            with self._lock:
                running = self._running
                sm      = self._sm

            if running and sm is not None:
                enc_l, enc_r = self._control.encoder_ticks
                output       = sm.step(det, enc_l, enc_r)

                if output.direct_voltage:
                    self._control.send_voltage(output.left, output.right)
                else:
                    self._control.send_drive(MotorCommand(left=output.left, right=output.right))
                if output.claw is not None:
                    self._control.send_claw(output.claw)

                if output.state == State.DONE:
                    with self._lock:
                        self._running = False
                    self._control.idle()
                    print("Mission complete. Type 'start' to run again.", flush=True)

                with self._lock:
                    self._last_det    = det
                    self._last_output = output
            else:
                output = None
                with self._lock:
                    self._last_det    = det
                    self._last_output = None

            rpm_l, rpm_r = self._control.measured_rpm
            self._telem_history.append((
                time.monotonic() - self._t_start,
                det.red_error if det else 0.0,
                rpm_l,
                rpm_r,
            ))

            if not no_display:
                overlay = self._perception.draw_debug(frame, det)
                if output:
                    label = f"{output.state.value}  L={output.left:+.2f}  R={output.right:+.2f}"
                    cv2.putText(overlay, label, (10, overlay.shape[0] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("mission", overlay)
                cv2.waitKey(1)

            if telemetry:
                t_run   = time.monotonic() - self._t_start
                iter_ms = (time.time() - t0) * 1000.0
                if output is not None and (self._frame_n % telem_every == 0):
                    enc_l, enc_r = self._control.encoder_ticks
                    mt     = self._control.motor_telemetry_line()
                    claw_s = f"{output.claw:.0f}°" if output.claw is not None else "--"
                    print(
                        f"{t_run:8.3f}s  {output.state.value:12s}  "
                        f"red={'Y' if det.red_found else 'N'} err={det.red_error:+.3f}  "
                        f"blue={'Y' if det.blue_found else 'N'} T={'Y' if det.t_junction else 'N'}  "
                        f"claw={claw_s}  enc_L={enc_l:6d} enc_R={enc_r:6d}  {mt}  "
                        f"iter_ms={iter_ms:5.1f}",
                        flush=True,
                    )
                elif output is None and (time.monotonic() - last_idle >= telem_idle):
                    last_idle = time.monotonic()
                    print(
                        f"{t_run:8.3f}s  IDLE  "
                        f"red={'Y' if det.red_found else 'N'} err={det.red_error:+.3f}  "
                        f"blue={'Y' if det.blue_found else 'N'} T={'Y' if det.t_junction else 'N'}  "
                        f"iter_ms={iter_ms:5.1f}",
                        flush=True,
                    )

            elapsed = time.time() - t0
            if elapsed < self._period:
                time.sleep(self._period - elapsed)

        if not no_display:
            import cv2 as _cv2
            _cv2.destroyAllWindows()


# ── Interactive shell ──────────────────────────────────────────────────────────

class MissionShell(cmd.Cmd):
    intro  = "MTE 380 — type 'help' for commands, 'quit' to exit."
    prompt = "(mission) "

    def __init__(self, runner: MissionRunner, control, args: argparse.Namespace):
        super().__init__()
        self._runner  = runner
        self._control = control
        self._args    = args

    def do_start(self, _: str):
        """start  — reload config.yaml and begin the mission."""
        self._runner.go()

    def do_go(self, line: str):
        """go  — alias for start."""
        self.do_start(line)

    def do_stop(self, _: str):
        """stop  — idle motors (serial stays open)."""
        self._runner.stop()

    def do_state(self, _: str):
        """state  — show current mission state."""
        print(f"  {self._runner.state}")

    def do_status(self, _: str):
        """status  — print a telemetry snapshot."""
        snap = self._runner.snapshot()
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
            self._runner.set_steer_gains(kp, ki, kd)
        elif target == "motor":
            self._runner.set_motor_gains(kp, ki, kd)
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
        self._control.send_claw(angle)
        print(f"  claw → {angle}°")

    def do_motors(self, line: str):
        """motors <left> <right>  — raw motor fractions [-1, 1], bypasses PID."""
        from control import MotorCommand
        parts = line.split()
        try:
            l, r = float(parts[0]), float(parts[1])
        except (IndexError, ValueError):
            print("  usage: motors <left> <right>")
            return
        self._control.send_drive(MotorCommand(left=l, right=r))
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
        self._runner._period = 1.0 / max(fresh.fps, 1.0)
        self._control.set_motor_pid(fresh)
        self._runner._perception.reconfigure(fresh)
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
    from control import LocalMotorController
    log = logging.getLogger("cli")

    cfg     = _config_module.get()
    control = LocalMotorController(
        serial_port=args.serial_port,
        baud=args.baud,
        dry_run=args.dry_run,
        cfg=cfg,
    )

    runner = MissionRunner(control, args)
    runner.start_loop()
    log.info("Mission loop started (camera live). Type 'start' to begin.")
    control.send_claw(cfg.claw_open)

    def _shutdown():
        log.info("Shutting down…")
        runner.shutdown()

    signal.signal(signal.SIGTERM, lambda *_: (_shutdown(), sys.exit(0)))

    try:
        import readline
        readline.parse_and_bind("tab: complete")
    except ImportError:
        pass

    shell = MissionShell(runner, control, args)
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        print()
    finally:
        _shutdown()


# ── Serve command (PID tuner web UI) ──────────────────────────────────────────

def cmd_serve(args: argparse.Namespace) -> None:
    import web_server.app as _server
    from control import LocalMotorController

    cfg_path = _config_module._CONFIG_PATH
    if not cfg_path.exists():
        _config_module.save(Config(), cfg_path)
        print(f"[serve] Created default config at {cfg_path}", flush=True)

    cfg = _config_module.get()
    control = LocalMotorController(
        serial_port=args.serial_port,
        baud=args.baud,
        dry_run=args.dry_run,
        cfg=cfg,
    )

    # Inject defaults that MissionRunner expects from argparse
    args.no_display       = True
    args.no_telemetry     = True
    args.telemetry_every  = 1
    args.telemetry_idle_s = 0.5

    runner = MissionRunner(control, args)
    runner.start_loop()
    control.send_claw(cfg.claw_open)

    _server.set_runner(runner)

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
        runner.shutdown()


# ── Hardware debug commands ────────────────────────────────────────────────────

def _make_controller(args: argparse.Namespace):
    from control import LocalMotorController
    return LocalMotorController(
        serial_port=args.serial_port,
        baud=args.baud,
        dry_run=args.dry_run,
    )


def cmd_encoders(ctrl, _args) -> None:
    stop = threading.Event()

    def _stream():
        while not stop.is_set():
            enc_l, enc_r = ctrl.encoder_ticks
            rpm_l, rpm_r = ctrl.measured_rpm
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


def cmd_serial(ctrl, _args) -> None:
    if ctrl.bridge is None:
        print("No serial bridge (dry-run mode?)")
        return

    def _on_raw(data: bytes):
        print(" ".join(f"{b:02x}" for b in data))

    ctrl.bridge.on_raw = _on_raw
    try:
        input("  Printing raw serial bytes — press Enter or Ctrl-C to stop\n")
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.bridge.on_raw = None


def cmd_motors(ctrl, args) -> None:
    from control import MotorCommand
    ctrl.send_drive(MotorCommand(left=args.left, right=args.right))
    try:
        input(f"  Motors L={args.left}  R={args.right} — press Enter or Ctrl-C to stop\n")
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.idle()


def cmd_claw(ctrl, args) -> None:
    ctrl.send_claw(args.angle)
    print(f"  Claw → {args.angle}°")


def cmd_voltage(ctrl, args) -> None:
    print(f"  Voltage L={args.left}  R={args.right} — Ctrl-C to stop", flush=True)
    try:
        while True:
            ctrl.send_voltage(args.left, args.right)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.idle()


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

    # run
    p = sub.add_parser("run", help="Start full mission (default)")
    p.add_argument("--config", default=str(_config_module._CONFIG_PATH),
                   help="Path to config.yaml")
    p.add_argument("--no-display", action="store_true",
                   default=not bool(os.environ.get("DISPLAY", "")))
    p.add_argument("--no-telemetry", action="store_true")
    p.add_argument("--telemetry-every",  type=int,   default=1, metavar="N")
    p.add_argument("--telemetry-idle-s", type=float, default=0.5)

    # serve
    p = sub.add_parser("serve", help="Start the PID tuner web UI (http://<host>:5050)")
    p.add_argument("--config", default=str(_config_module._CONFIG_PATH),
                   help="Path to config.yaml (read/written by the UI)")
    p.add_argument("--host",     default="0.0.0.0")
    p.add_argument("--web-port", type=int, default=5050, metavar="PORT")

    # hardware debug
    sub.add_parser("encoders", help="Stream encoder ticks + RPM (Enter to stop)")
    sub.add_parser("serial",   help="Print raw serial bytes hex (Enter to stop)")

    p = sub.add_parser("motors", help="Set left/right motor fractions [-1, 1]")
    p.add_argument("left",  type=float)
    p.add_argument("right", type=float)

    p = sub.add_parser("voltage", help="Send direct voltage [-1, 1] to motors, bypassing RPM PID (Ctrl-C to stop)")
    p.add_argument("left",  type=float)
    p.add_argument("right", type=float)

    p = sub.add_parser("claw", help="Set claw servo angle (0–180°)")
    p.add_argument("angle", type=float)

    return parser


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    # Pre-scan for --config so set_path() runs before any config.get()
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

    ctrl = _make_controller(args)
    try:
        HARDWARE_COMMANDS[args.command](ctrl, args)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.shutdown()


if __name__ == "__main__":
    main()
