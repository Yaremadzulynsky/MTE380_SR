"""
MTE 380 system CLI.

Start the full mission (perception + state machine + motors):
  python -m cli.main                                      # default port, interactive shell
  python -m cli.main --serial-port /dev/ttyACM0 run
  python -m cli.main --serial-port COM3 --dry-run run
  python -m cli.main run --no-display --pid-config my.json

Low-level hardware debug commands (talk directly to Arduino):
  python -m cli.main --serial-port /dev/ttyACM0 encoders
  python -m cli.main --serial-port /dev/ttyACM0 serial
  python -m cli.main --serial-port /dev/ttyACM0 motors 0.3 0.3
  python -m cli.main --serial-port /dev/ttyACM0 claw 90
"""
from __future__ import annotations

import argparse
import cmd
import json
import logging
import os
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from control import LocalMotorController, MotorCommand, MotorPIDConfig
from state_machine import Config, MissionStateMachine, State


# ── Config loading (mirrors run_local_line_follow.py) ─────────────────────────

_DEFAULT_CONFIG = Path(__file__).parent.parent / "pid_config.json"

_FALLBACK: dict = {
    "steer_kp": 0.65,     "steer_ki": 0.04,    "steer_kd": 0.10,  "steer_out_limit": 0.80,
    "motor_kp": 0.003125, "motor_ki": 0.0005,  "motor_kd": 0.0,
    "base_speed": 0.28,   "min_speed": 0.16,   "max_speed": 0.45,
    "search_turn": 0.18,  "search_turn_max": 0.32,
    "lost_frames_before_search": 5,
    "forward_ticks": 800, "forward_speed": 0.25,
    "turn_speed": 0.30,   "turn_duration_s": 2.2,
    "claw_open": 0.0,     "claw_closed": 90.0, "pickup_hold_s": 0.8,
    "geom_enable": True,
    "geom_lateral_norm_m": 0.10,
    "red_loss_debounce_frames": 4,
    "red_error_ema_alpha": 0.35,
}


def load_pid_config(path: Path) -> dict:
    try:
        raw = json.loads(path.read_text())
        merged = {**_FALLBACK, **{k: v for k, v in raw.items() if k in _FALLBACK}}
        if merged.get("steer_kp", 0) == merged.get("steer_ki", 0) == merged.get("steer_kd", 0) == 0:
            print("[config] steer PID all zero — restoring built-in defaults.", flush=True)
            for k in ("steer_kp", "steer_ki", "steer_kd", "steer_out_limit"):
                merged[k] = _FALLBACK[k]
        if merged.get("motor_kp", 0) == 0:
            print("[config] motor_kp is zero — restoring built-in defaults.", flush=True)
            for k in ("motor_kp", "motor_ki", "motor_kd"):
                merged[k] = _FALLBACK[k]
        return merged
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"[config] Could not read {path}: {e} — using built-in defaults.", flush=True)
        return dict(_FALLBACK)


def _apply_config_to_args(cfg: dict, args: argparse.Namespace) -> None:
    """Merge pid_config dict onto argparse Namespace (handles a few key renames)."""
    for k, v in cfg.items():
        if k not in _FALLBACK:
            continue
        if k == "turn_duration_s":
            args.turn_duration = float(v)
        elif k == "pickup_hold_s":
            args.pickup_hold = float(v)
        elif k == "forward_ticks":
            args.forward_ticks = int(v)
        elif k == "red_loss_debounce_frames":
            args.red_loss_debounce_frames = int(v)
        else:
            setattr(args, k, v)


# ── Mission runner (background thread) ────────────────────────────────────────

class MissionRunner:
    """Runs the perception → state machine → motor control loop in a background thread."""

    def __init__(self, control: LocalMotorController, cfg_path: Path, args: argparse.Namespace):
        from perception import Perception
        self._control  = control
        self._cfg_path = cfg_path
        self._args     = args

        self._perception = Perception(
            width=args.width,
            height=args.height,
            roi_top_ratio=args.roi_top_ratio,
            geom_enable=args.geom_enable,
            geom_lateral_norm_m=args.geom_lateral_norm_m,
            red_loss_debounce_frames=args.red_loss_debounce_frames,
            red_error_ema_alpha=args.red_error_ema_alpha,
        )

        self._sm: MissionStateMachine | None = None
        self._running = False
        self._quit    = threading.Event()
        self._lock    = threading.Lock()

        # Telemetry accessible from the shell
        self._last_det    = None
        self._last_output = None
        self._frame_n     = 0
        self._t_start     = time.monotonic()

        self._period = 1.0 / max(getattr(args, "fps", 30.0), 1.0)
        self._thread = threading.Thread(target=self._loop, daemon=True, name="mission-loop")

    # ── Public interface ───────────────────────────────────────────────────────

    def start_loop(self) -> None:
        self._thread.start()

    def go(self) -> None:
        """Reload pid_config.json and start the mission."""
        fresh = load_pid_config(self._cfg_path)
        _apply_config_to_args(fresh, self._args)
        args = self._args

        self._control.set_motor_pid(
            MotorPIDConfig(kp=args.motor_kp, ki=args.motor_ki, kd=args.motor_kd)
        )
        self._perception.configure_geometry(
            geom_enable=args.geom_enable,
            geom_lateral_norm_m=args.geom_lateral_norm_m,
        )
        self._perception.configure_red_stability(
            red_loss_debounce_frames=args.red_loss_debounce_frames,
            red_error_ema_alpha=args.red_error_ema_alpha,
        )

        with self._lock:
            self._sm      = self._make_sm()
            self._running = True

        print(
            f"[go] mission started — "
            f"steer_kp={args.steer_kp}  max_speed={args.max_speed}  "
            f"motor_kp={args.motor_kp:.6f}",
            flush=True,
        )

    def stop(self) -> None:
        """Pause the mission (idle motors, serial stays open)."""
        with self._lock:
            self._running = False
        self._control.idle()
        print("[stop] motors idle, serial open", flush=True)

    def shutdown(self) -> None:
        """Terminate the loop thread, release camera and serial."""
        self._quit.set()
        with self._lock:
            self._running = False
        self._control.idle()
        self._thread.join(timeout=2.0)
        self._perception.release()
        self._control.shutdown()

    def set_steer_gains(self, kp: float, ki: float, kd: float) -> None:
        """Update steering PID on the live state machine."""
        with self._lock:
            if self._sm is not None:
                self._sm.cfg.steer_kp = kp
                self._sm.cfg.steer_ki = ki
                self._sm.cfg.steer_kd = kd
                self._sm._steer_pid.tunings = (kp, ki, kd)
                self._sm._steer_pid.reset()
        self._args.steer_kp = kp
        self._args.steer_ki = ki
        self._args.steer_kd = kd

    def set_motor_gains(self, kp: float, ki: float, kd: float) -> None:
        self._args.motor_kp = kp
        self._args.motor_ki = ki
        self._args.motor_kd = kd
        self._control.set_motor_pid(MotorPIDConfig(kp=kp, ki=ki, kd=kd))

    @property
    def state(self) -> str:
        with self._lock:
            return self._sm.state.value if self._sm else "not started"

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def snapshot(self) -> dict:
        """Thread-safe telemetry snapshot for the shell's status command."""
        with self._lock:
            det    = self._last_det
            output = self._last_output
            sm_state = self._sm.state.value if self._sm else "N/A"
            running  = self._running
        enc_l, enc_r = self._control.encoder_ticks
        rpm_l, rpm_r = self._control.measured_rpm
        return {
            "sm_state":    sm_state,
            "running":     running,
            "enc_l":       enc_l,
            "enc_r":       enc_r,
            "rpm_l":       rpm_l,
            "rpm_r":       rpm_r,
            "det":         det,
            "output":      output,
            "motor_telem": self._control.motor_telemetry_line(),
        }

    # ── Internal ──────────────────────────────────────────────────────────────

    def _make_sm(self) -> MissionStateMachine:
        a = self._args
        return MissionStateMachine(Config(
            steer_kp=a.steer_kp,         steer_ki=a.steer_ki,
            steer_kd=a.steer_kd,         steer_out_limit=a.steer_out_limit,
            base_speed=a.base_speed,     min_speed=a.min_speed,
            max_speed=a.max_speed,
            search_turn=a.search_turn,   search_turn_max=a.search_turn_max,
            forward_ticks=a.forward_ticks, forward_speed=a.forward_speed,
            turn_speed=a.turn_speed,     turn_duration_s=a.turn_duration,
            claw_open=a.claw_open,       claw_closed=a.claw_closed,
            pickup_hold_s=a.pickup_hold,
        ))

    def _loop(self) -> None:
        import cv2
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

            if not no_display:
                overlay = self._perception.draw_debug(frame, det)
                if output:
                    label = (
                        f"{output.state.value}  "
                        f"L={output.left:+.2f}  R={output.right:+.2f}"
                    )
                    cv2.putText(
                        overlay, label,
                        (10, overlay.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2, cv2.LINE_AA,
                    )
                cv2.imshow("mission", overlay)
                cv2.waitKey(1)

            if telemetry:
                t_run = time.monotonic() - self._t_start
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
    """Interactive shell — calls Python APIs directly, no HTTP."""

    intro  = "MTE 380 — type 'help' for commands, 'quit' to exit."
    prompt = "(mission) "

    def __init__(
        self,
        runner:  MissionRunner,
        control: LocalMotorController,
        args:    argparse.Namespace,
    ):
        super().__init__()
        self._runner  = runner
        self._control = control
        self._args    = args

    # ── Mission control ────────────────────────────────────────────────────────

    def do_start(self, _: str):
        """start  — reload pid_config.json and begin the mission."""
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
            print(
                f"  vision:   red={'Y' if det.red_found else 'N'}  "
                f"err={det.red_error:+.3f}  "
                f"blue={'Y' if det.blue_found else 'N'}  "
                f"T={'Y' if det.t_junction else 'N'}"
            )
        out = snap["output"]
        if out is not None:
            print(f"  drive:    L={out.left:+.3f}  R={out.right:+.3f}")
        print(f"  motors:   {snap['motor_telem']}")

    # ── PID tuning ─────────────────────────────────────────────────────────────

    def do_gains(self, line: str):
        """gains <steer|motor> <kp> <ki> <kd>  — update PID gains live."""
        parts = line.split()
        try:
            target      = parts[0]
            kp, ki, kd  = float(parts[1]), float(parts[2]), float(parts[3])
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
        print(f"  {target} gains → kp={kp}  ki={ki}  kd={kd}")

    def complete_gains(self, text: str, *_):
        return [t for t in ("steer", "motor") if t.startswith(text)]

    # ── Actuators ──────────────────────────────────────────────────────────────

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
        parts = line.split()
        try:
            l, r = float(parts[0]), float(parts[1])
        except (IndexError, ValueError):
            print("  usage: motors <left> <right>")
            return
        self._control.send_drive(MotorCommand(left=l, right=r))
        print(f"  motors → L={l}  R={r}")

    # ── Config ─────────────────────────────────────────────────────────────────

    def do_config(self, _: str):
        """config  — show current PID and mission config values."""
        a = self._args
        print("  Steering PID:")
        print(f"    kp={a.steer_kp}  ki={a.steer_ki}  kd={a.steer_kd}  "
              f"out_limit={a.steer_out_limit}")
        print("  Motor PID:")
        print(f"    kp={a.motor_kp:.6f}  ki={a.motor_ki}  kd={a.motor_kd}")
        print("  Speed:")
        print(f"    base={a.base_speed}  min={a.min_speed}  max={a.max_speed}")
        print(f"    search_turn={a.search_turn}  search_turn_max={a.search_turn_max}")
        print("  Mission:")
        print(f"    forward_ticks={a.forward_ticks}  forward_speed={a.forward_speed}")
        print(f"    turn_speed={a.turn_speed}  turn_duration={a.turn_duration}")
        print("  Claw:")
        print(f"    open={a.claw_open}  closed={a.claw_closed}  pickup_hold={a.pickup_hold}")
        print("  Vision:")
        print(f"    geom_enable={a.geom_enable}  "
              f"geom_lateral_norm_m={a.geom_lateral_norm_m}")
        print(f"    red_loss_debounce_frames={a.red_loss_debounce_frames}  "
              f"red_error_ema_alpha={a.red_error_ema_alpha}")

    def do_reload(self, _: str):
        """reload  — reload pid_config.json without starting the mission."""
        fresh = load_pid_config(self._runner._cfg_path)
        _apply_config_to_args(fresh, self._args)
        self._control.set_motor_pid(
            MotorPIDConfig(
                kp=self._args.motor_kp,
                ki=self._args.motor_ki,
                kd=self._args.motor_kd,
            )
        )
        self._runner._perception.configure_geometry(
            geom_enable=self._args.geom_enable,
            geom_lateral_norm_m=self._args.geom_lateral_norm_m,
        )
        self._runner._perception.configure_red_stability(
            red_loss_debounce_frames=self._args.red_loss_debounce_frames,
            red_error_ema_alpha=self._args.red_error_ema_alpha,
        )
        print(f"  Reloaded {self._runner._cfg_path} (mission not started)")

    # ── Misc ───────────────────────────────────────────────────────────────────

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
    """Start full mission: perception + state machine + motors. Interactive shell."""
    log = logging.getLogger("cli")

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

    cfg_path = Path(args.pid_config)
    runner   = MissionRunner(control, cfg_path, args)
    runner.start_loop()
    log.info("Mission loop started (camera live). Type 'start' to begin.")

    control.send_claw(args.claw_open)

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


# ── Hardware debug commands ────────────────────────────────────────────────────

def _make_controller(args: argparse.Namespace) -> LocalMotorController:
    return LocalMotorController(
        serial_port=args.serial_port,
        baud=args.baud,
        dry_run=args.dry_run,
    )


def cmd_encoders(ctrl: LocalMotorController, _args: argparse.Namespace) -> None:
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


def cmd_serial(ctrl: LocalMotorController, _args: argparse.Namespace) -> None:
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


def cmd_motors(ctrl: LocalMotorController, args: argparse.Namespace) -> None:
    ctrl.send_drive(MotorCommand(left=args.left, right=args.right))
    try:
        input(f"  Motors L={args.left}  R={args.right} — press Enter or Ctrl-C to stop\n")
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.idle()


def cmd_claw(ctrl: LocalMotorController, args: argparse.Namespace) -> None:
    ctrl.send_claw(args.angle)
    print(f"  Claw → {args.angle}°")


HARDWARE_COMMANDS = {
    "encoders": cmd_encoders,
    "serial":   cmd_serial,
    "motors":   cmd_motors,
    "claw":     cmd_claw,
}


# ── Argument parser ────────────────────────────────────────────────────────────

def build_parser(cfg: dict) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="MTE 380 system CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--serial-port", default="/dev/ttyACM0",
        help="Arduino serial port (default: /dev/ttyACM0)",
    )
    parser.add_argument("--baud",    type=int, default=115200)
    parser.add_argument("--dry-run", action="store_true",
                        help="Skip serial; print motor commands to stdout")
    parser.add_argument("--debug",   action="store_true")

    sub = parser.add_subparsers(dest="command", metavar="command")

    # ── Run (default) ─────────────────────────────────────────────────────────
    p = sub.add_parser("run", help="Start full mission (default when no command given)")
    p.add_argument("--pid-config", default=str(_DEFAULT_CONFIG),
                   help="Path to pid_config.json")
    p.add_argument("--width",  type=int,   default=640)
    p.add_argument("--height", type=int,   default=480)
    p.add_argument("--fps",    type=float, default=30.0)
    p.add_argument("--roi-top-ratio", type=float, default=0.0,
                   help="Fraction of frame top to ignore (0 = full frame)")
    p.add_argument("--no-display", action="store_true",
                   default=not bool(os.environ.get("DISPLAY", "")),
                   help="Disable OpenCV debug window")
    p.add_argument("--no-telemetry", action="store_true",
                   help="Disable per-frame terminal logs")
    p.add_argument("--telemetry-every", type=int, default=1, metavar="N",
                   help="Print telemetry every N frames")
    p.add_argument("--telemetry-idle-s", type=float, default=0.5,
                   help="Max interval (s) for idle telemetry lines")

    # PID tunables — defaults come from the config file
    p.add_argument("--steer-kp",        type=float, default=cfg["steer_kp"])
    p.add_argument("--steer-ki",        type=float, default=cfg["steer_ki"])
    p.add_argument("--steer-kd",        type=float, default=cfg["steer_kd"])
    p.add_argument("--steer-out-limit", type=float, default=cfg["steer_out_limit"])
    p.add_argument("--motor-kp",        type=float, default=cfg["motor_kp"])
    p.add_argument("--motor-ki",        type=float, default=cfg["motor_ki"])
    p.add_argument("--motor-kd",        type=float, default=cfg["motor_kd"])
    p.add_argument("--base-speed",      type=float, default=cfg["base_speed"])
    p.add_argument("--min-speed",       type=float, default=cfg["min_speed"])
    p.add_argument("--max-speed",       type=float, default=cfg["max_speed"])
    p.add_argument("--search-turn",     type=float, default=cfg["search_turn"])
    p.add_argument("--search-turn-max", type=float, default=cfg["search_turn_max"])
    p.add_argument("--forward-ticks",   type=int,   default=cfg["forward_ticks"])
    p.add_argument("--forward-speed",   type=float, default=cfg["forward_speed"])
    p.add_argument("--turn-speed",      type=float, default=cfg["turn_speed"])
    p.add_argument("--turn-duration",   type=float, default=cfg["turn_duration_s"])
    p.add_argument("--claw-open",       type=float, default=cfg["claw_open"])
    p.add_argument("--claw-closed",     type=float, default=cfg["claw_closed"])
    p.add_argument("--pickup-hold",     type=float, default=cfg["pickup_hold_s"])
    p.add_argument(
        "--geom-enable",
        action=argparse.BooleanOptionalAction,
        default=bool(cfg.get("geom_enable", True)),
        help="Use ground-plane lateral error (camera pose fixed in perception.py)",
    )
    p.add_argument("--geom-lateral-norm-m", type=float, default=cfg["geom_lateral_norm_m"],
                   help="Lateral offset (m) that maps to ±1 steering error")
    p.add_argument("--red-loss-debounce-frames", type=int,
                   default=int(cfg["red_loss_debounce_frames"]),
                   help="Consecutive raw misses before red_found=False")
    p.add_argument("--red-error-ema-alpha", type=float, default=cfg["red_error_ema_alpha"],
                   help="Low-pass alpha on red_error (0=off, ~0.2–0.5 typical)")

    # ── Hardware debug commands ───────────────────────────────────────────────
    sub.add_parser("encoders", help="Stream encoder ticks + RPM live (Enter to stop)")
    sub.add_parser("serial",   help="Print raw serial bytes hex (Enter to stop)")

    p = sub.add_parser("motors", help="Set left/right motor fractions [-1, 1]")
    p.add_argument("left",  type=float)
    p.add_argument("right", type=float)

    p = sub.add_parser("claw", help="Set claw servo angle (0–180°)")
    p.add_argument("angle", type=float)

    return parser


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    # Pre-scan for --pid-config before full parse (mirrors run_local_line_follow.py)
    cfg_path = _DEFAULT_CONFIG
    for i, arg in enumerate(sys.argv[1:]):
        if arg.startswith("--pid-config="):
            cfg_path = Path(arg.split("=", 1)[1])
        elif arg == "--pid-config" and i + 1 < len(sys.argv) - 1:
            cfg_path = Path(sys.argv[i + 2])

    cfg    = load_pid_config(cfg_path)
    parser = build_parser(cfg)
    args   = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    if args.command is None or args.command == "run":
        # When invoked without a subcommand, fill in run-subparser defaults
        if args.command is None:
            if not hasattr(args, "pid_config"):
                args.pid_config = str(cfg_path)
            if not hasattr(args, "width"):
                args.width = 640
            if not hasattr(args, "height"):
                args.height = 480
            if not hasattr(args, "fps"):
                args.fps = 30.0
            if not hasattr(args, "roi_top_ratio"):
                args.roi_top_ratio = 0.0
            if not hasattr(args, "no_display"):
                args.no_display = not bool(os.environ.get("DISPLAY", ""))
            if not hasattr(args, "no_telemetry"):
                args.no_telemetry = False
            if not hasattr(args, "telemetry_every"):
                args.telemetry_every = 1
            if not hasattr(args, "telemetry_idle_s"):
                args.telemetry_idle_s = 0.5
            # PID tunables from config
            _apply_config_to_args(cfg, args)
            # Ensure all PID attrs exist with correct names
            for k, fallback in _FALLBACK.items():
                attr = k
                if k == "turn_duration_s":
                    attr = "turn_duration"
                elif k == "pickup_hold_s":
                    attr = "pickup_hold"
                if not hasattr(args, attr):
                    setattr(args, attr, fallback)

        cmd_run(args)
        return

    # Hardware debug commands — connect to Arduino directly, no camera
    ctrl = _make_controller(args)
    try:
        HARDWARE_COMMANDS[args.command](ctrl, args)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.shutdown()


if __name__ == "__main__":
    main()
