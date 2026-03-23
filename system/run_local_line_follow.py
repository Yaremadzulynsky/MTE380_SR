#!/usr/bin/env python3
"""
Main runner: perception → state machine → motor/servo control.

PID values are loaded from pid_config.json at startup and **reloaded from the same
file each time you press `g`** (so saves from the PID tuner apply on the next go).

CLI flags override the config file when provided explicitly.
Run:
    python run_local_line_follow.py [--dry-run]
    python run_local_line_follow.py --steer-kp 0.8   # override one value

Local OpenCV preview is **off** by default; use ``--show-window`` for a live window.
``--web-preview`` still writes JPEGs for the PID tuner (``GET /api/camera/preview.jpg``)
without opening a window.

While running, optional JSONL (lateral_err, avg_rpm) is written for the PID tuner
charts; truncated each time you press ``g``. See ``--mission-log`` / ``--no-mission-log``.

Default JPEG path: ``CAMERA_PREVIEW_PATH`` or ``system/.camera_preview.jpg``.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from typing import TextIO

import cv2
import numpy as np

from local.control import LocalMotorController, MotorCommand, MotorPIDConfig
from local.perception import FrameDetection, Perception
from local.state_machine import Config, ControlOutput, MissionStateMachine, State

# ── Config file ───────────────────────────────────────────────────────────────

_DEFAULT_CONFIG = Path(__file__).parent / "pid_config.json"
_DEFAULT_MISSION_LOG = Path(__file__).parent / "mission_telemetry.jsonl"
_DEFAULT_CAMERA_PREVIEW = Path(__file__).parent / ".camera_preview.jpg"

_FALLBACK: dict = {
    "steer_kp": 0.65,   "steer_ki": 0.04,    "steer_kd": 0.10,  "steer_out_limit": 0.80,
    "motor_kp": 0.003125, "motor_ki": 0.0005, "motor_kd": 0.0,
    "base_speed": 0.28, "min_speed": 0.16,   "max_speed": 0.45,
    "search_turn": 0.18, "search_turn_max": 0.32,
    "forward_ticks": 800, "forward_speed": 0.25,
    "turn_speed": 0.30,   "turn_duration_s": 2.2,
    "claw_open": 0.0,     "claw_closed": 90.0, "pickup_hold_s": 0.8,
    # Ground error: height/pitch/FOV are fixed in local/perception.py — tune scale only
    "geom_enable": True,
    "geom_lateral_norm_m": 0.10,
    "geom_camera_to_axle_m": 0.0,
    "red_loss_debounce_frames": 4,
    "red_error_ema_alpha": 0.35,
}


def load_pid_config(path: Path) -> dict:
    try:
        raw = json.loads(path.read_text())
        merged = {**_FALLBACK, **{k: v for k, v in raw.items() if k in _FALLBACK}}
        # Recover from an empty / zeroed JSON (e.g. bad PID tuner save)
        if merged.get("steer_kp", 0) == merged.get("steer_ki", 0) == merged.get("steer_kd", 0) == 0:
            print(
                "[config] steer PID all zero — restoring built-in steering gains from code.",
                flush=True,
            )
            for k in ("steer_kp", "steer_ki", "steer_kd", "steer_out_limit"):
                merged[k] = _FALLBACK[k]
        if merged.get("motor_kp", 0) == 0:
            print(
                "[config] motor_kp is zero — restoring built-in motor PID gains.",
                flush=True,
            )
            for k in ("motor_kp", "motor_ki", "motor_kd"):
                merged[k] = _FALLBACK[k]
        return merged
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"[config] Could not read {path}: {e} — using built-in defaults.", flush=True)
        return dict(_FALLBACK)


def apply_pid_config_to_args(cfg: dict, args: argparse.Namespace) -> None:
    """Copy merged pid_config dict onto argparse Namespace (field names differ for a few keys)."""
    for k, v in cfg.items():
        if k not in _FALLBACK:
            continue
        if k == "turn_duration_s":
            args.turn_duration = float(v)
        elif k == "pickup_hold_s":
            args.pickup_hold = float(v)
        elif k == "forward_ticks":
            setattr(args, "forward_ticks", int(v))
        elif k == "red_loss_debounce_frames":
            setattr(args, "red_loss_debounce_frames", int(v))
        else:
            setattr(args, k, v)


def _build_mission_preview_bgr(
    perception: Perception,
    frame: np.ndarray,
    det: FrameDetection,
    output: ControlOutput | None,
    rpm_l: float,
    rpm_r: float,
) -> np.ndarray:
    """Side-by-side: camera overlay (+ lateral error viz), red mask, Canny edges."""
    overlay = perception.draw_debug(frame, det)
    mh, mw = overlay.shape[:2]
    roi_y = int(mh * perception.roi_top_ratio)

    # Lateral error: err = (cx - w/2) / (w/2)  =>  cx = (w/2) * (1 + err)
    x_line = int(round(mw * 0.5 * (1.0 + float(det.red_error))))
    x_line = max(0, min(mw - 1, x_line))
    y_pt = min(mh - 18, max(roi_y + 20, mh // 2))
    if det.red_found:
        cv2.line(overlay, (x_line, 0), (x_line, mh - 1), (255, 128, 0), 2, cv2.LINE_AA)  # orange = line position
        cv2.circle(overlay, (x_line, y_pt), 7, (255, 128, 0), 2, cv2.LINE_AA)
        cv2.circle(overlay, (x_line, y_pt), 2, (255, 128, 0), -1, cv2.LINE_AA)
    else:
        cv2.line(overlay, (x_line, roi_y), (x_line, mh - 1), (80, 80, 80), 1, cv2.LINE_AA)
    cv2.putText(
        overlay,
        "white=center  orange=err",
        (mw - 220, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )

    label = (
        f"{output.state.value}  L={output.left:+.2f}({rpm_l:+.0f})  R={output.right:+.2f}({rpm_r:+.0f})"
        if output
        else "IDLE — press 'g'"
    )
    cv2.putText(
        overlay,
        label,
        (10, overlay.shape[0] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 0),
        2,
        cv2.LINE_AA,
    )

    mask_gray = perception.red_mask_full(frame)
    mask_bgr = cv2.cvtColor(mask_gray, cv2.COLOR_GRAY2BGR)
    if roi_y > 0:
        cv2.line(mask_bgr, (0, roi_y), (mw - 1, roi_y), (128, 128, 128), 1)
    cv2.putText(
        mask_bgr,
        "red mask",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 60, 140)
    edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    if roi_y > 0:
        cv2.line(edges_bgr, (0, roi_y), (mw - 1, roi_y), (128, 128, 128), 1)
    cv2.putText(
        edges_bgr,
        "edges (Canny)",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    return np.hstack([overlay, mask_bgr, edges_bgr])


def _write_camera_preview_jpeg(path: Path, bgr: np.ndarray) -> None:
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    if not ok:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(buf.tobytes())
    tmp.replace(path)


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
    p.add_argument("--fps",           type=float, default=60.0)
    p.add_argument(
        "--roi-top-ratio",
        type=float,
        default=0.0,
        help="Crop this fraction from the top of the frame before line detection (0 = full frame).",
    )
    p.add_argument(
        "--camera-rgb",
        action="store_true",
        help="Main stream is true RGB order (forces RGB→BGR before HSV). Default matches "
        "local/perception: BGR order buffer.",
    )
    p.add_argument(
        "--web-preview",
        action="store_true",
        help="Write camera+mask JPEG for the PID tuner (CAMERA_PREVIEW_PATH or "
        "--camera-preview-path). No local OpenCV window unless --show-window.",
    )
    p.add_argument(
        "--camera-preview-path",
        type=Path,
        default=None,
        metavar="PATH",
        help="Output JPEG for web UI (overrides CAMERA_PREVIEW_PATH env).",
    )
    p.add_argument("--serial-port",   default="/dev/ttyACM0")
    p.add_argument("--baud",          type=int,   default=115200)
    p.add_argument("--dry-run",       action="store_true")
    p.add_argument(
        "--show-window",
        action="store_true",
        help="Open a local OpenCV window with mission overlay (off by default).",
    )
    p.add_argument(
        "--no-display",
        action="store_true",
        help="Force no OpenCV window (redundant unless combined with --show-window).",
    )
    p.add_argument("--pid-config",    default=str(_DEFAULT_CONFIG),
                   help="Path to pid_config.json")
    p.add_argument(
        "--no-telemetry",
        action="store_true",
        help="Disable periodic terminal logs (vision + wheels + errors).",
    )
    p.add_argument(
        "--telemetry-every",
        type=int,
        default=1,
        metavar="N",
        help="While running, print telemetry every N camera frames (default: 1 = every frame).",
    )
    p.add_argument(
        "--telemetry-idle-s",
        type=float,
        default=0.5,
        help="When IDLE, print a short perception line at most this often (seconds).",
    )
    p.add_argument(
        "--mission-log",
        type=str,
        default=os.environ.get("MISSION_TELEMETRY_PATH", str(_DEFAULT_MISSION_LOG)),
        help="JSONL path (lateral_err, avg_rpm) for PID tuner charts; truncated each 'g'.",
    )
    p.add_argument(
        "--no-mission-log",
        action="store_true",
        help="Disable writing mission telemetry JSONL.",
    )

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
    p.add_argument(
        "--search-turn-max",
        type=float,
        default=cfg["search_turn_max"],
        help="Max wheel fraction for lost-line search (independent of --max-speed).",
    )

    p.add_argument("--forward-ticks",         type=int,   default=cfg["forward_ticks"])
    p.add_argument("--forward-speed",         type=float, default=cfg["forward_speed"])

    p.add_argument("--turn-speed",            type=float, default=cfg["turn_speed"])
    p.add_argument("--turn-duration",         type=float, default=cfg["turn_duration_s"])

    p.add_argument("--claw-open",             type=float, default=cfg["claw_open"])
    p.add_argument("--claw-closed",           type=float, default=cfg["claw_closed"])
    p.add_argument("--pickup-hold",           type=float, default=cfg["pickup_hold_s"])

    p.add_argument(
        "--geom-enable",
        action=argparse.BooleanOptionalAction,
        default=bool(cfg.get("geom_enable", True)),
        help="Use ground-plane lateral error (camera pose is fixed in perception.py).",
    )
    p.add_argument(
        "--geom-lateral-norm-m",
        type=float,
        default=cfg["geom_lateral_norm_m"],
        help="Metres of lateral offset → ~1.0 steering error (track-tune).",
    )
    p.add_argument(
        "--geom-camera-to-axle-m",
        type=float,
        default=float(cfg.get("geom_camera_to_axle_m", 0.0)),
        help="Ground-plane distance camera→axle; >0 extrapolates lateral error to the wheelbase.",
    )
    p.add_argument(
        "--red-loss-debounce-frames",
        type=int,
        default=int(cfg["red_loss_debounce_frames"]),
        help="Consecutive raw misses before red_found=False (holds last smoothed error).",
    )
    p.add_argument(
        "--red-error-ema-alpha",
        type=float,
        default=cfg["red_error_ema_alpha"],
        help="Low-pass on red_error when line visible (0=off, ~0.2–0.5 typical).",
    )
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
        """Return and clear the last keypress, or None if none pending."""
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

    camera_preview_path = args.camera_preview_path or Path(
        os.environ.get("CAMERA_PREVIEW_PATH", str(_DEFAULT_CAMERA_PREVIEW))
    )

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
        camera_channel_order="rgb" if args.camera_rgb else "bgr",
        geom_enable=args.geom_enable,
        geom_lateral_norm_m=args.geom_lateral_norm_m,
        geom_camera_to_axle_m=args.geom_camera_to_axle_m,
        red_loss_debounce_frames=args.red_loss_debounce_frames,
        red_error_ema_alpha=args.red_error_ema_alpha,
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

    def _make_sm() -> MissionStateMachine:
        return MissionStateMachine(Config(
            steer_kp=args.steer_kp,       steer_ki=args.steer_ki,
            steer_kd=args.steer_kd,       steer_out_limit=args.steer_out_limit,
            base_speed=args.base_speed,   min_speed=args.min_speed,
            max_speed=args.max_speed,
            search_turn=args.search_turn, search_turn_max=args.search_turn_max,
            forward_ticks=args.forward_ticks, forward_speed=args.forward_speed,
            turn_speed=args.turn_speed,   turn_duration_s=args.turn_duration,
            claw_open=args.claw_open,     claw_closed=args.claw_closed,
            pickup_hold_s=args.pickup_hold,
        ))

    sm      = _make_sm()
    running = False   # waiting for 'g' to start
    mission_log_fp: TextIO | None = None
    mission_t0: float | None = None

    control.send_claw(args.claw_open)

    def _stop_mission_log() -> None:
        nonlocal mission_log_fp
        if mission_log_fp is not None:
            mission_log_fp.close()
            mission_log_fp = None

    def go_reload_and_start() -> None:
        """Reload pid_config.json (dashboard), apply motor + state machine, then run."""
        nonlocal sm, running, mission_log_fp, mission_t0
        fresh = load_pid_config(cfg_path)
        apply_pid_config_to_args(fresh, args)
        control.set_motor_pid(
            MotorPIDConfig(kp=args.motor_kp, ki=args.motor_ki, kd=args.motor_kd)
        )
        perception.configure_geometry(
            geom_enable=args.geom_enable,
            geom_lateral_norm_m=args.geom_lateral_norm_m,
            geom_camera_to_axle_m=args.geom_camera_to_axle_m,
        )
        perception.configure_red_stability(
            red_loss_debounce_frames=args.red_loss_debounce_frames,
            red_error_ema_alpha=args.red_error_ema_alpha,
        )
        sm = _make_sm()
        running = True
        mission_t0 = time.monotonic()
        _stop_mission_log()
        if not args.no_mission_log:
            log_path = Path(args.mission_log)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            mission_log_fp = open(log_path, "w", buffering=1, encoding="utf-8")
        print(
            f"[KEY] go — reloaded {cfg_path}  "
            f"steer_kp={args.steer_kp}  max_speed={args.max_speed}  "
            f"motor_kp={args.motor_kp:.6f}  geom={args.geom_enable}",
            flush=True,
        )

    ready_extra = ""
    if args.web_preview:
        ready_extra = f"\n  web preview : {camera_preview_path.resolve()}  (PID tuner /api/camera/preview.jpg)"

    print(
        f"Ready — serial={args.serial_port}  dry_run={args.dry_run}\n"
        f"  config      : {cfg_path}\n"
        f"  steering PID: kp={args.steer_kp}  ki={args.steer_ki}  kd={args.steer_kd}\n"
        f"  motor    PID: kp={args.motor_kp:.6f}  ki={args.motor_ki}  kd={args.motor_kd}\n"
        f"  Press 'g' to go (reloads config from file), 's' to stop, 'q' to quit."
        f"{ready_extra}",
        flush=True,
    )

    kb = KeyboardListener()
    kb.start()

    telemetry = not args.no_telemetry
    frame_n = 0
    t_start = time.monotonic()
    last_idle_log = 0.0
    last_web_preview_t = 0.0
    _WEB_PREVIEW_MIN_INTERVAL_S = 0.12  # ~8 Hz; limits SD card / flash writes
    show_opencv = bool(args.show_window) and not args.no_display
    need_preview = show_opencv or args.web_preview

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
                mission_t0 = None
                _stop_mission_log()
                control.idle()
                print("[KEY] stop (motors idle, serial still open)", flush=True)
            elif key in ("q", "\x03"):   # q or Ctrl-C
                break

            # ── Vision (always, so display stays live) ────────────────────────
            frame = perception.read_frame()
            if frame is None:
                print("Camera read failed — stopping.", flush=True)
                break

            det = perception.detect(frame)

            # ── Control (only when running) ───────────────────────────────────
            if running:
                left, right = control.encoder_ticks
                output      = sm.step(det, left, right)

                control.send_drive(MotorCommand(left=output.left, right=output.right))
                if output.claw is not None:
                    control.send_claw(output.claw)

                if output.state == State.DONE:
                    running = False
                    mission_t0 = None
                    _stop_mission_log()
                    control.idle()
                    print("Mission complete. Press 'g' to run again.", flush=True)

                rpm_l, rpm_r = control.measured_rpm
                if (
                    mission_log_fp is not None
                    and mission_t0 is not None
                    and not args.no_mission_log
                ):
                    t_m = time.monotonic() - mission_t0
                    avg_rpm = (rpm_l + rpm_r) / 2.0
                    mission_log_fp.write(
                        json.dumps(
                            {
                                "t_s": round(t_m, 4),
                                "lateral_err": det.red_error,
                                "avg_rpm": avg_rpm,
                                "state": output.state.value,
                            },
                            separators=(",", ":"),
                        )
                        + "\n"
                    )
                status_line = (
                    f"state={output.state.value:14s}  "
                    f"err={det.red_error:+.2f}  "
                    f"blue={'Y' if det.blue_found else 'N'}  "
                    f"T={'Y' if det.t_junction else 'N'}  "
                    f"L={output.left:+.2f}({rpm_l:+.0f}rpm)  "
                    f"R={output.right:+.2f}({rpm_r:+.0f}rpm)"
                )
            else:
                output     = None
                rpm_l = rpm_r = 0.0
                status_line = "IDLE — press 'g' to start"

            # ── Display / web preview ─────────────────────────────────────────
            if not need_preview and not telemetry:
                print(status_line, flush=True)
            elif need_preview:
                preview_bgr = _build_mission_preview_bgr(
                    perception, frame, det, output, rpm_l, rpm_r
                )
                if args.web_preview:
                    now = time.monotonic()
                    if now - last_web_preview_t >= _WEB_PREVIEW_MIN_INTERVAL_S:
                        _write_camera_preview_jpeg(camera_preview_path, preview_bgr)
                        last_web_preview_t = now
                if show_opencv:
                    cv2.imshow("mission", preview_bgr)
                    k = cv2.waitKey(1) & 0xFF
                    if k == ord("q"):
                        break
                    elif k == ord("g") and not running:
                        go_reload_and_start()
                    elif k == ord("s") and running:
                        running = False
                        mission_t0 = None
                        _stop_mission_log()
                        control.idle()
                        print("[KEY] stop (motors idle, serial still open)", flush=True)

            elapsed = time.time() - t0
            iter_ms = elapsed * 1000.0

            # ── Terminal telemetry (after work + GUI; includes camera + draw time) ──
            if telemetry:
                t_run = time.monotonic() - t_start
                if output is not None and (frame_n % max(1, args.telemetry_every) == 0):
                    enc_l, enc_r = control.encoder_ticks
                    mt = control.motor_telemetry_line()
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
        _stop_mission_log()
        kb.stop()
        control.shutdown()
        perception.release()
        cv2.destroyAllWindows()
        print("Stopped.", flush=True)


if __name__ == "__main__":
    main()
