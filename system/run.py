#!/usr/bin/env python3
"""
MTE380 Robot — single-file runner (no Docker).

Web UI and JSON API are served by FastAPI (uvicorn).

Tuning (PID, smoothing, servo, SM params) is saved to data/ui_tuning.json on each
Apply, and reloaded on startup. Override path with RUN_UI_TUNING_PATH; set
RUN_UI_TUNING_DISABLE=1 to skip load/save.

Usage:
    pip install -r requirements-run.txt
    python run.py [--source rpicam|webcam] [--config PATH] [--port PORT]
                  [--serial PORT] [--fps N] [--no-robot]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

# ── sys.path — expose all sub-packages ────────────────────────────────────────
_HERE = Path(__file__).parent.resolve()
sys.path.insert(0, str(_HERE / "perception"))
sys.path.insert(0, str(_HERE / "state_machine"))
sys.path.insert(0, str(_HERE / "control_communication"))

# ── Third-party ───────────────────────────────────────────────────────────────
import cv2
import numpy as np
import uvicorn
from fastapi import Body, FastAPI
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

# ── Perception ────────────────────────────────────────────────────────────────
from src.config import AppConfig, load_config
from src.pipeline import PipelineOutput, PipelineState, run_pipeline
from src.vision.camera import OpenCVCamera, RpicamVidCamera, _read_one_jpeg_from_stream
from src.vision.masks import crop_roi
from src.utils.math2d import to_robot_frame_clamped

# ── State machine (import module before monkey-patch) ─────────────────────────
import sm_state_machine                                     # noqa: E402
from sm_input_buffer import InputBuffer
from sm_models import Inputs, State, Vector
from sm_state_machine import LINE_FOLLOW_PID_BOUNDS, StateMachine

# ── Robot hardware ────────────────────────────────────────────────────────────
try:
    from robot_control_system.robot import Robot as _RobotCls
except Exception as _e:
    print(f"[run] Robot module not available: {_e}", file=sys.stderr)
    _RobotCls = None          # type: ignore[assignment,misc]


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="MTE380 single-file runner (no Docker)")
    p.add_argument("--source", default="rpicam",
                   help="Camera: rpicam | webcam | http://... | video:/path")
    p.add_argument("--config",
                   default=str(_HERE / "perception/configs/default.yaml"),
                   help="Perception YAML config path")
    p.add_argument("--port", type=int, default=5000, help="Web UI HTTP port")
    p.add_argument("--serial", default="/dev/ttyACM0", help="Arduino serial device")
    p.add_argument("--fps", type=float, default=30.0, help="Camera FPS")
    p.add_argument("--no-robot", action="store_true",
                   help="Skip robot init (software-only / test mode)")
    p.add_argument("--h264-udp", default=None, metavar="HOST:PORT",
                   help="Also stream annotated overlay as H.264 MPEG-TS over UDP. "
                        "View on laptop: ffplay -fflags nobuffer -flags low_delay "
                        "-framedrop 'udp://0.0.0.0:PORT?localaddr=<laptop-tailscale-ip>'")
    p.add_argument("--h264-encoder", default="h264_v4l2m2m",
                   help="ffmpeg video encoder: h264_v4l2m2m (Pi HW) or libx264 (CPU)")
    p.add_argument("--h264-bitrate", default="2000k",
                   help="Bitrate for H.264 encoder (e.g. 2000k)")
    return p.parse_args()


# ─────────────────────────────────────────────────────────────────────────────
# H.264 UDP writer — pipes BGR frames into ffmpeg → MPEG-TS → UDP
# ─────────────────────────────────────────────────────────────────────────────

import shutil
import subprocess as _subprocess

class _H264UdpWriter:
    """
    Sends annotated BGR frames as H.264 MPEG-TS over UDP using ffmpeg.

    On Pi, use --h264-encoder h264_v4l2m2m to offload encoding to hardware.
    View on laptop (replace IP/port):
        ffplay -fflags nobuffer -flags low_delay -framedrop \\
               'udp://0.0.0.0:5004?localaddr=<laptop-tailscale-ip>'
    """

    def __init__(self, dest: str, width: int, height: int, fps: float,
                 encoder: str = "h264_v4l2m2m", bitrate: str = "2000k") -> None:
        if not shutil.which("ffmpeg"):
            raise RuntimeError("ffmpeg not found — install with: apt install ffmpeg")
        host, _, port = dest.partition(":")
        self._dest = f"udp://{host}:{port}?pkt_size=1316"
        w = width + (width % 2)   # encoder needs even dimensions
        h = height + (height % 2)
        self._w, self._h = w, h
        cmd = [
            "ffmpeg", "-loglevel", "warning", "-y",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{w}x{h}", "-r", f"{fps:.3f}", "-i", "-", "-an",
        ]
        if encoder == "h264_v4l2m2m":
            cmd += ["-c:v", "h264_v4l2m2m", "-b:v", bitrate, "-pix_fmt", "yuv420p"]
        else:
            cmd += ["-c:v", encoder, "-preset", "ultrafast",
                    "-tune", "zerolatency", "-pix_fmt", "yuv420p",
                    "-g", str(max(2, int(fps * 2))), "-bf", "0"]
        cmd += ["-f", "mpegts", self._dest]
        self._proc = _subprocess.Popen(
            cmd, stdin=_subprocess.PIPE,
            stdout=_subprocess.DEVNULL, stderr=_subprocess.DEVNULL,
        )
        self._q: queue.Queue = queue.Queue(maxsize=2)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        assert self._proc.stdin is not None
        while not self._stop.is_set():
            try:
                data = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._proc.stdin.write(data)
            except (BrokenPipeError, ValueError):
                break

    def write(self, frame_bgr: np.ndarray) -> None:
        # Pad to even dimensions if needed
        h, w = frame_bgr.shape[:2]
        if w != self._w or h != self._h:
            frame_bgr = frame_bgr[:self._h, :self._w]
        data = np.ascontiguousarray(frame_bgr).tobytes()
        try:
            self._q.put_nowait(data)
        except queue.Full:
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait(data)
            except queue.Full:
                pass

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        if self._proc.stdin:
            try:
                self._proc.stdin.close()
            except (BrokenPipeError, ValueError):
                pass
        try:
            self._proc.wait(timeout=3.0)
        except _subprocess.TimeoutExpired:
            self._proc.kill()


# ─────────────────────────────────────────────────────────────────────────────
# Shared cross-thread state
# ─────────────────────────────────────────────────────────────────────────────

class SharedState:
    """Thread-safe telemetry + frame buffer for the web layer."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.zone: str = "UNKNOWN"
        self.gamma: float = 0.0
        self.line_error_x: float = 0.0
        self.line_error_raw: float = 0.0
        self.px: float = 0.0
        self.py: float = 0.0
        self.path_detected: bool = False
        self.target_detected: bool = False
        self.target_px: float = 0.0
        self.target_py: float = 0.0
        self.path_mask_key: str = "red"
        self.frame_count: int = 0
        self.sm_state: str = "REMOTE_CONTROL"
        self.fps: float = 0.0
        self.robot_active: bool = False
        self._jpeg: bytes | None = None
        self.frame_event = threading.Event()

    def update_perception(self, out: PipelineOutput, fps: float) -> None:
        with self._lock:
            self.zone = out.zone
            self.gamma = out.gamma
            self.line_error_x = out.line_error_x
            self.line_error_raw = float(out.debug_artifacts.get("smoothing_debug", {}).get("line_error_raw", out.line_error_x))
            self.px = out.px
            self.py = out.py
            self.path_detected = out.path_detected
            self.target_detected = out.target_detected
            self.target_px = out.target_px
            self.target_py = out.target_py
            self.path_mask_key = out.path_mask_key
            self.frame_count += 1
            self.fps = fps

    def set_raw_frame(self, jpeg: bytes) -> None:
        with self._lock:
            if self._jpeg is None:  # only use raw as fallback before first overlay
                self._jpeg = jpeg
                self.frame_event.set()

    def set_frame(self, jpeg: bytes) -> None:
        with self._lock:
            self._jpeg = jpeg
        self.frame_event.set()

    def get_frame(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def set_sm_state(self, name: str) -> None:
        with self._lock:
            self.sm_state = name

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "zone": self.zone,
                "gamma": round(self.gamma, 4),
                "line_error_x": round(self.line_error_x, 5),
                "line_error_raw": round(self.line_error_raw, 5),
                "px": round(self.px, 4),
                "py": round(self.py, 4),
                "path_detected": self.path_detected,
                "target_detected": self.target_detected,
                "target_px": round(self.target_px, 4),
                "target_py": round(self.target_py, 4),
                "path_mask_key": self.path_mask_key,
                "frame_count": self.frame_count,
                "sm_state": self.sm_state,
                "fps": round(self.fps, 1),
                "robot_active": self.robot_active,
            }


shared = SharedState()

# Web UI tuning persistence (line PID, smoothing, servo, SM params)
_DEFAULT_UI_TUNING_PATH = _HERE / "data" / "ui_tuning.json"
_UI_TUNING_SAVE_LOCK = threading.Lock()
_SMOOTHING_KEYS = (
    ("max_error_step", "max_error_step", 0.0, 1.0),
    ("ema_alpha_error", "ema_alpha_error", 0.0, 0.999),
    ("ema_alpha_path", "ema_alpha_path", 0.0, 0.999),
    ("window_size", "window_size", 1, 15),
    ("freeze_on_miss_frames", "freeze_on_miss_frames", 0, 30),
)
_SM_PARAMS_KEYS = (
    ("find_line_speed", "find_line_speed", 0.0, 1.0),
    ("line_lost_timeout_s", "line_lost_timeout_s", 0.05, 30.0),
    ("scan_switch_interval_s", "scan_switch_interval_s", 0.05, 10.0),
    ("retrieved_turn_speed", "retrieved_turn_speed", 0.0, 1.0),
    ("retrieved_turn_tol_rad", "retrieved_turn_tol_rad", 0.01, math.pi),
    ("tick_interval", "tick_interval", 0.001, 1.0),
)


def _smoothing_dict_from_cfg(cfg: AppConfig) -> dict[str, Any]:
    s = cfg.smoothing
    return {
        "max_error_step": s.max_error_step,
        "ema_alpha_error": s.ema_alpha_error,
        "ema_alpha_path": s.ema_alpha_path,
        "window_size": s.window_size,
        "freeze_on_miss_frames": s.freeze_on_miss_frames,
    }


def _apply_smoothing_payload(cfg: AppConfig, payload: dict[str, Any]) -> None:
    s = cfg.smoothing
    for json_key, attr, lo, hi in _SMOOTHING_KEYS:
        if json_key not in payload:
            continue
        setattr(
            s,
            attr,
            type(getattr(s, attr))(max(lo, min(hi, float(payload[json_key])))),
        )


def _sm_params_dict(state_machine: StateMachine) -> dict[str, float]:
    return {
        "find_line_speed": state_machine.find_line_speed,
        "line_lost_timeout_s": state_machine.line_lost_timeout_s,
        "scan_switch_interval_s": state_machine.scan_switch_interval_s,
        "retrieved_turn_speed": state_machine.retrieved_turn_speed,
        "retrieved_turn_tol_rad": state_machine.retrieved_turn_tol_rad,
        "tick_interval": state_machine.tick_interval,
    }


def _apply_sm_params_payload(state_machine: StateMachine, payload: dict[str, Any]) -> None:
    for json_key, attr, lo, hi in _SM_PARAMS_KEYS:
        if json_key not in payload:
            continue
        val = max(lo, min(hi, float(payload[json_key])))
        setattr(state_machine, attr, val)


def _ui_tuning_snapshot(state_machine: StateMachine, cfg: AppConfig) -> dict[str, Any]:
    return {
        "version": 1,
        "saved_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "pid": state_machine.get_line_follow_pid_settings(),
        "smoothing": _smoothing_dict_from_cfg(cfg),
        "servo": {
            "min": int(sm_state_machine.SERVO_MIN_DEG),
            "max": int(sm_state_machine.SERVO_MAX_DEG),
            "center": int(sm_state_machine.SERVO_CENTER_DEG),
        },
        "sm_params": _sm_params_dict(state_machine),
    }


def _save_ui_tuning(path: Path, state_machine: StateMachine, cfg: AppConfig) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(_ui_tuning_snapshot(state_machine, cfg), f, indent=2)
        os.replace(tmp, path)
    except OSError as exc:
        print(f"[run] ui tuning save failed: {exc}", file=sys.stderr)
        try:
            tmp.unlink()
        except OSError:
            pass


def _load_ui_tuning(path: Path, state_machine: StateMachine, cfg: AppConfig) -> bool:
    if not path.is_file():
        return False
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[run] ui tuning load failed: {exc}", file=sys.stderr)
        return False
    if not isinstance(data, dict):
        return False
    if "pid" in data and isinstance(data["pid"], dict):
        _, errs = state_machine.apply_line_follow_pid_updates(data["pid"])
        if errs:
            print(f"[run] ui tuning: skipped invalid pid keys: {errs}", file=sys.stderr)
    if "smoothing" in data and isinstance(data["smoothing"], dict):
        _apply_smoothing_payload(cfg, data["smoothing"])
    if "servo" in data and isinstance(data["servo"], dict):
        sv = data["servo"]
        if "min" in sv:
            sm_state_machine.SERVO_MIN_DEG = int(sv["min"])
        if "max" in sv:
            sm_state_machine.SERVO_MAX_DEG = int(sv["max"])
        if "center" in sv:
            sm_state_machine.SERVO_CENTER_DEG = int(sv["center"])
    if "sm_params" in data and isinstance(data["sm_params"], dict):
        _apply_sm_params_payload(state_machine, data["sm_params"])
    return True


# ─────────────────────────────────────────────────────────────────────────────
# HTTP MJPEG camera — for DroidCam / any HTTP MJPEG source
# ─────────────────────────────────────────────────────────────────────────────

class MJPEGHttpCamera:
    """
    Reads an HTTP MJPEG stream using requests (chunked) instead of OpenCV's
    ffmpeg backend, which cannot open HTTP MJPEG sources.
    """

    def __init__(self, url: str, timeout: float = 10.0) -> None:
        import requests  # local import — optional dep
        self._url = url
        self._stop = False
        self._eof = False
        self._latest: "np.ndarray | None" = None
        self._latest_jpeg: bytes | None = None  # raw bytes — no re-encode needed
        self._lock = threading.Lock()
        self._buffer = bytearray()
        try:
            self._resp = requests.get(url, stream=True, timeout=timeout)
            self._resp.raise_for_status()
        except Exception as exc:
            raise RuntimeError(f"Could not connect to MJPEG stream at {url}: {exc}") from exc
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self) -> None:
        try:
            for chunk in self._resp.iter_content(chunk_size=65536):
                if self._stop:
                    break
                if not chunk:
                    continue
                with self._lock:
                    self._buffer.extend(chunk)
                while True:
                    with self._lock:
                        data = bytes(self._buffer)
                    jpeg_bytes, consumed = _read_one_jpeg_from_stream(data)
                    if jpeg_bytes is None:
                        if consumed > 0:
                            with self._lock:
                                del self._buffer[:consumed]
                        break
                    with self._lock:
                        del self._buffer[:consumed]
                    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self._lock:
                            self._latest = frame
                            self._latest_jpeg = jpeg_bytes
        except Exception:
            pass
        finally:
            self._eof = True

    def read(self) -> "np.ndarray | None":
        deadline = time.perf_counter() + 5.0
        while self._latest is None and not self._eof and time.perf_counter() < deadline:
            time.sleep(0.005)
        with self._lock:
            return self._latest

    def read_jpeg(self) -> bytes | None:
        """Return the latest raw JPEG bytes without any re-encoding."""
        with self._lock:
            return self._latest_jpeg

    def release(self) -> None:
        self._stop = True
        try:
            self._resp.close()
        except Exception:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


# ─────────────────────────────────────────────────────────────────────────────
# Perception overlay drawing (ported from test_hough_redline.py, text removed)
# ─────────────────────────────────────────────────────────────────────────────

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _normalize(vx: float, vy: float) -> tuple[float, float]:
    n = math.hypot(vx, vy)
    if n < 1e-6:
        return 0.0, 0.0
    return vx / n, vy / n


def _compute_guidance_vectors(
    debug: dict,
    w: int,
    h: int,
    path_vec: tuple[float, float],
    line_error_x: float,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[int, int], tuple[int, int]]:
    origin = (w // 2, h - 10)
    sel = debug.get("selected_segment")
    nearest_dbg = debug.get("nearest_point")

    if not (isinstance(sel, list) and len(sel) == 4):
        path_vec = _normalize(float(path_vec[0]), float(path_vec[1]))
        dx = _clamp(float(line_error_x), -1.0, 1.0) * (w / 2.0)
        lateral_vec = (_clamp(float(line_error_x), -1.0, 1.0), 0.0)
        return_vec = _normalize(dx, -0.7 * h)
        nearest = (int(origin[0] + dx), origin[1])
        return path_vec, return_vec, lateral_vec, origin, nearest

    x1, y1, x2, y2 = [float(v) for v in sel]
    path_vec = _normalize(float(path_vec[0]), float(path_vec[1]))
    ox, oy = float(origin[0]), float(origin[1])
    abx, aby = (x2 - x1), (y2 - y1)
    ab2 = abx * abx + aby * aby
    if isinstance(nearest_dbg, list) and len(nearest_dbg) == 2:
        nearest_x, nearest_y = float(nearest_dbg[0]), float(nearest_dbg[1])
    elif ab2 < 1e-6:
        nearest_x, nearest_y = x1, y1
    else:
        t = ((ox - x1) * abx + (oy - y1) * aby) / ab2
        t = _clamp(t, 0.0, 1.0)
        nearest_x = x1 + t * abx
        nearest_y = y1 + t * aby

    return_vec = _normalize(nearest_x - ox, nearest_y - oy)
    lateral_vec = (_clamp(float(line_error_x), -1.0, 1.0), 0.0)
    nearest = (int(round(nearest_x)), int(round(nearest_y)))
    return path_vec, return_vec, lateral_vec, origin, nearest


def _draw_perception_overlay(
    roi: np.ndarray,
    debug: dict,
    path_vec: tuple[float, float],
    return_vec: tuple[float, float],
    lateral_vec: tuple[float, float],
    origin: tuple[int, int],
    nearest_point: tuple[int, int],
    line_error_x: float,
) -> np.ndarray:
    """Full vector overlay without any text labels."""
    out = roi.copy()
    h, w = out.shape[:2]
    cx = w // 2

    # Center reference line + lookahead line
    cv2.line(out, (cx, 0), (cx, h), (100, 100, 100), 1)
    lookahead_y = int(float(debug.get("lookahead_y", max(0, h - 1 - int(0.35 * h)))))
    cv2.line(out, (0, lookahead_y), (w, lookahead_y), (120, 120, 255), 1)

    # All candidate Hough segments (thin green)
    for seg in debug.get("hough_segments", []):
        cv2.line(out, (int(seg[0]), int(seg[1])), (int(seg[2]), int(seg[3])), (0, 200, 0), 1)

    # Centerline points from mask (white polyline)
    centerline_points = debug.get("centerline_points", [])
    if isinstance(centerline_points, list) and len(centerline_points) >= 2:
        pts = np.array([[int(p[0]), int(p[1])] for p in centerline_points], dtype=np.int32)
        cv2.polylines(out, [pts], isClosed=False, color=(240, 240, 240), thickness=2)

    # Selected dominant segment (thick red)
    sel = debug.get("selected_segment")
    if isinstance(sel, list) and len(sel) == 4:
        sx1, sy1, sx2, sy2 = [int(v) for v in sel]
        cv2.line(out, (sx1, sy1), (sx2, sy2), (0, 0, 255), 3)
        cv2.circle(out, (sx1, sy1), 5, (0, 255, 255), -1)
        cv2.circle(out, (sx2, sy2), 5, (0, 255, 255), -1)

    # Robot origin
    ox, oy = origin
    cv2.circle(out, (ox, oy), 6, (255, 255, 255), -1)

    arrow_len = int(min(h, w) * 0.42)
    # 1) Path direction vector (cyan)
    cv2.arrowedLine(out, (ox, oy),
                    (int(ox + path_vec[0] * arrow_len), int(oy + path_vec[1] * arrow_len)),
                    (255, 255, 0), 4, tipLength=0.24)
    # 2) Return-to-line vector (yellow)
    rl = int(arrow_len * 0.85)
    cv2.arrowedLine(out, (ox, oy),
                    (int(ox + return_vec[0] * rl), int(oy + return_vec[1] * rl)),
                    (0, 255, 255), 4, tipLength=0.24)
    # 3) Lateral correction vector (magenta), proportional to |line_error_x|
    ll = int(arrow_len * 0.75)
    cv2.arrowedLine(out, (ox, oy),
                    (int(ox + _clamp(float(lateral_vec[0]), -1.0, 1.0) * ll), oy),
                    (255, 0, 255), 4, tipLength=0.24)

    # Nearest point + connector
    cv2.circle(out, nearest_point, 6, (0, 255, 255), -1)
    cv2.line(out, (ox, oy), nearest_point, (0, 255, 255), 2)

    # Lateral error bar at bottom (smoothed)
    bar_y = h - 4
    bar_half = w // 2
    err_px = int(line_error_x * bar_half)
    bar_color = (0, 255, 0) if abs(line_error_x) < 0.15 else (0, 165, 255) if abs(line_error_x) < 0.4 else (0, 0, 255)
    cv2.rectangle(out, (cx, bar_y - 10), (cx + err_px, bar_y), bar_color, -1)
    cv2.line(out, (cx, bar_y - 12), (cx, bar_y + 2), (255, 255, 255), 2)

    return out


# ─────────────────────────────────────────────────────────────────────────────
# Direct robot client — replaces HTTP ControlCommClient
# ─────────────────────────────────────────────────────────────────────────────

class _DirectControlClient:
    """Same interface as ControlCommClient but drives the Robot object directly."""

    def __init__(self, robot: Any) -> None:
        self._robot = robot

    def send_control(
        self, x: float, y: float, speed: float, servo: Optional[int] = None
    ) -> dict:
        if self._robot is not None:
            self._robot.set_direction(float(x), float(y))
            self._robot.set_speed(float(speed))
        return {"ok": True}

    def get_control(self) -> dict:
        return {"ok": True, "data": {"command": {"x": 0.0, "y": 0.0, "speed": 0.0}}}

    def get_line_follow_pid(self) -> dict:
        return {"ok": False, "values": {}}


# ─────────────────────────────────────────────────────────────────────────────
# Stale-input helper
# ─────────────────────────────────────────────────────────────────────────────

def _safe_inputs(inputs: Inputs) -> Inputs:
    z = Vector(detected=False, x=0.0, y=0.0)
    return Inputs(
        heading_rad=inputs.heading_rad,
        speed=0.0,
        red_line=z, line_error=z, home=z,
        danger_zone=z, target=z, safe_zone=z,
    )


# ─────────────────────────────────────────────────────────────────────────────
# FastAPI app factory
# ─────────────────────────────────────────────────────────────────────────────

def _build_app(
    state_machine: StateMachine,
    robot: Any,
    cfg: AppConfig,
    *,
    tuning_path: Path,
    persist_tuning: bool,
) -> FastAPI:
    ui_dir = _HERE / "ui"
    app = FastAPI()

    def _persist_tuning() -> None:
        if not persist_tuning:
            return
        with _UI_TUNING_SAVE_LOCK:
            _save_ui_tuning(tuning_path, state_machine, cfg)

    # ── Serve UI ──────────────────────────────────────────────────────────────
    @app.get("/")
    async def index():
        return FileResponse(ui_dir / "index.html")

    # ── MJPEG camera stream (sync generator → threadpool per Starlette) ─────────
    def _mjpeg_frames():
        while True:
            shared.frame_event.wait(timeout=0.5)
            shared.frame_event.clear()
            frame = shared.get_frame()
            if frame:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )

    @app.get("/stream")
    async def stream():
        return StreamingResponse(
            _mjpeg_frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    # ── Status ─────────────────────────────────────────────────────────────────
    @app.get("/api/status")
    async def api_status():
        return shared.snapshot()

    @app.get("/api/tuning")
    async def api_get_tuning():
        """Full tuning snapshot (same shape as saved JSON under `data/ui_tuning.json`)."""
        return {
            "path": str(tuning_path),
            "persist_enabled": persist_tuning,
            "data": _ui_tuning_snapshot(state_machine, cfg),
        }

    # ── Line-follow PID ─────────────────────────────────────────────────────────
    @app.get("/api/pid")
    async def api_get_pid():
        return {
            "values": state_machine.get_line_follow_pid_settings(),
            "bounds": LINE_FOLLOW_PID_BOUNDS,
        }

    @app.post("/api/pid")
    async def api_set_pid(payload: dict[str, Any] = Body(default_factory=dict)):
        updated, errors = state_machine.apply_line_follow_pid_updates(payload)
        if errors:
            return JSONResponse(
                status_code=400,
                content={"ok": False, "errors": errors},
            )
        out = {
            "ok": True,
            "values": state_machine.get_line_follow_pid_settings(),
            "updated": updated,
        }
        _persist_tuning()
        return out

    # ── Servo range ────────────────────────────────────────────────────────────
    @app.get("/api/servo")
    async def api_get_servo():
        return {
            "min": sm_state_machine.SERVO_MIN_DEG,
            "max": sm_state_machine.SERVO_MAX_DEG,
            "center": sm_state_machine.SERVO_CENTER_DEG,
        }

    @app.post("/api/servo")
    async def api_set_servo(payload: dict[str, Any] = Body(default_factory=dict)):
        if "min" in payload:
            sm_state_machine.SERVO_MIN_DEG = int(payload["min"])
        if "max" in payload:
            sm_state_machine.SERVO_MAX_DEG = int(payload["max"])
        if "center" in payload:
            sm_state_machine.SERVO_CENTER_DEG = int(payload["center"])
        out = {
            "ok": True,
            "min": sm_state_machine.SERVO_MIN_DEG,
            "max": sm_state_machine.SERVO_MAX_DEG,
            "center": sm_state_machine.SERVO_CENTER_DEG,
        }
        _persist_tuning()
        return out

    # ── Perception smoothing params (live-tunable, applied each pipeline tick) ──
    @app.get("/api/smoothing")
    async def api_get_smoothing():
        return _smoothing_dict_from_cfg(cfg)

    @app.post("/api/smoothing")
    async def api_set_smoothing(payload: dict[str, Any] = Body(default_factory=dict)):
        _apply_smoothing_payload(cfg, payload)
        out = _smoothing_dict_from_cfg(cfg)
        _persist_tuning()
        return out

    # ── State machine runtime params ────────────────────────────────────────────
    @app.get("/api/sm-params")
    async def api_get_sm_params():
        return _sm_params_dict(state_machine)

    @app.post("/api/sm-params")
    async def api_set_sm_params(payload: dict[str, Any] = Body(default_factory=dict)):
        _apply_sm_params_payload(state_machine, payload)
        out = _sm_params_dict(state_machine)
        _persist_tuning()
        return out

    # ── State control ──────────────────────────────────────────────────────────
    @app.get("/api/states")
    async def api_states():
        return [s.value for s in State]

    @app.post("/api/set-state")
    async def api_set_state(payload: dict[str, Any] = Body(default_factory=dict)):
        state_name = payload.get("state", "")
        try:
            target = State(state_name)
        except ValueError:
            return JSONResponse(
                status_code=400,
                content={"ok": False, "message": f"Unknown state: {state_name!r}"},
            )
        state_machine.force_state(target)
        shared.set_sm_state(state_machine.state.value)
        return {"ok": True, "state": state_name}

    @app.post("/api/reset")
    async def api_reset():
        state_machine.reset()
        shared.set_sm_state(state_machine.state.value)
        return {"ok": True}

    app.mount("/ui", StaticFiles(directory=str(ui_dir)), name="ui")

    return app


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    args = _parse_args()

    # ── Init robot ─────────────────────────────────────────────────────────────
    robot: Any = None
    if not args.no_robot and _RobotCls is not None:
        try:
            robot = _RobotCls(args.serial, 115200)
            robot.start()
            print(f"[run] Robot started on {args.serial}", flush=True)
        except Exception as exc:
            print(f"[run] Robot init failed ({exc}) — software-only mode", file=sys.stderr)
            robot = None

    shared.robot_active = robot is not None

    # ── Monkey-patch state machine to bypass HTTP ──────────────────────────────
    direct_client = _DirectControlClient(robot)
    sm_state_machine.control = direct_client
    sm_state_machine._ASYNC_SENDER = sm_state_machine._AsyncControlSender(
        direct_client, sm_state_machine.ASYNC_CONTROL_SEND_HZ
    )

    # ── State machine ──────────────────────────────────────────────────────────
    input_buffer = InputBuffer(
        target_align_threshold=float(os.getenv("TARGET_ALIGN_THRESHOLD", "0.15")),
        place_align_threshold=float(os.getenv("PLACE_ALIGN_THRESHOLD", "0.2")),
    )
    state_machine = StateMachine()
    shared.set_sm_state(state_machine.state.value)

    # ── Perception config ──────────────────────────────────────────────────────
    cfg = load_config(args.config)
    cfg.fps = args.fps
    cfg.camera.source = args.source
    cfg.comms.method = "stdout"          # we feed to input_buffer directly

    tuning_path = Path(
        os.environ.get("RUN_UI_TUNING_PATH", str(_DEFAULT_UI_TUNING_PATH))
    ).expanduser()
    persist_ui_tuning = os.environ.get("RUN_UI_TUNING_DISABLE", "").strip().lower() not in {
        "1",
        "true",
        "yes",
    }
    if persist_ui_tuning:
        if tuning_path.is_file():
            if _load_ui_tuning(tuning_path, state_machine, cfg):
                print(f"[run] Restored UI tuning from {tuning_path}", flush=True)
            else:
                print(f"[run] UI tuning file unreadable (using defaults): {tuning_path}", flush=True)
        else:
            print(f"[run] UI tuning will save to {tuning_path} after Apply", flush=True)

    # ── Camera ────────────────────────────────────────────────────────────────
    _HTTP_PREFIXES = ("http://", "https://")
    _NET_PREFIXES = ("rtsp://", "udp://", "tcp://")
    try:
        if args.source == "rpicam":
            cam = RpicamVidCamera(
                width=cfg.camera.width,
                height=cfg.camera.height,
                fps=cfg.fps,
                camera_index=cfg.camera.webcam_index,
            )
        elif args.source.startswith(_HTTP_PREFIXES):
            # HTTP MJPEG streams (DroidCam, IP cams, etc.) — OpenCV ffmpeg can't handle these
            cam = MJPEGHttpCamera(url=args.source)
        else:
            if args.source == "webcam":
                _cv_source = cfg.camera.webcam_index
                _backend = cfg.camera.backend
            elif args.source.startswith(_NET_PREFIXES):
                _cv_source = args.source
                _backend = "ffmpeg"
            else:
                _cv_source = args.source
                _backend = "gstreamer"
            cam = OpenCVCamera(
                source=_cv_source,
                width=cfg.camera.width,
                height=cfg.camera.height,
                fps=cfg.fps,
                backend=_backend,
                threaded=True,
            )
    except RuntimeError as exc:
        raise SystemExit(f"[run] Camera init failed: {exc}") from exc

    # ── H.264 UDP stream (optional, Pi HW encoder) ────────────────────────────
    h264_writer: _H264UdpWriter | None = None
    if args.h264_udp:
        try:
            h264_writer = _H264UdpWriter(
                dest=args.h264_udp,
                width=cfg.camera.width,
                height=cfg.camera.height - cfg.roi_y_start,  # overlay is ROI-sized
                fps=args.fps,
                encoder=args.h264_encoder,
                bitrate=args.h264_bitrate,
            )
            print(f"[run] H.264 UDP → {args.h264_udp} ({args.h264_encoder})", flush=True)
            print(f"[run] View on laptop: ffplay -fflags nobuffer -flags low_delay "
                  f"-framedrop 'udp://0.0.0.0:{args.h264_udp.split(':')[-1]}"
                  f"?localaddr=<laptop-tailscale-ip>'", flush=True)
        except RuntimeError as exc:
            print(f"[run] H.264 UDP disabled: {exc}", file=sys.stderr)

    stop_event = threading.Event()

    # ── Capture thread ─────────────────────────────────────────────────────────
    frame_q: queue.Queue = queue.Queue(maxsize=2)

    def _capture_loop() -> None:
        try:
            while not stop_event.is_set():
                frame = cam.read()
                if frame is None:
                    break
                ts = time.time()
                # Provide a raw frame only as startup fallback (before first overlay is ready).
                # Use set_raw_frame so it never overwrites the annotated overlay.
                if isinstance(cam, MJPEGHttpCamera):
                    raw_jpeg = cam.read_jpeg()
                    if raw_jpeg:
                        shared.set_raw_frame(raw_jpeg)
                else:
                    _, raw_buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 55])
                    shared.set_raw_frame(raw_buf.tobytes())
                try:
                    frame_q.put_nowait((ts, frame))
                except queue.Full:
                    try:
                        frame_q.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        frame_q.put_nowait((ts, frame))
                    except queue.Full:
                        pass
        finally:
            try:
                frame_q.put_nowait(None)
            except queue.Full:
                pass

    # ── Vision worker thread ───────────────────────────────────────────────────
    pipeline_state = PipelineState()
    pipeline_state.path_mask_key = "red"

    _fps_count = 0
    _fps_start = time.time()
    _fps_cur = 0.0

    def _worker_loop() -> None:
        nonlocal _fps_count, _fps_start, _fps_cur
        while not stop_event.is_set():
            try:
                item = frame_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:
                break
            _, frame = item

            roi = crop_roi(frame, cfg.roi_y_start)
            out = run_pipeline(roi_bgr=roi, state=pipeline_state, cfg=cfg)

            # FPS counter
            _fps_count += 1
            now = time.time()
            dt = now - _fps_start
            if dt >= 1.0:
                _fps_cur = _fps_count / dt
                _fps_count = 0
                _fps_start = now

            shared.update_perception(out, _fps_cur)

            # Annotated frame → JPEG
            px_out, py_out = to_robot_frame_clamped(out.px, out.py)
            hd = out.debug_artifacts.get("heading_debug", {})
            smoothing_dbg = out.debug_artifacts.get("smoothing_debug", {})
            smoothed_err = float(smoothing_dbg.get("line_error_median", out.line_error_x))
            path_vec, return_vec, lateral_vec, origin, nearest_pt = _compute_guidance_vectors(
                debug=hd,
                w=roi.shape[1],
                h=roi.shape[0],
                path_vec=(float(out.px), float(out.py)),
                line_error_x=smoothed_err,
            )
            overlay = _draw_perception_overlay(
                roi, hd,
                path_vec=path_vec,
                return_vec=return_vec,
                lateral_vec=lateral_vec,
                origin=origin,
                nearest_point=nearest_pt,
                line_error_x=smoothed_err,
            )
            _, buf = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 55])
            shared.set_frame(buf.tobytes())

            if h264_writer is not None:
                h264_writer.write(overlay)

            # Feed into state machine input buffer
            line_key = "black_line" if out.path_mask_key == "black" else "red_line"
            payload: dict = {
                "target": {
                    "detected": out.target_detected,
                    "vector": {"x": float(out.target_px), "y": float(out.target_py)},
                },
                line_key: {
                    "detected": out.path_detected,
                    "vector": {"x": float(px_out), "y": float(py_out)},
                },
                "line_error": {
                    "detected": out.path_detected,
                    "vector": {"x": float(out.line_error_x), "y": float(py_out)},
                },
            }
            input_buffer.update(payload)

    # ── State machine thread ───────────────────────────────────────────────────
    MAX_INPUT_AGE_S = max(float(os.getenv("MAX_INPUT_AGE_S", "0.25")), 0.01)

    def _sm_worker() -> None:
        while not stop_event.is_set():
            inputs, _, updated_at = input_buffer.snapshot()
            now = time.time()
            stale = updated_at is None or (now - updated_at) > MAX_INPUT_AGE_S
            if stale:
                inputs = _safe_inputs(inputs)
            state_machine.step(inputs)
            shared.set_sm_state(state_machine.state.value)
            time.sleep(state_machine.tick_interval)

    # ── Start threads ──────────────────────────────────────────────────────────
    threads = [
        threading.Thread(target=_capture_loop, daemon=True, name="capture"),
        threading.Thread(target=_worker_loop,  daemon=True, name="vision"),
        threading.Thread(target=_sm_worker,    daemon=True, name="state_machine"),
    ]
    for t in threads:
        t.start()

    # ── FastAPI (uvicorn) web server ───────────────────────────────────────────
    app = _build_app(
        state_machine,
        robot,
        cfg,
        tuning_path=tuning_path,
        persist_tuning=persist_ui_tuning,
    )
    print(f"[run] Web UI → http://0.0.0.0:{args.port}", flush=True)
    print(f"[run] Camera source: {args.source}", flush=True)
    print(f"[run] Robot active: {robot is not None}", flush=True)

    try:
        uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="info")
    finally:
        stop_event.set()
        cam.release()
        if h264_writer is not None:
            h264_writer.close()
        if robot is not None:
            try:
                robot.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()
