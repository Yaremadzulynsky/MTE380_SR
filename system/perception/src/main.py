from __future__ import annotations

import argparse
import copy
import json
import math
import os
import platform
import re
import shutil
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import requests
import yaml

from src.config import (
    AppConfig,
    PERCEPTION_TUNING_RANGES,
    load_config,
    perception_tuning_ranges,
    perception_tuning_values,
)
from src.debug_stream import DebugStreamServer
from src.pipeline import PipelineOutput, PipelineState, draw_overlay, run_pipeline


def _find_macos_camera_index(pattern: str) -> int:
    probe = pattern.strip().lower()
    if not probe:
        raise RuntimeError("Camera name pattern cannot be empty")
    if platform.system() != "Darwin":
        raise RuntimeError(f"Named camera lookup is only supported on macOS: {pattern}")

    result = subprocess.run(
        ["ffmpeg", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
        capture_output=True,
        text=True,
        check=False,
    )
    listing = f"{result.stdout}\n{result.stderr}"
    in_video = False
    matches: list[tuple[int, str]] = []
    for line in listing.splitlines():
        if "AVFoundation video devices:" in line:
            in_video = True
            continue
        if "AVFoundation audio devices:" in line:
            break
        if not in_video:
            continue
        found = re.search(r"\[(\d+)\]\s+(.+)$", line)
        if not found:
            continue
        index = int(found.group(1))
        name = found.group(2).strip()
        matches.append((index, name))
        if probe in name.lower():
            return index

    available = ", ".join(f"{index}:{name}" for index, name in matches) or "none"
    raise RuntimeError(f"Could not find macOS camera matching '{pattern}'. Available video devices: {available}")


def _parse_source(source: str) -> int | str:
    if source == "webcam":
        return 0
    if source in {"rpicam", "raspberry-pi-cam", "pi-cam"}:
        return "rpicam"
    if source in {"udp", "udp-stream", "ffplay-udp"}:
        return "udp://0.0.0.0:5000"
    if source in {"iphone", "continuity", "continuity-camera"}:
        return _find_macos_camera_index("iphone")
    if source in {"deskview", "desk-view"}:
        return _find_macos_camera_index("desk view")
    if source.startswith("camera:"):
        return _find_macos_camera_index(source.split("camera:", 1)[1])
    if source.startswith("webcam:"):
        return int(source.split("webcam:", 1)[1])
    if source.startswith("video:"):
        return source.split("video:", 1)[1]
    if source.isdigit():
        return int(source)
    return source


def _with_ffmpeg_capture_options(options: str):
    class _EnvContext:
        def __enter__(self_nonlocal):
            self_nonlocal._previous = os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS")
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = options
            return None

        def __exit__(self_nonlocal, exc_type, exc, tb):
            if self_nonlocal._previous is None:
                os.environ.pop("OPENCV_FFMPEG_CAPTURE_OPTIONS", None)
            else:
                os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = self_nonlocal._previous
            return False

    return _EnvContext()


def _open_capture(cfg: AppConfig) -> cv2.VideoCapture:
    source = _parse_source(cfg.source)
    if isinstance(source, str) and "://" in source:
        ffmpeg_options = None
        if source.startswith("udp://"):
            ffmpeg_options = (
                "fflags;nobuffer|flags;low_delay|overrun_nonfatal;1|fifo_size;5000000"
            )
        if ffmpeg_options is not None:
            with _with_ffmpeg_capture_options(ffmpeg_options):
                cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
        else:
            cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
    elif isinstance(source, int) and platform.system() == "Darwin" and hasattr(cv2, "CAP_AVFOUNDATION"):
        cap = cv2.VideoCapture(source, cv2.CAP_AVFOUNDATION)
    else:
        cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
    cap.set(cv2.CAP_PROP_FPS, cfg.fps)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera/video source: {cfg.source}")
    return cap


def _is_live_source(source: str) -> bool:
    if source == "webcam" or source.startswith("webcam:") or source.isdigit():
        return True
    return not source.startswith("video:")


class LatestFrameReader:
    def __init__(self, cap: cv2.VideoCapture):
        self._cap = cap
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame = None
        self._seq = 0
        self._closed = False
        self._failed = False
        self._thread = threading.Thread(target=self._run, name="perception-capture", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while True:
            ok, frame = self._cap.read()
            with self._cond:
                if self._closed:
                    return
                if not ok or frame is None:
                    self._failed = True
                    self._cond.notify_all()
                    return
                self._frame = frame
                self._seq += 1
                self._cond.notify_all()

    def read(self, last_seq: int, timeout_s: float = 1.0) -> tuple[int, object | None]:
        deadline = time.time() + timeout_s
        with self._cond:
            while self._seq <= last_seq and not self._failed and not self._closed:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self._cond.wait(timeout=remaining)
            if self._seq <= last_seq:
                return last_seq, None
            return self._seq, self._frame.copy()

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()
        self._thread.join(timeout=1.0)


class FfmpegFrameReader:
    def __init__(self, source: str, width: int, height: int):
        self._width = int(width)
        self._height = int(height)
        self._frame_size = self._width * self._height * 3
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame = None
        self._seq = 0
        self._closed = False
        self._failed = False
        self._proc = subprocess.Popen(
            [
                "ffmpeg",
                "-loglevel",
                "error",
                "-fflags",
                "nobuffer",
                "-flags",
                "low_delay",
                "-i",
                source,
                "-vf",
                f"scale={self._width}:{self._height}",
                "-pix_fmt",
                "bgr24",
                "-f",
                "rawvideo",
                "pipe:1",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            stdin=subprocess.DEVNULL,
            bufsize=self._frame_size * 2,
        )
        if self._proc.stdout is None:
            raise RuntimeError("Failed to open ffmpeg stdout for UDP stream.")
        self._thread = threading.Thread(target=self._run, name="perception-ffmpeg-capture", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        stdout = self._proc.stdout
        while True:
            chunk = stdout.read(self._frame_size)
            with self._cond:
                if self._closed:
                    return
                if len(chunk) != self._frame_size:
                    self._failed = True
                    self._cond.notify_all()
                    return
                frame = np.frombuffer(chunk, dtype=np.uint8).reshape((self._height, self._width, 3)).copy()
                self._frame = frame
                self._seq += 1
                self._cond.notify_all()

    def read(self, last_seq: int, timeout_s: float = 1.0) -> tuple[int, object | None]:
        deadline = time.time() + timeout_s
        with self._cond:
            while self._seq <= last_seq and not self._failed and not self._closed:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self._cond.wait(timeout=remaining)
            if self._seq <= last_seq:
                return last_seq, None
            return self._seq, self._frame.copy()

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._thread.join(timeout=1.0)


class RpicamFrameReader:
    def __init__(self, width: int, height: int, fps: float):
        if shutil.which("rpicam-vid") is None:
            raise RuntimeError("rpicam-vid is required for source 'rpicam' on Raspberry Pi.")
        if shutil.which("ffmpeg") is None:
            raise RuntimeError("ffmpeg is required for source 'rpicam' on Raspberry Pi.")

        self._width = int(width)
        self._height = int(height)
        self._frame_size = self._width * self._height * 3
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame = None
        self._seq = 0
        self._closed = False
        self._failed = False
        self._camera_proc = subprocess.Popen(
            [
                "rpicam-vid",
                "--nopreview",
                "--timeout",
                "0",
                "--width",
                str(self._width),
                "--height",
                str(self._height),
                "--framerate",
                str(max(float(fps), 1.0)),
                "--codec",
                "mjpeg",
                "--output",
                "-",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            stdin=subprocess.DEVNULL,
            bufsize=0,
        )
        if self._camera_proc.stdout is None:
            raise RuntimeError("Failed to open rpicam-vid stdout.")

        self._decode_proc = subprocess.Popen(
            [
                "ffmpeg",
                "-loglevel",
                "error",
                "-fflags",
                "nobuffer",
                "-flags",
                "low_delay",
                "-f",
                "mjpeg",
                "-i",
                "pipe:0",
                "-vf",
                f"scale={self._width}:{self._height}",
                "-pix_fmt",
                "bgr24",
                "-f",
                "rawvideo",
                "pipe:1",
            ],
            stdin=self._camera_proc.stdout,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=self._frame_size * 2,
        )
        self._camera_proc.stdout.close()
        if self._decode_proc.stdout is None:
            raise RuntimeError("Failed to open ffmpeg stdout for rpicam.")
        self._thread = threading.Thread(target=self._run, name="perception-rpicam-capture", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        stdout = self._decode_proc.stdout
        while True:
            chunk = stdout.read(self._frame_size)
            with self._cond:
                if self._closed:
                    return
                if len(chunk) != self._frame_size:
                    self._failed = True
                    self._cond.notify_all()
                    return
                frame = np.frombuffer(chunk, dtype=np.uint8).reshape((self._height, self._width, 3)).copy()
                self._frame = frame
                self._seq += 1
                self._cond.notify_all()

    def read(self, last_seq: int, timeout_s: float = 1.0) -> tuple[int, object | None]:
        deadline = time.time() + timeout_s
        with self._cond:
            while self._seq <= last_seq and not self._failed and not self._closed:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self._cond.wait(timeout=remaining)
            if self._seq <= last_seq:
                return last_seq, None
            return self._seq, self._frame.copy()

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()
        for proc in (self._decode_proc, self._camera_proc):
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self._thread.join(timeout=1.0)


class AsyncPayloadSender:
    def __init__(self, cfg: AppConfig):
        self._cfg = cfg
        self._session = requests.Session()
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._payload: dict[str, object] | None = None
        self._closed = False
        self._min_interval_s = 0.0 if cfg.output.send_hz <= 0 else 1.0 / float(cfg.output.send_hz)
        self._last_sent_at = 0.0
        self._thread = threading.Thread(target=self._run, name="perception-sender", daemon=True)
        self._thread.start()

    def submit(self, payload: dict[str, object]) -> None:
        with self._cond:
            self._payload = payload
            self._cond.notify()

    def _run(self) -> None:
        while True:
            with self._cond:
                while self._payload is None and not self._closed:
                    self._cond.wait()
                if self._closed:
                    return
                now = time.time()
                wait_s = self._min_interval_s - (now - self._last_sent_at)
                if wait_s > 0:
                    self._cond.wait(timeout=wait_s)
                    continue
                payload = self._payload
                self._payload = None
            try:
                self._session.post(self._cfg.output_url, json=payload, timeout=self._cfg.output.timeout_s)
                self._last_sent_at = time.time()
            except requests.RequestException as exc:
                print(json.dumps({"event": "perception_send_error", "message": str(exc)}), flush=True)

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()
        self._thread.join(timeout=1.0)
        self._session.close()


class RuntimePerceptionConfig:
    def __init__(self, cfg: AppConfig, config_path: Path, state: PipelineState):
        self._cfg = cfg
        self._config_path = config_path
        self._state = state
        self._lock = threading.Lock()

    def snapshot(self) -> AppConfig:
        with self._lock:
            return copy.deepcopy(self._cfg)

    def get_payload(self) -> dict[str, object]:
        with self._lock:
            return {
                "values": perception_tuning_values(self._cfg),
                "ranges": perception_tuning_ranges(),
                "source_config_path": str(self._config_path.resolve()),
            }

    def update(self, updates: dict[str, object]) -> tuple[dict[str, object], int]:
        cleaned: dict[str, float] = {}
        errors: dict[str, str] = {}
        for key, raw_value in updates.items():
            bounds = PERCEPTION_TUNING_RANGES.get(key)
            if bounds is None:
                errors[key] = "Unknown setting."
                continue
            try:
                value = float(raw_value)
            except (TypeError, ValueError):
                errors[key] = "Value must be a finite number."
                continue
            if not math.isfinite(value):
                errors[key] = "Value must be a finite number."
                continue
            lo, hi = bounds
            if value < lo or value > hi:
                errors[key] = f"Value must be between {lo} and {hi}."
                continue
            cleaned[key] = value

        if errors:
            return {"message": "Invalid perception tuning update.", "errors": errors}, 400
        if not cleaned:
            return {"message": "No settings to update."}, 400

        with self._lock:
            next_values = perception_tuning_values(self._cfg)
            next_values.update(cleaned)
            if next_values["min_speed"] > next_values["max_speed"]:
                return {
                    "message": "Invalid speed bounds.",
                    "errors": {
                        "min_speed": "min_speed must be <= max_speed.",
                        "max_speed": "max_speed must be >= min_speed.",
                    },
                }, 400

            for key, value in cleaned.items():
                setattr(self._cfg.heading, key, value)
            self._reset_pipeline_state_locked()
            saved = self._persist_locked()
            payload = {
                "values": perception_tuning_values(self._cfg),
                "updated": cleaned,
                "saved": saved,
                "source_config_path": str(self._config_path.resolve()),
            }
        return payload, 200

    def _reset_pipeline_state_locked(self) -> None:
        self._state.pid_integral = 0.0
        self._state.pid_prev_error = 0.0

    def _persist_locked(self) -> bool:
        existing: dict[str, object] = {}
        if self._config_path.exists():
            with self._config_path.open("r", encoding="utf-8") as handle:
                loaded = yaml.safe_load(handle) or {}
                if isinstance(loaded, dict):
                    existing = loaded
        heading = existing.get("heading")
        if not isinstance(heading, dict):
            heading = {}
            existing["heading"] = heading
        for key, value in perception_tuning_values(self._cfg).items():
            heading[key] = float(value)
        if self._config_path.parent:
            self._config_path.parent.mkdir(parents=True, exist_ok=True)
        with self._config_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(existing, handle, sort_keys=False)
        return True


def _vector_payload(detected: bool, x: float, y: float) -> dict[str, float | bool]:
    return {"detected": detected, "x": float(x), "y": float(y)}


def _build_payload(output: PipelineOutput, path_mask_key: str) -> dict[str, object]:
    line_key = f"{path_mask_key}_line"
    line = _vector_payload(output.path_detected, output.robot_vector[0], output.robot_vector[1])
    empty = _vector_payload(False, 0.0, 0.0)
    return {
        line_key: line,
        "line_error": line,
        "line": line,
        "target": empty,
        "safe_zone": empty,
        "danger_zone": empty,
        "path_detected": output.path_detected,
        "path_mask_key": path_mask_key,
        "zone": output.zone,
        "gamma": output.gamma,
    }


def _send_payload(payload: dict[str, object], cfg: AppConfig, session: requests.Session | None) -> None:
    if cfg.output.method == "none":
        return
    if cfg.output.method == "stdout":
        print(json.dumps(payload), flush=True)
        return
    if session is None:
        raise RuntimeError("HTTP output selected but no session is available")
    session.post(cfg.output_url, json=payload, timeout=cfg.output.timeout_s)


def _apply_overrides(cfg: AppConfig, args: argparse.Namespace) -> AppConfig:
    if getattr(args, "udp_stream", False):
        cfg.source = "udp://0.0.0.0:5000"
    if args.source is not None:
        cfg.source = args.source
    if args.fps is not None:
        cfg.fps = args.fps
    if args.path_mask is not None:
        cfg.path_mask_key = args.path_mask
    return cfg


def _open_live_reader(cfg: AppConfig):
    parsed = _parse_source(cfg.source)
    if parsed == "rpicam":
        return None, RpicamFrameReader(cfg.width, cfg.height, cfg.fps)
    if isinstance(parsed, str) and parsed.startswith("udp://"):
        return None, FfmpegFrameReader(parsed, cfg.width, cfg.height)
    cap = _open_capture(cfg)
    return cap, LatestFrameReader(cap)


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal perception node")
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--mode", choices=["test", "production"], default="production")
    parser.add_argument("--source", default=None)
    parser.add_argument("--udp-stream", action="store_true")
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--path-mask", choices=["red", "blue", "black"], default=None)
    parser.add_argument("--no-gui", action="store_true")
    args = parser.parse_args()

    config_path = Path(args.config)
    cfg = _apply_overrides(load_config(config_path), args)
    if args.mode == "test" and cfg.output.method == "http":
        cfg.output.method = "stdout"

    cap = None
    reader = None
    live_source = _is_live_source(cfg.source)
    if live_source:
        cap, reader = _open_live_reader(cfg)
    else:
        cap = _open_capture(cfg)
    state = PipelineState()
    runtime_cfg = RuntimePerceptionConfig(cfg, config_path, state)
    sender = AsyncPayloadSender(cfg) if cfg.output.method == "http" else None
    gui = args.mode == "test" and not args.no_gui
    frame_period = 1.0 / max(cfg.fps, 1.0)
    debug_stream = None
    if cfg.debug_stream.enabled:
        debug_stream = DebugStreamServer(
            host=cfg.debug_stream.host,
            port=cfg.debug_stream.port,
            jpeg_quality=cfg.debug_stream.jpeg_quality,
            max_fps=cfg.debug_stream.max_fps,
            tuning_api=runtime_cfg,
        )
        debug_stream.start()

    try:
        last_seq = 0
        while True:
            started = time.time()
            if reader is not None:
                last_seq, frame = reader.read(last_seq, timeout_s=1.0)
                if frame is None:
                    continue
            else:
                ok, frame = cap.read()
                if not ok or frame is None:
                    break

            loop_cfg = runtime_cfg.snapshot()
            output = run_pipeline(frame, state, loop_cfg)
            payload = _build_payload(output, loop_cfg.path_mask_key)

            if sender is not None:
                sender.submit(payload)
            else:
                _send_payload(payload, cfg, None)

            overlay = draw_overlay(output)
            if debug_stream is not None:
                debug_stream.update(overlay)
            if gui:
                cv2.imshow("perception_roi", overlay)
                cv2.imshow("perception_mask", output.mask)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

            if reader is None:
                elapsed = time.time() - started
                remaining = frame_period - elapsed
                if remaining > 0:
                    time.sleep(remaining)
    finally:
        if reader is not None:
            reader.close()
        if cap is not None:
            cap.release()
        if sender is not None:
            sender.close()
        if debug_stream is not None:
            debug_stream.close()
        if gui:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
