from __future__ import annotations

import argparse
import json
import platform
import re
import subprocess
import threading
import time
from pathlib import Path

import cv2
import requests

from src.config import AppConfig, load_config
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


def _open_capture(cfg: AppConfig) -> cv2.VideoCapture:
    source = _parse_source(cfg.source)
    if isinstance(source, str) and "://" in source:
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


class AsyncPayloadSender:
    def __init__(self, cfg: AppConfig):
        self._cfg = cfg
        self._session = requests.Session()
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._payload: dict[str, object] | None = None
        self._closed = False
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
                payload = self._payload
                self._payload = None
            try:
                self._session.post(self._cfg.output_url, json=payload, timeout=self._cfg.output.timeout_s)
            except requests.RequestException as exc:
                print(json.dumps({"event": "perception_send_error", "message": str(exc)}), flush=True)

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()
        self._thread.join(timeout=1.0)
        self._session.close()


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
    if args.source is not None:
        cfg.source = args.source
    if args.fps is not None:
        cfg.fps = args.fps
    if args.path_mask is not None:
        cfg.path_mask_key = args.path_mask
    return cfg


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal perception node")
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--mode", choices=["test", "production"], default="production")
    parser.add_argument("--source", default=None)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--path-mask", choices=["red", "blue", "black"], default=None)
    parser.add_argument("--no-gui", action="store_true")
    args = parser.parse_args()

    cfg = _apply_overrides(load_config(Path(args.config)), args)
    if args.mode == "test" and cfg.output.method == "http":
        cfg.output.method = "stdout"

    cap = _open_capture(cfg)
    live_source = _is_live_source(cfg.source)
    reader = LatestFrameReader(cap) if live_source else None
    state = PipelineState()
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

            output = run_pipeline(frame, state, cfg)
            payload = _build_payload(output, cfg.path_mask_key)

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
        cap.release()
        if sender is not None:
            sender.close()
        if debug_stream is not None:
            debug_stream.close()
        if gui:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
