"""Main perception loop for Milestone 3.3."""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import threading
import time
import urllib.request
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional

import cv2

from src.comms.http_tx import HTTPSender
from src.comms.packet import PerceptionPacket
from src.comms.serial_tx import SerialSender
from src.comms.udp_tx import UDPSender
from src.config import AppConfig, load_config
from src.pipeline import PipelineOutput, PipelineState, run_pipeline
from src.utils.logging import log
from src.utils.math2d import to_robot_frame_clamped
from src.utils.timing import LoopRegulator
from src.vision.camera import OpenCVCamera, RpicamVidCamera
from src.vision.debug_draw import draw_overlay, make_mask_preview
from src.vision.masks import crop_roi

LOOKAHEAD_MIN = 0.05
LOOKAHEAD_MAX = 0.9


def process_roi(
    roi_bgr: Any,
    state: PipelineState,
    cfg: AppConfig,
) -> tuple[float, float, float, str, float, bool, str, bool, float, float, dict[str, Any]]:
    """
    Single pipeline function:
      ROI BGR -> (px, py, line_error_x, zone, gamma, path_detected, path_mask_key, target_detected, target_px, target_py, debug_artifacts)
    """
    out: PipelineOutput = run_pipeline(roi_bgr=roi_bgr, state=state, cfg=cfg)
    return (
        out.px, out.py, out.line_error_x, out.zone, out.gamma,
        out.path_detected, out.path_mask_key,
        out.target_detected, out.target_px, out.target_py,
        out.debug_artifacts,
    )


@dataclass
class FrameItem:
    timestamp: float
    frame: Any


@dataclass
class PerceptionResult:
    timestamp: float
    output: PipelineOutput
    roi: Any


def _make_sender(method: str, cfg: AppConfig) -> Any:
    if method == "udp":
        return UDPSender(cfg.comms.udp_ip, cfg.comms.udp_port)
    if method == "serial":
        return SerialSender(cfg.comms.serial_port, cfg.comms.serial_baud)
    if method == "http":
        if not cfg.comms.http_url:
            raise ValueError("comms.method is 'http' but comms.http_url is not set")
        return HTTPSender(cfg.comms.http_url)
    if method == "stdout":
        return None
    raise ValueError(f"Unsupported comms method: {method}")


def _parse_source(source: str, cfg: AppConfig) -> int | str:
    if source == "webcam":
        return cfg.camera.webcam_index
    if source == "rpicam":
        return "rpicam"
    if source.startswith(("rtsp://", "http://", "https://", "udp://", "tcp://")):
        return source
    if source.startswith("video:"):
        return source.split("video:", 1)[1]
    src_path = Path(source)
    if src_path.exists():
        return str(src_path)
    raise ValueError(
        "source must be webcam, rpicam, video:/path/to/file, network URL (rtsp/http/udp/tcp), or an existing file path"
    )


def _send_zero_http_inputs(url: str, timeout_s: float = 0.5) -> None:
    mode_raw = (os.environ.get("PERCEPTION_HTTP_MODE") or "").strip().lower()
    direct_raw = (os.environ.get("PERCEPTION_HTTP_DIRECT_CONTROL") or "").strip().lower()
    direct_control = mode_raw == "control" or direct_raw in {"1", "true", "yes", "on"}
    if direct_control:
        payload = {"x": 0.0, "y": 0.0, "speed": 0.0}
    else:
        payload = {
            "black_line": {"detected": False, "vector": {"x": 0.0, "y": 0.0}},
            "red_line": {"detected": False, "vector": {"x": 0.0, "y": 0.0}},
            "target": {"detected": False, "vector": {"x": 0.0, "y": 0.0}},
        }
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout_s):
            pass
    except Exception as exc:
        log("perception_zero_vector_send_failed", error=str(exc), url=url)


def _start_runtime_settings_server(
    cfg: AppConfig,
    runtime_settings: dict[str, float],
    settings_lock: threading.Lock,
) -> ThreadingHTTPServer:
    port = int((os.environ.get("PERCEPTION_TUNING_PORT") or os.environ.get("PORT") or "4000").strip())

    class RuntimeSettingsHandler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:
            return

        def _json(self, status: int, payload: dict[str, Any]) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _values(self) -> dict[str, float]:
            with settings_lock:
                return {"centerline_lookahead_ratio": float(runtime_settings["centerline_lookahead_ratio"])}

        def do_GET(self) -> None:
            if self.path != "/runtime-settings":
                self._json(404, {"ok": False, "message": "Not found."})
                return
            self._json(
                200,
                {
                    "ok": True,
                    "values": self._values(),
                    "bounds": {
                        "centerline_lookahead_ratio": {
                            "min": LOOKAHEAD_MIN,
                            "max": LOOKAHEAD_MAX,
                        }
                    },
                },
            )

        def do_POST(self) -> None:
            if self.path != "/runtime-settings":
                self._json(404, {"ok": False, "message": "Not found."})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                length = 0
            raw = self.rfile.read(max(0, length)) if length > 0 else b"{}"
            try:
                payload = json.loads(raw.decode("utf-8") or "{}")
            except json.JSONDecodeError:
                self._json(400, {"ok": False, "message": "Invalid JSON payload."})
                return
            if not isinstance(payload, dict):
                self._json(400, {"ok": False, "message": "Payload must be an object."})
                return
            if "centerline_lookahead_ratio" not in payload:
                self._json(400, {"ok": False, "message": "Missing 'centerline_lookahead_ratio'."})
                return
            value = payload.get("centerline_lookahead_ratio")
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                self._json(400, {"ok": False, "message": "'centerline_lookahead_ratio' must be numeric."})
                return
            numeric = float(value)
            if not math.isfinite(numeric):
                self._json(400, {"ok": False, "message": "'centerline_lookahead_ratio' must be finite."})
                return
            if numeric < LOOKAHEAD_MIN or numeric > LOOKAHEAD_MAX:
                self._json(
                    400,
                    {
                        "ok": False,
                        "message": f"'centerline_lookahead_ratio' must be between {LOOKAHEAD_MIN} and {LOOKAHEAD_MAX}.",
                    },
                )
                return
            with settings_lock:
                runtime_settings["centerline_lookahead_ratio"] = numeric
                cfg.smoothing.centerline_lookahead_ratio = numeric
            log("perception_runtime_setting_updated", centerline_lookahead_ratio=numeric)
            self._json(200, {"ok": True, "values": self._values()})

    server = ThreadingHTTPServer(("0.0.0.0", port), RuntimeSettingsHandler)
    threading.Thread(target=server.serve_forever, name="perception_runtime_settings", daemon=True).start()
    log("perception_runtime_settings_server", port=port, path="/runtime-settings")
    return server


def main() -> None:
    parser = argparse.ArgumentParser(description="Pi perception node (Milestone 3.3)")
    parser.add_argument("--config", default="configs/default.yaml", help="Path to YAML config")
    parser.add_argument(
        "--mode",
        choices=["test", "production"],
        default=None,
        help="test: GUI + heading arrow + packet logging (stdout). production: full packet over UDP/serial, no GUI",
    )
    parser.add_argument("--source", default=None, help="webcam or video:/path/to/file")
    parser.add_argument("--comms", choices=["udp", "serial", "stdout", "http"], default=None)
    parser.add_argument("--fps", type=float, default=None)
    args = parser.parse_args()

    cfg_path = Path(args.config)
    cfg = load_config(cfg_path)
    if args.source is not None:
        cfg.camera.source = args.source
    if args.fps is not None:
        cfg.fps = args.fps

    # Mode: test = GUI + line overlay + packet logging; production = send full packet, no GUI
    if args.mode == "test":
        if args.comms is None:
            cfg.comms.method = "stdout"
        else:
            cfg.comms.method = args.comms
        gui = not getattr(args, "no_gui", False)
    elif args.mode == "production":
        if args.comms is not None:
            cfg.comms.method = args.comms
        # else: use cfg.comms.method from config (http in default/docker)
        gui = False  # production: no GUI
    else:
        if args.comms is not None:
            cfg.comms.method = args.comms
        gui = not getattr(args, "no_gui", False)
    # Override state machine HTTP ingest URL (host vs Docker / scripts).
    _http_url_override = (
        os.environ.get("STATE_MACHINE_INPUT_URL")
        or os.environ.get("PERCEPTION_HTTP_INPUTS_URL")
        or ""
    ).strip()
    if _http_url_override:
        cfg.comms.http_url = _http_url_override
    source = _parse_source(cfg.camera.source, cfg)
    sender = _make_sender(cfg.comms.method, cfg)
    regulator = LoopRegulator(target_hz=cfg.fps)
    settings_lock = threading.Lock()
    runtime_settings = {
        "centerline_lookahead_ratio": float(cfg.smoothing.centerline_lookahead_ratio),
    }
    settings_server = _start_runtime_settings_server(cfg, runtime_settings, settings_lock)

    state = PipelineState()
    state.path_mask_key = "red"
    log("path_mask_red", reason="tracking red tape")
    if isinstance(source, str) and Path(source).name in {"test_run.mp4", "test_video.mp4"}:
        state.path_mask_key = "red"
        log("path_mask_red", reason="test clip uses red line")

    backend = cfg.camera.backend
    if isinstance(source, str) and source != "rpicam":
        if source.startswith(("rtsp://", "http://", "https://", "udp://", "tcp://")):
            backend = "ffmpeg"
        else:
            backend = "gstreamer"
    try:
        if source == "rpicam":
            cam = RpicamVidCamera(
                width=cfg.camera.width,
                height=cfg.camera.height,
                fps=cfg.fps,
                camera_index=cfg.camera.webcam_index,
            )
        else:
            cam = OpenCVCamera(
                source=source,
                width=cfg.camera.width,
                height=cfg.camera.height,
                fps=cfg.fps,
                backend=backend,
                threaded=True,
                gstreamer_device=cfg.camera.gstreamer_device,
            )
    except RuntimeError as exc:
        raise SystemExit(f"Camera initialization failed: {exc}") from exc

    mode = args.mode or "default"
    log("perception_start", source=cfg.camera.source, fps=cfg.fps, comms=cfg.comms.method, mode=mode)

    frame_queue: "queue.Queue[Optional[FrameItem]]" = queue.Queue(maxsize=2)
    result_queue: "queue.Queue[Optional[PerceptionResult]]" = queue.Queue(maxsize=2)
    stop_event = threading.Event()

    def capture_loop() -> None:
        try:
            while not stop_event.is_set():
                frame = cam.read()
                if frame is None:
                    log("stream_end_or_read_fail")
                    break
                ts = time.time()
                try:
                    frame_queue.put(FrameItem(timestamp=ts, frame=frame), timeout=0.5)
                except queue.Full:
                    # Drop frames to keep latency bounded.
                    continue
        finally:
            try:
                frame_queue.put_nowait(None)
            except queue.Full:
                pass

    def worker_loop() -> None:
        try:
            while not stop_event.is_set():
                try:
                    item = frame_queue.get(timeout=0.5)
                except queue.Empty:
                    continue
                if item is None:
                    break

                roi = crop_roi(item.frame, cfg.roi_y_start)
                with settings_lock:
                    cfg.smoothing.centerline_lookahead_ratio = float(runtime_settings["centerline_lookahead_ratio"])
                out = run_pipeline(roi_bgr=roi, state=state, cfg=cfg)
                try:
                    result_queue.put(
                        PerceptionResult(timestamp=item.timestamp, output=out, roi=roi),
                        timeout=0.5,
                    )
                except queue.Full:
                    continue
        finally:
            try:
                result_queue.put_nowait(None)
            except queue.Full:
                pass

    capture_thread = threading.Thread(target=capture_loop, name="perception_capture", daemon=True)
    worker_thread = threading.Thread(target=worker_loop, name="perception_worker", daemon=True)
    capture_thread.start()
    worker_thread.start()

    frame_count = 0
    fps_window_start = time.time()

    try:
        while True:
            try:
                result = result_queue.get(timeout=1.0)
            except queue.Empty:
                if stop_event.is_set():
                    break
                continue
            if result is None:
                break

            out = result.output
            frame_count += 1

            now = time.time()
            window_dt = now - fps_window_start
            if window_dt >= 1.0:
                current_fps = frame_count / window_dt if window_dt > 0 else 0.0
                log(
                    "perception_fps",
                    fps=current_fps,
                    target_fps=cfg.fps,
                    window_s=window_dt,
                    frames=frame_count,
                )
                fps_window_start = now
                frame_count = 0

            # Robot frame: X+ right, Y+ forward; clamp so sqrt(px^2+py^2) <= 1 (max speed)
            px_out, py_out = to_robot_frame_clamped(out.px, out.py)
            pkt = PerceptionPacket(
                px=px_out,
                py=py_out,
                line_error_x=float(out.line_error_x),
                line_error_y=float(py_out),
                zone=out.zone,
                gamma=out.gamma,
                t=result.timestamp,
                path_detected=out.path_detected,
                path_mask_key=out.path_mask_key,
                target_detected=out.target_detected,
                target_px=out.target_px,
                target_py=out.target_py,
            )
            line = pkt.to_json(zone_encoding=cfg.comms.zone_encoding)
            if sender is None:
                print(line, flush=True)
            else:
                sender.send_line(line)

            if gui:
                overlay = draw_overlay(result.roi, state.p_prev, out.zone, out.gamma)
                cv2.imshow("perception_roi", overlay)
                if cfg.show_masks and "masks" in out.debug_artifacts:
                    cv2.imshow("perception_masks", make_mask_preview(out.debug_artifacts["masks"]))
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

            regulator.sleep()
    finally:
        stop_event.set()
        settings_server.shutdown()
        settings_server.server_close()
        capture_thread.join(timeout=1.0)
        worker_thread.join(timeout=1.0)
        cam.release()
        if cfg.comms.method == "http" and cfg.comms.http_url:
            _send_zero_http_inputs(cfg.comms.http_url)
        if sender is not None:
            sender.close()
        cv2.destroyAllWindows()
        log("perception_stop")


if __name__ == "__main__":
    main()
