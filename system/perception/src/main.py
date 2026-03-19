"""Main perception loop for Milestone 3.3."""

from __future__ import annotations

import argparse
import queue
import threading
import time
from dataclasses import dataclass
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
    if source.startswith("video:"):
        return source.split("video:", 1)[1]
    raise ValueError("source must be webcam, rpicam, or video:/path/to/file")


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
    source = _parse_source(cfg.camera.source, cfg)
    sender = _make_sender(cfg.comms.method, cfg)
    regulator = LoopRegulator(target_hz=cfg.fps)

    state = PipelineState()
    state.path_mask_key = "red"
    log("path_mask_red", reason="tracking red tape")
    if isinstance(source, str) and Path(source).name in {"test_run.mp4", "test_video.mp4"}:
        state.path_mask_key = "red"
        log("path_mask_red", reason="test clip uses red line")

    backend = "gstreamer" if isinstance(source, str) and source != "rpicam" else cfg.camera.backend
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
                line_error_y=1.0,
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
        capture_thread.join(timeout=1.0)
        worker_thread.join(timeout=1.0)
        cam.release()
        if sender is not None:
            sender.close()
        cv2.destroyAllWindows()
        log("perception_stop")


if __name__ == "__main__":
    main()
