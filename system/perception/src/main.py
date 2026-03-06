"""Main perception loop for Milestone 3.3."""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import cv2

from src.comms.packet import PerceptionPacket
from src.comms.serial_tx import SerialSender
from src.comms.udp_tx import UDPSender
from src.config import AppConfig, load_config
from src.pipeline import PipelineOutput, PipelineState, run_pipeline
from src.utils.logging import log
from src.utils.math2d import to_robot_frame_clamped
from src.utils.timing import LoopRegulator
from src.vision.camera import OpenCVCamera
from src.vision.debug_draw import draw_overlay, make_mask_preview
from src.vision.masks import crop_roi


def process_roi(
    roi_bgr: Any,
    state: PipelineState,
    cfg: AppConfig,
) -> tuple[float, float, str, float, bool, str, dict[str, Any]]:
    """
    Single pipeline function:
      ROI BGR -> (px, py, zone, gamma, path_detected, path_mask_key, debug_artifacts)
    """
    out: PipelineOutput = run_pipeline(roi_bgr=roi_bgr, state=state, cfg=cfg)
    return out.px, out.py, out.zone, out.gamma, out.path_detected, out.path_mask_key, out.debug_artifacts


def _make_sender(method: str, cfg: AppConfig) -> Any:
    if method == "udp":
        return UDPSender(cfg.comms.udp_ip, cfg.comms.udp_port)
    if method == "serial":
        return SerialSender(cfg.comms.serial_port, cfg.comms.serial_baud)
    if method == "stdout":
        return None
    raise ValueError(f"Unsupported comms method: {method}")


def _parse_source(source: str, cfg: AppConfig) -> int | str:
    if source == "webcam":
        return cfg.camera.webcam_index
    if source.startswith("video:"):
        return source.split("video:", 1)[1]
    raise ValueError("source must be webcam or video:/path/to/file")


def main() -> None:
    parser = argparse.ArgumentParser(description="Pi perception node (Milestone 3.3)")
    parser.add_argument("--config", default="configs/default.yaml", help="Path to YAML config")
    parser.add_argument(
        "--mode",
        choices=["test", "production"],
        default=None,
        help="test: GUI + heading arrow + packet logging (stdout). production: full packet over UDP/serial, no GUI",
    )
    parser.add_argument("--no-gui", action="store_true", help="Disable OpenCV debug windows")
    parser.add_argument("--source", default=None, help="webcam or video:/path/to/file")
    parser.add_argument("--comms", choices=["udp", "serial", "stdout"], default=None)
    parser.add_argument("--fps", type=float, default=None)
    args = parser.parse_args()

    cfg_path = Path(args.config)
    cfg = load_config(cfg_path)
    if args.source is not None:
        cfg.camera.source = args.source
    if args.fps is not None:
        cfg.fps = args.fps

    # Mode: test = GUI + blue line overlay + packet logging; production = send full packet, no GUI
    if args.mode == "test":
        if args.comms is None:
            cfg.comms.method = "stdout"
        else:
            cfg.comms.method = args.comms
        gui = not args.no_gui
    elif args.mode == "production":
        if args.comms is None:
            cfg.comms.method = "udp"
        else:
            cfg.comms.method = args.comms
        gui = False  # production: no GUI
    else:
        if args.comms is not None:
            cfg.comms.method = args.comms
        gui = not args.no_gui
    source = _parse_source(cfg.camera.source, cfg)
    sender = _make_sender(cfg.comms.method, cfg)
    regulator = LoopRegulator(target_hz=cfg.fps)

    state = PipelineState()
    if isinstance(source, str) and Path(source).name in {"test_run.mp4", "test_video.mp4"}:
        state.path_mask_key = "blue"
        log("path_mask_blue", reason="test clip uses blue line")

    try:
        cam = OpenCVCamera(
            source=source,
            width=cfg.camera.width,
            height=cfg.camera.height,
            fps=cfg.fps,
        )
    except RuntimeError as exc:
        raise SystemExit(f"Camera initialization failed: {exc}") from exc

    mode = args.mode or "default"
    log("perception_start", source=cfg.camera.source, fps=cfg.fps, comms=cfg.comms.method, mode=mode)

    try:
        while True:
            frame = cam.read()
            if frame is None:
                log("stream_end_or_read_fail")
                break

            roi = crop_roi(frame, cfg.roi_y_start)
            px, py, zone, gamma, path_detected, path_mask_key, debug = process_roi(roi, state, cfg)
            # Robot frame: X+ right, Y+ forward; clamp so sqrt(px^2+py^2) <= 1 (max speed)
            px_out, py_out = to_robot_frame_clamped(px, py)

            pkt = PerceptionPacket(
                px=px_out,
                py=py_out,
                zone=zone,
                gamma=gamma,
                t=time.time(),
                path_detected=path_detected,
                path_mask_key=path_mask_key,
            )
            line = pkt.to_json(zone_encoding=cfg.comms.zone_encoding)
            if sender is None:
                print(line)
            else:
                sender.send_line(line)

            if gui:
                overlay = draw_overlay(roi, state.p_prev, zone, gamma)
                cv2.imshow("perception_roi", overlay)
                if cfg.show_masks:
                    cv2.imshow("perception_masks", make_mask_preview(debug["masks"]))
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

            regulator.sleep()
    finally:
        cam.release()
        if sender is not None:
            sender.close()
        cv2.destroyAllWindows()
        log("perception_stop")


if __name__ == "__main__":
    main()
