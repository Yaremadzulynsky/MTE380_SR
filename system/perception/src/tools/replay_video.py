"""Replay a recorded ROI video through the perception pipeline."""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from src.comms.packet import PerceptionPacket
from src.config import load_config
from src.pipeline import PipelineState, run_pipeline
from src.utils.math2d import to_robot_frame_clamped
from src.vision.camera import OpenCVCamera
from src.vision.debug_draw import draw_overlay, make_mask_preview


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay recorded ROI video")
    parser.add_argument("video_path", help="Path to ROI video file")
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--path-mask", choices=["red", "blue", "black"], default=None)
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.path_mask is not None:
        cfg.path_mask_key = args.path_mask
    cam = OpenCVCamera(
        source=args.video_path,
        width=cfg.camera.width,
        height=cfg.camera.height,
        fps=cfg.fps,
        backend="gstreamer",
    )

    state = PipelineState(path_mask_key=cfg.path_mask_key)
    gui = not args.no_gui
    while True:
        roi = cam.read()
        if roi is None:
            break

        out = run_pipeline(roi, state, cfg)
        px_out, py_out = to_robot_frame_clamped(out.px, out.py)
        pkt = PerceptionPacket(
            px=px_out,
            py=py_out,
            zone=out.zone,
            gamma=out.gamma,
            t=time.time(),
            path_detected=out.path_detected,
            path_mask_key=out.path_mask_key,
        )
        zone, gamma, debug = out.zone, out.gamma, out.debug_artifacts
        print(pkt.to_json(zone_encoding=cfg.comms.zone_encoding))

        if gui:
            cv2.imshow(
                "replay_roi",
                draw_overlay(
                    roi,
                    np.array([out.px, out.py], dtype=float),
                    zone,
                    gamma,
                    path_detected=out.path_detected,
                    tangent_vector=np.asarray(
                        out.debug_artifacts.get("tangent_vector", np.zeros(2, dtype=float)),
                        dtype=float,
                    ),
                    probe_point=tuple(out.debug_artifacts.get("heading_debug", {}).get("probe_point", ())),
                    target_point=tuple(out.debug_artifacts.get("heading_debug", {}).get("target_point", ())),
                ),
            )
            if cfg.show_masks:
                cv2.imshow("replay_masks", make_mask_preview(debug["masks"]))
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
