"""Record ROI stream to an MP4 file for offline testing."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from src.config import AppConfig, load_config
from src.vision.camera import OpenCVCamera
from src.vision.masks import crop_roi


def _parse_source(source: str, cfg: AppConfig) -> int | str:
    if source == "webcam":
        return cfg.camera.webcam_index
    if source.startswith("video:"):
        return source.split("video:", 1)[1]
    raise ValueError("source must be webcam or video:/path")


def main() -> None:
    parser = argparse.ArgumentParser(description="Record ROI video")
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--source", default=None, help="webcam or video:/path")
    parser.add_argument("--output", default="recordings/roi_capture.mp4")
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.source:
        cfg.camera.source = args.source
    source = _parse_source(cfg.camera.source, cfg)

    cam = OpenCVCamera(
        source=source,
        width=cfg.camera.width,
        height=cfg.camera.height,
        fps=cfg.fps,
        backend=cfg.camera.backend,
        gstreamer_device=cfg.camera.gstreamer_device,
    )
    frame = cam.read()
    if frame is None:
        cam.release()
        raise SystemExit("Failed to read initial frame from source")
    roi = crop_roi(frame, cfg.roi_y_start, cfg.roi_y_start_ratio)
    h, w = roi.shape[:2]

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(out_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        cfg.fps,
        (w, h),
    )

    total_frames = int(max(1, args.seconds * cfg.fps))
    writer.write(roi)
    for _ in range(total_frames - 1):
        frame = cam.read()
        if frame is None:
            break
        roi = crop_roi(frame, cfg.roi_y_start, cfg.roi_y_start_ratio)
        writer.write(roi)
        cv2.imshow("record_roi", roi)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    writer.release()
    cam.release()
    cv2.destroyAllWindows()
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
