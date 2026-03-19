"""Interactive HSV tuner with trackbars."""

from __future__ import annotations

import argparse

import cv2

from src.config import AppConfig, HSVRange, load_config
from src.vision.camera import OpenCVCamera
from src.vision.masks import build_masks, crop_roi, to_hsv


def _parse_source(source: str, cfg: AppConfig) -> int | str:
    if source == "webcam":
        return cfg.camera.webcam_index
    if source.startswith("video:"):
        return source.split("video:", 1)[1]
    raise ValueError("source must be webcam or video:/path")


def _noop(_: int) -> None:
    pass


def _make_range_trackbars(win: str, prefix: str, r: HSVRange) -> None:
    cv2.createTrackbar(f"{prefix}_h_lo", win, r.lo[0], 179, _noop)
    cv2.createTrackbar(f"{prefix}_s_lo", win, r.lo[1], 255, _noop)
    cv2.createTrackbar(f"{prefix}_v_lo", win, r.lo[2], 255, _noop)
    cv2.createTrackbar(f"{prefix}_h_hi", win, r.hi[0], 179, _noop)
    cv2.createTrackbar(f"{prefix}_s_hi", win, r.hi[1], 255, _noop)
    cv2.createTrackbar(f"{prefix}_v_hi", win, r.hi[2], 255, _noop)


def _read_range(win: str, prefix: str) -> HSVRange:
    return HSVRange(
        lo=(
            cv2.getTrackbarPos(f"{prefix}_h_lo", win),
            cv2.getTrackbarPos(f"{prefix}_s_lo", win),
            cv2.getTrackbarPos(f"{prefix}_v_lo", win),
        ),
        hi=(
            cv2.getTrackbarPos(f"{prefix}_h_hi", win),
            cv2.getTrackbarPos(f"{prefix}_s_hi", win),
            cv2.getTrackbarPos(f"{prefix}_v_hi", win),
        ),
    )


def _print_yaml(cfg: AppConfig) -> None:
    print("---")
    for name in ["red1", "red2", "green", "blue", "black", "danger"]:
        r = getattr(cfg, name)
        print(f"{name}:")
        print(f"  lo: [{r.lo[0]}, {r.lo[1]}, {r.lo[2]}]")
        print(f"  hi: [{r.hi[0]}, {r.hi[1]}, {r.hi[2]}]")
    print("morph:")
    print(f"  kernel_size: {cfg.morph.kernel_size}")
    print(f"  open_iters: {cfg.morph.open_iters}")
    print(f"  close_iters: {cfg.morph.close_iters}")


def main() -> None:
    parser = argparse.ArgumentParser(description="HSV tuner")
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--source", default=None, help="webcam or video:/path")
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

    ctrl_win = "hsv_controls"
    cv2.namedWindow(ctrl_win, cv2.WINDOW_NORMAL)
    _make_range_trackbars(ctrl_win, "red1", cfg.red1)
    _make_range_trackbars(ctrl_win, "red2", cfg.red2)
    _make_range_trackbars(ctrl_win, "green", cfg.green)
    _make_range_trackbars(ctrl_win, "blue", cfg.blue)
    _make_range_trackbars(ctrl_win, "black", cfg.black)
    _make_range_trackbars(ctrl_win, "danger", cfg.danger)

    try:
        while True:
            frame = cam.read()
            if frame is None:
                break
            roi = crop_roi(frame, cfg.roi_y_start, cfg.roi_y_start_ratio)
            _ = to_hsv(roi)  # Keeps this script explicit about the conversion stage.

            cfg.red1 = _read_range(ctrl_win, "red1")
            cfg.red2 = _read_range(ctrl_win, "red2")
            cfg.green = _read_range(ctrl_win, "green")
            cfg.blue = _read_range(ctrl_win, "blue")
            cfg.black = _read_range(ctrl_win, "black")
            cfg.danger = _read_range(ctrl_win, "danger")

            masks = build_masks(to_hsv(roi), cfg)
            cv2.imshow("roi", roi)
            cv2.imshow("red", masks["red"])
            cv2.imshow("green", masks["green"])
            cv2.imshow("blue", masks["blue"])
            cv2.imshow("black", masks["black"])
            cv2.imshow("danger", masks["danger"])

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                _print_yaml(cfg)
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
