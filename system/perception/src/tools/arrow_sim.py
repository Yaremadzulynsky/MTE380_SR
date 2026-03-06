"""Animated heading arrow simulator/visualizer."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Optional

if __package__ is None or __package__ == "":
    import sys

    sys.path.append(str(Path(__file__).resolve().parents[2]))

import cv2
import numpy as np

from src.config import AppConfig, load_config
from src.pipeline import PipelineState, run_pipeline
from src.utils.timing import LoopRegulator
from src.vision.camera import OpenCVCamera
from src.vision.masks import crop_roi


def _parse_source(source: str, cfg: AppConfig) -> int | str:
    if source == "webcam":
        return cfg.camera.webcam_index
    if source.startswith("video:"):
        return source.split("video:", 1)[1]
    raise ValueError("source must be webcam, video:/path, or synthetic")


def _resolve_video_path(raw_path: str) -> Path:
    """
    Resolve a video path from common working directories.

    Tries:
      1) as-provided absolute/relative path
      2) relative to project root (.../project)
      3) relative to perception root (.../project/perception)
    """
    p = Path(raw_path).expanduser()
    candidates = []

    if p.is_absolute():
        candidates.append(p)
    else:
        cwd = Path.cwd()
        project_root = Path(__file__).resolve().parents[3]
        perception_root = Path(__file__).resolve().parents[2]
        candidates.extend(
            [
                cwd / p,
                project_root / p,
                perception_root / p,
            ]
        )

    for c in candidates:
        if c.exists() and c.is_file():
            return c.resolve()

    attempted = ", ".join(str(c) for c in candidates)
    raise FileNotFoundError(
        f"Video file not found for '{raw_path}'. Attempted: {attempted}. "
        "Try an absolute path or 'video:perception/test_video.mp4'."
    )


def _draw_status(frame: np.ndarray, zone: str, gamma: float, fps_now: float) -> None:
    h, w = frame.shape[:2]
    cv2.putText(frame, f"zone: {zone}", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"gamma: {gamma:.2f}", (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    txt = f"fps: {fps_now:.1f}"
    size, _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    cv2.putText(
        frame,
        txt,
        (w - size[0] - 12, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )


def _print_profile(capture_ms: float, pre_ms: float, pipeline_ms: float, render_ms: float, total_ms: float) -> None:
    print(
        f"[profile] cap={capture_ms:.1f}ms pre={pre_ms:.1f}ms "
        f"pipe={pipeline_ms:.1f}ms draw={render_ms:.1f}ms total={total_ms:.1f}ms"
    )


def _draw_heading_arrow(frame: np.ndarray, px: float, py: float, gamma: float) -> None:
    h, w = frame.shape[:2]
    origin = (w // 2, h - 12)
    base_len = int(min(h, w) * 0.35)
    length = int(base_len * (0.3 + 0.7 * float(gamma)))
    length = max(30, length)

    # Keep the same convention as pipeline/main overlay:
    # px -> +x, py follows image coordinates (+down, -up).
    vx = float(px)
    vy = float(py)
    n = math.hypot(vx, vy)
    if n < 1e-9:
        vx, vy = 0.0, -1.0
    else:
        vx, vy = vx / n, vy / n

    tip = (int(origin[0] + vx * length), int(origin[1] + vy * length))
    cv2.arrowedLine(frame, origin, tip, (255, 0, 0), 3, tipLength=0.2)  # blue (BGR)
    steer = "STRAIGHT"
    if vx > 0.08:
        steer = "TURN RIGHT"
    elif vx < -0.08:
        steer = "TURN LEFT"
    cv2.putText(frame, steer, (12, frame.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)


def _make_small_mask_panel(masks: dict[str, np.ndarray], roi_shape: tuple[int, int, int]) -> np.ndarray:
    h, w = roi_shape[:2]
    tile_w = max(80, w // 6)
    tile_h = max(60, h // 6)

    def prep(name: str, color: tuple[int, int, int]) -> np.ndarray:
        tile = cv2.cvtColor(masks[name], cv2.COLOR_GRAY2BGR)
        tile = cv2.resize(tile, (tile_w, tile_h), interpolation=cv2.INTER_NEAREST)
        cv2.putText(tile, name.upper(), (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        return tile

    red = prep("red", (0, 0, 255))
    green = prep("green", (0, 255, 0))
    blue = prep("blue", (255, 0, 0))
    danger = prep("danger", (180, 180, 180))
    return np.vstack([np.hstack([red, green]), np.hstack([blue, danger])])


def _overlay_mask_panel(frame: np.ndarray, panel: np.ndarray) -> None:
    ph, pw = panel.shape[:2]
    h, w = frame.shape[:2]
    x0 = max(0, w - pw - 10)
    y0 = max(0, h - ph - 10)
    roi = frame[y0 : y0 + ph, x0 : x0 + pw]
    cv2.addWeighted(panel, 0.85, roi, 0.15, 0.0, roi)
    cv2.rectangle(frame, (x0 - 1, y0 - 1), (x0 + pw + 1, y0 + ph + 1), (255, 255, 255), 1)


def _synthetic_frame(w: int, h: int, t_sec: float) -> np.ndarray:
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:, :] = (60, 170, 60)  # green background in BGR

    # Red path line with oscillating angle around vertical.
    angle = 0.5 * math.sin(0.7 * t_sec)
    c = (w // 2, h - 15)
    length = int(0.9 * h)
    dx = int(math.sin(angle) * length * 0.6)
    dy = int(math.cos(angle) * length)
    p1 = (c[0] - dx, c[1] - dy)
    p2 = (c[0] + dx, c[1] + dy)
    cv2.line(frame, p1, p2, (0, 0, 255), 12)

    # Periodic overlays to trigger DANGER/TARGET.
    phase = int(t_sec) % 12
    if 4 <= phase < 7:
        cv2.rectangle(frame, (w // 5, h // 3), (4 * w // 5, h - 10), (40, 40, 40), -1)
    elif 8 <= phase < 11:
        cv2.circle(frame, (w // 2, h // 2), min(h, w) // 8, (255, 0, 0), -1)
    return frame


def main() -> None:
    parser = argparse.ArgumentParser(description="Animated heading arrow simulation")
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--source", default="webcam", help="webcam | video:/path | synthetic")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--backend", choices=["auto", "gstreamer", "ffmpeg"], default="auto")
    parser.add_argument("--log-rate", type=float, default=0.0, help="JSON log rate in Hz (0 disables)")
    parser.add_argument("--proc-scale", type=float, default=1.0, help="ROI processing scale (0.25..1.0)")
    parser.add_argument("--display-scale", type=float, default=1.0, help="Display scale (0.25..1.0)")
    parser.add_argument("--profile", action="store_true", help="Print timing breakdowns")
    parser.add_argument("--no-masks", action="store_true")
    parser.add_argument("--save", default=None, help="Optional output mp4 path")
    args = parser.parse_args()

    cv2.setUseOptimized(True)
    proc_scale = max(0.25, min(1.0, float(args.proc_scale)))
    display_scale = max(0.25, min(1.0, float(args.display_scale)))
    cfg = load_config(args.config)
    cfg.fps = args.fps
    cfg.camera.source = args.source
    show_masks = not args.no_masks

    state = PipelineState()

    use_synth = args.source == "synthetic"
    cam: Optional[OpenCVCamera] = None
    if not use_synth:
        source = _parse_source(args.source, cfg)
        if isinstance(source, str):
            source = str(_resolve_video_path(source))
            if Path(source).name in {"test_video.mp4", "test_run.mp4"}:
                # These test clips use a blue line instead of red tape.
                state.path_mask_key = "blue"
        cam = OpenCVCamera(
            source=source,
            width=cfg.camera.width,
            height=cfg.camera.height,
            fps=cfg.fps,
            backend=args.backend,
            threaded=True,
        )

    regulator = LoopRegulator(target_hz=cfg.fps)
    writer: Optional[cv2.VideoWriter] = None
    last_t = time.perf_counter()
    last_log_t = 0.0
    log_period = 1.0 / args.log_rate if args.log_rate > 0 else 0.0
    fps_ema = cfg.fps
    frame_idx = 0
    cached_panel: Optional[np.ndarray] = None
    last_panel_t = 0.0
    profile_acc = {"cap": 0.0, "pre": 0.0, "pipe": 0.0, "draw": 0.0, "total": 0.0}
    profile_count = 0
    profile_last = time.perf_counter()

    try:
        while True:
            t_loop0 = time.perf_counter()

            t0 = time.perf_counter()
            if use_synth:
                roi = _synthetic_frame(cfg.camera.width, cfg.camera.height - cfg.roi_y_start, frame_idx / cfg.fps)
            else:
                frame = cam.read() if cam is not None else None
                if frame is None:
                    break
                roi = crop_roi(frame, cfg.roi_y_start)
            t1 = time.perf_counter()

            display_roi = roi
            if proc_scale < 0.999:
                roi_proc = cv2.resize(
                    roi,
                    (int(roi.shape[1] * proc_scale), int(roi.shape[0] * proc_scale)),
                    interpolation=cv2.INTER_AREA,
                )
            else:
                roi_proc = roi
            t2 = time.perf_counter()

            out = run_pipeline(roi_bgr=roi_proc, state=state, cfg=cfg)
            t3 = time.perf_counter()
            zone_conf = out.debug_artifacts.get("zone_debug", {}).get("zone_confidences", {})
            now_epoch = time.time()
            if log_period > 0.0 and (now_epoch - last_log_t) >= log_period:
                print(
                    json.dumps(
                        {
                            "px": out.px,
                            "py": out.py,
                            "zone": out.zone,
                            "gamma": out.gamma,
                            "t": now_epoch,
                            "conf_safe": float(zone_conf.get("SAFE", 0.0)),
                            "conf_path": float(zone_conf.get("PATH", 0.0)),
                            "conf_danger": float(zone_conf.get("DANGER", 0.0)),
                            "conf_target": float(zone_conf.get("TARGET", 0.0)),
                        },
                        separators=(",", ":"),
                    )
                )
                last_log_t = now_epoch

            canvas = display_roi.copy()
            _draw_heading_arrow(canvas, out.px, out.py, out.gamma)

            now = time.perf_counter()
            dt = max(1e-6, now - last_t)
            last_t = now
            fps_inst = 1.0 / dt
            fps_ema = 0.9 * fps_ema + 0.1 * fps_inst
            _draw_status(canvas, out.zone, out.gamma, fps_ema)

            if show_masks:
                now_perf = time.perf_counter()
                # Rebuild panel at a lower rate; overlays still update every frame.
                if cached_panel is None or (now_perf - last_panel_t) >= 0.1:
                    cached_panel = _make_small_mask_panel(out.debug_artifacts["masks"], roi_proc.shape)
                    last_panel_t = now_perf
                _overlay_mask_panel(canvas, cached_panel)

            canvas_show = canvas
            if display_scale < 0.999:
                canvas_show = cv2.resize(
                    canvas,
                    (int(canvas.shape[1] * display_scale), int(canvas.shape[0] * display_scale)),
                    interpolation=cv2.INTER_AREA,
                )
            cv2.imshow("arrow_sim", canvas_show)

            if args.save:
                if writer is None:
                    out_path = Path(args.save)
                    out_path.parent.mkdir(parents=True, exist_ok=True)
                    h, w = canvas.shape[:2]
                    writer = cv2.VideoWriter(
                        str(out_path),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        cfg.fps,
                        (w, h),
                    )
                writer.write(canvas)

            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break

            t4 = time.perf_counter()
            profile_acc["cap"] += (t1 - t0) * 1000.0
            profile_acc["pre"] += (t2 - t1) * 1000.0
            profile_acc["pipe"] += (t3 - t2) * 1000.0
            profile_acc["draw"] += (t4 - t3) * 1000.0
            profile_acc["total"] += (t4 - t_loop0) * 1000.0
            profile_count += 1
            if args.profile and (time.perf_counter() - profile_last) >= 1.0 and profile_count > 0:
                _print_profile(
                    capture_ms=profile_acc["cap"] / profile_count,
                    pre_ms=profile_acc["pre"] / profile_count,
                    pipeline_ms=profile_acc["pipe"] / profile_count,
                    render_ms=profile_acc["draw"] / profile_count,
                    total_ms=profile_acc["total"] / profile_count,
                )
                profile_acc = {"cap": 0.0, "pre": 0.0, "pipe": 0.0, "draw": 0.0, "total": 0.0}
                profile_count = 0
                profile_last = time.perf_counter()

            frame_idx += 1
            regulator.sleep()
    finally:
        if cam is not None:
            cam.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
