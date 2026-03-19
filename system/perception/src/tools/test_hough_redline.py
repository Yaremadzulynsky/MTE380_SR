"""Local Linux test utility for Hough red-line tracking visualization."""

from __future__ import annotations

import argparse
import csv
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from src.config import AppConfig, load_config
from src.pipeline import PipelineState, run_pipeline
from src.vision.camera import OpenCVCamera
from src.vision.masks import crop_roi


class _FrameStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame_jpeg: bytes | None = None

    def update_bgr(self, frame_bgr: Any, *, jpeg_quality: int = 80) -> None:
        ok, encoded = cv2.imencode(
            ".jpg",
            frame_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(max(1, min(100, jpeg_quality)))],
        )
        if not ok:
            return
        payload = encoded.tobytes()
        with self._cond:
            self._frame_jpeg = payload
            self._cond.notify_all()

    def wait_latest(self, timeout_s: float = 1.0) -> bytes | None:
        with self._cond:
            if self._frame_jpeg is None:
                self._cond.wait(timeout=timeout_s)
            return self._frame_jpeg


class _MjpegHandler(BaseHTTPRequestHandler):
    server_version = "HoughMjpeg/1.0"

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            body = b"Hough stream available at /stream.mjpg\n"
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path != "/stream.mjpg":
            self.send_error(404, "Not found")
            return

        frame_store: _FrameStore = self.server.frame_store  # type: ignore[attr-defined]
        boundary = b"--frame\r\n"
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        try:
            while True:
                frame = frame_store.wait_latest(timeout_s=2.0)
                if frame is None:
                    continue
                self.wfile.write(boundary)
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            return

    def log_message(self, fmt: str, *args: Any) -> None:
        _ = (fmt, args)


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
    """
    Returns:
      path_vec: vector along the detected line direction (where to travel)
      return_vec: vector from robot origin to nearest point on selected line (get back to line)
      lateral_vec: purely horizontal correction vector with proportional magnitude
      origin_px: robot origin in image coordinates
      nearest_px: nearest point on selected line (or estimated from lateral error)
    """
    origin = (w // 2, h - 10)
    sel = debug.get("selected_segment")
    nearest_dbg = debug.get("nearest_point")

    # Fallbacks if no selected segment is available.
    if not (isinstance(sel, list) and len(sel) == 4):
        path_vec = _normalize(float(path_vec[0]), float(path_vec[1]))
        dx = _clamp(float(line_error_x), -1.0, 1.0) * (w / 2.0)
        lateral_vec = (_clamp(float(line_error_x), -1.0, 1.0), 0.0)
        # Recovery vector includes a forward component so it points back toward the path.
        return_vec = _normalize(dx, -0.7 * h)
        nearest = (int(origin[0] + dx), origin[1])
        return path_vec, return_vec, lateral_vec, origin, nearest

    x1, y1, x2, y2 = [float(v) for v in sel]

    # Use filtered path vector from pipeline to avoid introducing extra visual jitter.
    path_vec = _normalize(float(path_vec[0]), float(path_vec[1]))

    # Nearest point on selected segment to robot origin.
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

    # Return-to-line vector: shortest correction toward nearest point on line.
    return_vec = _normalize(nearest_x - ox, nearest_y - oy)
    # Lateral-only vector: proportional horizontal correction (no unit normalization).
    lateral_signed = _clamp(float(line_error_x), -1.0, 1.0)
    lateral_vec = (lateral_signed, 0.0)

    nearest = (int(round(nearest_x)), int(round(nearest_y)))
    return path_vec, return_vec, lateral_vec, origin, nearest


def _parse_source(raw: str, cfg: AppConfig, webcam_index: int | None = None) -> int | str:
    if raw == "webcam":
        return webcam_index if webcam_index is not None else cfg.camera.webcam_index
    if raw.startswith(("http://", "https://", "rtsp://")):
        return raw
    if raw.startswith("video:"):
        return raw.split("video:", 1)[1]
    p = Path(raw)
    if p.exists():
        return str(p)
    raise ValueError("source must be webcam, video:/path, http://ip:port/video, or an existing file path")


def _draw_full_overlay(
    roi: Any,
    debug: dict,
    path_vec: tuple[float, float],
    return_vec: tuple[float, float],
    lateral_vec: tuple[float, float],
    origin: tuple[int, int],
    nearest_point: tuple[int, int],
    line_error_x: float,
    heading_deg: float,
    path_detected: bool,
    gamma: float,
    fps: float,
    zone: str,
    err_raw: float,
    err_med: float,
    err_filt: float,
) -> Any:
    out = roi.copy()
    h, w = out.shape[:2]
    cx = w // 2

    # Vertical center reference line (dashed feel via thin gray)
    cv2.line(out, (cx, 0), (cx, h), (100, 100, 100), 1)
    lookahead_y = int(float(debug.get("lookahead_y", max(0, h - 1 - int(0.35 * h)))))
    cv2.line(out, (0, lookahead_y), (w, lookahead_y), (120, 120, 255), 1)

    # All candidate Hough segments (thin green)
    segments = debug.get("hough_segments", [])
    for seg in segments:
        x1, y1, x2, y2 = int(seg[0]), int(seg[1]), int(seg[2]), int(seg[3])
        cv2.line(out, (x1, y1), (x2, y2), (0, 200, 0), 1)

    # Centerline points estimated from mask (white polyline).
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

    # Guidance vectors from robot origin (bottom center)
    arrow_len = int(min(h, w) * 0.35)
    ox, oy = origin
    cv2.circle(out, (ox, oy), 4, (255, 255, 255), -1)

    # 1) Path direction along the line (cyan)
    ptx = int(ox + path_vec[0] * arrow_len)
    pty = int(oy + path_vec[1] * arrow_len)
    cv2.arrowedLine(out, (ox, oy), (ptx, pty), (255, 255, 0), 3, tipLength=0.24)

    # 2) Return-to-line vector (yellow)
    rtx = int(ox + return_vec[0] * int(arrow_len * 0.85))
    rty = int(oy + return_vec[1] * int(arrow_len * 0.85))
    cv2.arrowedLine(out, (ox, oy), (rtx, rty), (0, 255, 255), 3, tipLength=0.24)

    # 3) Lateral-only correction vector (magenta), proportional to |line_error_x|.
    lateral_len = int(arrow_len * 0.75)
    ltx = int(ox + _clamp(float(lateral_vec[0]), -1.0, 1.0) * lateral_len)
    lty = oy
    cv2.arrowedLine(out, (ox, oy), (ltx, lty), (255, 0, 255), 3, tipLength=0.24)

    # Draw nearest point used for correction and connector.
    cv2.circle(out, nearest_point, 5, (0, 255, 255), -1)
    cv2.line(out, (ox, oy), nearest_point, (0, 255, 255), 1)

    # Lateral error bar at bottom
    bar_y = h - 4
    bar_half = w // 2
    err_px = int(line_error_x * bar_half)
    bar_color = (0, 255, 0) if abs(line_error_x) < 0.15 else (0, 165, 255) if abs(line_error_x) < 0.4 else (0, 0, 255)
    cv2.rectangle(out, (cx, bar_y - 6), (cx + err_px, bar_y), bar_color, -1)
    cv2.line(out, (cx, bar_y - 8), (cx, bar_y + 2), (255, 255, 255), 1)

    # Steering label
    if abs(heading_deg) < 5:
        steer = "STRAIGHT"
        steer_color = (0, 255, 0)
    elif heading_deg > 0:
        steer = f"RIGHT {abs(heading_deg):.0f} deg"
        steer_color = (0, 165, 255)
    else:
        steer = f"LEFT {abs(heading_deg):.0f} deg"
        steer_color = (0, 165, 255)

    # Text HUD (top area)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(out, f"FPS: {fps:.1f}  Zone: {zone}", (8, 18), font, 0.5, (255, 255, 255), 1)
    cv2.putText(out, f"Detected: {path_detected}  Segments: {len(segments)}", (8, 38), font, 0.5, (255, 255, 255), 1)
    cv2.putText(
        out,
        f"PathVec: ({path_vec[0]:+.3f}, {path_vec[1]:+.3f})  Angle: {heading_deg:+.1f} deg",
        (8, 58),
        font,
        0.5,
        (255, 255, 0),
        1,
    )
    cv2.putText(out, f"Lateral Error(filt): {line_error_x:+.3f}  Gamma: {gamma:.2f}", (8, 78), font, 0.5, (0, 255, 255), 1)
    cv2.putText(out, f"Nearest source: {debug.get('nearest_source', 'na')}", (8, 98), font, 0.5, (200, 220, 255), 1)
    cv2.putText(out, f"ReturnVec: ({return_vec[0]:+.3f}, {return_vec[1]:+.3f})", (8, 118), font, 0.5, (0, 255, 255), 1)
    cv2.putText(out, f"LateralVec: ({lateral_vec[0]:+.3f}, {lateral_vec[1]:+.3f})", (8, 138), font, 0.5, (255, 0, 255), 1)
    cv2.putText(out, f"Err raw/med/filt: {err_raw:+.3f} / {err_med:+.3f} / {err_filt:+.3f}", (8, 158), font, 0.5, (200, 255, 200), 1)
    cv2.putText(out, steer, (8, h - 16), font, 0.7, steer_color, 2)

    # Legend (bottom-right)
    lx = w - 170
    cv2.putText(out, "-- candidates", (lx, h - 60), font, 0.4, (0, 200, 0), 1)
    cv2.putText(out, "-- selected", (lx, h - 44), font, 0.4, (0, 0, 255), 1)
    cv2.putText(out, "-> path vec", (lx, h - 28), font, 0.4, (255, 255, 0), 1)
    cv2.putText(out, "-> return vec", (lx, h - 12), font, 0.4, (0, 255, 255), 1)
    cv2.putText(out, "-> lateral vec", (lx, h - 76), font, 0.4, (255, 0, 255), 1)
    cv2.putText(out, "[] lat error", (lx, h - 92), font, 0.4, bar_color, 1)

    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Hough red-line local visualizer")
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--source", default="webcam", help="webcam | video:/path | /path/to/video | http://ip:port/video")
    parser.add_argument("--webcam-index", type=int, default=None, help="Override camera.webcam_index (e.g. 10 for virtual cam)")
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--csv", default=None, help="Optional CSV output path for metrics")
    parser.add_argument(
        "--proc-scale",
        type=float,
        default=1.0,
        help="Processing scale for ROI (0.5 is faster, less precise)",
    )
    parser.add_argument(
        "--hide-masks",
        action="store_true",
        help="Hide red-mask and edge windows (faster)",
    )
    parser.add_argument(
        "--window-scale",
        type=float,
        default=1.0,
        help="Display scale for OpenCV windows (e.g. 1.5 or 2.0)",
    )
    parser.add_argument(
        "--fullscreen",
        action="store_true",
        help="Start overlay window in fullscreen mode",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Disable OpenCV windows (useful for headless stream mode).",
    )
    parser.add_argument(
        "--mjpeg-port",
        type=int,
        default=0,
        help="If >0, publish overlay stream at /stream.mjpg on this port.",
    )
    parser.add_argument(
        "--mjpeg-host",
        default="0.0.0.0",
        help="Bind host for MJPEG stream server.",
    )
    args = parser.parse_args()

    cv2.setUseOptimized(True)
    cfg = load_config(args.config)
    if args.fps is not None:
        cfg.fps = float(args.fps)
    proc_scale = max(0.25, min(1.0, float(args.proc_scale)))
    source = _parse_source(args.source, cfg, webcam_index=args.webcam_index)
    backend = "gstreamer" if isinstance(source, str) else cfg.camera.backend
    cam = OpenCVCamera(
        source=source,
        width=cfg.camera.width,
        height=cfg.camera.height,
        fps=cfg.fps,
        backend=backend,
        threaded=True,
        gstreamer_device=cfg.camera.gstreamer_device,
    )

    state = PipelineState(path_mask_key="red")
    window_scale = max(0.5, float(args.window_scale))
    fullscreen = bool(args.fullscreen)

    display_enabled = not bool(args.no_display)
    show_masks = display_enabled and not args.hide_masks
    if display_enabled:
        cv2.namedWindow("hough_test_overlay", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            "hough_test_overlay",
            cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL,
        )
        if show_masks:
            cv2.namedWindow("hough_test_red_mask", cv2.WINDOW_NORMAL)
            cv2.namedWindow("hough_test_edges", cv2.WINDOW_NORMAL)

    frame_store: _FrameStore | None = None
    mjpeg_server: ThreadingHTTPServer | None = None
    mjpeg_thread: threading.Thread | None = None
    if int(args.mjpeg_port) > 0:
        frame_store = _FrameStore()
        mjpeg_server = ThreadingHTTPServer((str(args.mjpeg_host), int(args.mjpeg_port)), _MjpegHandler)
        setattr(mjpeg_server, "frame_store", frame_store)
        mjpeg_thread = threading.Thread(target=mjpeg_server.serve_forever, daemon=True)
        mjpeg_thread.start()
        print(f"[hough_test] stream_url=http://{args.mjpeg_host}:{args.mjpeg_port}/stream.mjpg")

    csv_file = None
    csv_writer = None
    if args.csv:
        csv_path = Path(args.csv)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            [
                "t",
                "path_detected",
                "line_error_x",
                "heading_px",
                "heading_py",
                "heading_deg",
                "segment_count",
                "gamma",
            ]
        )

    last = time.perf_counter()
    fps_ema = 0.0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                break
            roi = crop_roi(frame, cfg.roi_y_start)
            if proc_scale < 0.999:
                roi_proc = cv2.resize(
                    roi,
                    None,
                    fx=proc_scale,
                    fy=proc_scale,
                    interpolation=cv2.INTER_AREA,
                )
            else:
                roi_proc = roi

            out = run_pipeline(roi_bgr=roi_proc, state=state, cfg=cfg)
            heading_deg = math.degrees(math.atan2(float(out.px), float(-out.py)))

            now = time.perf_counter()
            dt = max(1e-6, now - last)
            last = now
            fps_now = 1.0 / dt
            fps_ema = fps_now if fps_ema <= 0 else (0.9 * fps_ema + 0.1 * fps_now)

            hd = out.debug_artifacts.get("heading_debug", {})
            smoothing = out.debug_artifacts.get("smoothing_debug", {})
            line_error_for_vectors = float(smoothing.get("line_error_median", out.line_error_x))
            path_vec, return_vec, lateral_vec, origin, nearest_point = _compute_guidance_vectors(
                debug=hd,
                w=roi_proc.shape[1],
                h=roi_proc.shape[0],
                path_vec=(float(out.px), float(out.py)),
                line_error_x=line_error_for_vectors,
            )
            overlay = _draw_full_overlay(
                roi_proc,
                hd,
                path_vec=path_vec,
                return_vec=return_vec,
                lateral_vec=lateral_vec,
                origin=origin,
                nearest_point=nearest_point,
                line_error_x=float(out.line_error_x),
                heading_deg=heading_deg,
                path_detected=out.path_detected,
                gamma=float(out.gamma),
                fps=fps_ema,
                zone=out.zone,
                err_raw=float(smoothing.get("line_error_raw", out.line_error_x)),
                err_med=float(smoothing.get("line_error_median", out.line_error_x)),
                err_filt=float(smoothing.get("line_error_filtered", out.line_error_x)),
            )

            if show_masks:
                masks = out.debug_artifacts.get("masks", {})
                red_mask = masks.get("red")
                edges = hd.get("edges")
                if red_mask is not None:
                    if window_scale > 1.0:
                        red_mask_show = cv2.resize(
                            red_mask,
                            None,
                            fx=window_scale,
                            fy=window_scale,
                            interpolation=cv2.INTER_NEAREST,
                        )
                    else:
                        red_mask_show = red_mask
                    cv2.imshow("hough_test_red_mask", red_mask_show)
                if edges is not None:
                    if window_scale > 1.0:
                        edges_show = cv2.resize(
                            edges,
                            None,
                            fx=window_scale,
                            fy=window_scale,
                            interpolation=cv2.INTER_NEAREST,
                        )
                    else:
                        edges_show = edges
                    cv2.imshow("hough_test_edges", edges_show)

            if window_scale > 1.0:
                overlay_show = cv2.resize(
                    overlay,
                    None,
                    fx=window_scale,
                    fy=window_scale,
                    interpolation=cv2.INTER_LINEAR,
                )
            else:
                overlay_show = overlay
            if frame_store is not None:
                frame_store.update_bgr(overlay_show)
            if display_enabled:
                cv2.imshow("hough_test_overlay", overlay_show)

            if csv_writer is not None:
                csv_writer.writerow(
                    [
                        time.time(),
                        int(bool(out.path_detected)),
                        float(out.line_error_x),
                        float(out.px),
                        float(out.py),
                        float(heading_deg),
                        int(len(hd.get("hough_segments", []))),
                        float(out.gamma),
                    ]
                )

            if display_enabled:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("f"):
                    fullscreen = not fullscreen
                    cv2.setWindowProperty(
                        "hough_test_overlay",
                        cv2.WND_PROP_FULLSCREEN,
                        cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL,
                    )
    finally:
        cam.release()
        if csv_file is not None:
            csv_file.close()
        if mjpeg_server is not None:
            mjpeg_server.shutdown()
            mjpeg_server.server_close()
            if mjpeg_thread is not None:
                mjpeg_thread.join(timeout=1.0)
        if display_enabled:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
