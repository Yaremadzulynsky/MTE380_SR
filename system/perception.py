"""Camera perception: red line, blue target, and T-junction detection."""
from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import subprocess
import threading
import time
from typing import Optional

import cv2
import numpy as np

import config as _config_module
from config import Config

log = logging.getLogger(__name__)

_JPEG_SOI = bytes([0xFF, 0xD8])
_JPEG_EOI = bytes([0xFF, 0xD9])


class _RpicamVidCamera:
    """Pi Camera via rpicam-vid subprocess (MJPEG to stdout). No Picamera2/libcamera lock."""

    def __init__(self, width: int = 640, height: int = 480, fps: float = 30.0) -> None:
        self._lock   = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._stop   = False
        self._eof    = False
        self._buffer = bytearray()

        cmd = [
            "rpicam-vid", "-t", "0", "-o", "-",
            "--codec", "mjpeg", "-n",
            "--width", str(width), "--height", str(height),
            "--framerate", str(int(fps)),
        ]
        try:
            self._proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                          stderr=subprocess.PIPE, bufsize=0)
        except FileNotFoundError as e:
            raise RuntimeError("rpicam-vid not found — install rpicam-apps") from e

        threading.Thread(target=self._log_stderr, daemon=True).start()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _log_stderr(self) -> None:
        assert self._proc.stderr
        for line in self._proc.stderr:
            log.warning("rpicam-vid: %s", line.decode(errors="replace").rstrip())

    def _reader(self) -> None:
        assert self._proc.stdout
        while not self._stop:
            chunk = self._proc.stdout.read(65536)
            if not chunk:
                self._eof = True
                break
            with self._lock:
                self._buffer.extend(chunk)
            self._drain()

    def _drain(self) -> None:
        while True:
            with self._lock:
                data = bytes(self._buffer)
            start = data.find(_JPEG_SOI)
            if start < 0:
                with self._lock:
                    self._buffer.clear()
                break
            end = data.find(_JPEG_EOI, start)
            if end < 0:
                if start > 0:
                    with self._lock:
                        del self._buffer[:start]
                break
            end += 2
            jpeg = data[start:end]
            with self._lock:
                del self._buffer[:end]
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                with self._lock:
                    self._latest = frame

    def read(self) -> Optional[np.ndarray]:
        deadline = time.perf_counter() + 2.0
        while self._latest is None and not self._eof and time.perf_counter() < deadline:
            time.sleep(0.005)
        with self._lock:
            return self._latest

    def release(self) -> None:
        self._stop = True
        self._thread.join(timeout=1.0)
        self._proc.terminate()
        try:
            self._proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._proc.kill()


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _focal_px(width: int, horizontal_fov_deg: float) -> float:
    """Pinhole focal length in pixels from horizontal field of view."""
    half = math.radians(horizontal_fov_deg) / 2.0
    return (width / 2.0) / math.tan(half)


def ground_xz_from_pixel(
    u: float,
    v: float,
    *,
    camera_height_m: float,
    pitch_deg: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[float, float] | None:
    """
    Ray through pixel (u,v) intersected with ground plane Y=0.

    World: X right, Y up, Z forward (robot forward). Camera at (0, camera_height_m, 0)
    looking ~forward and down; pitch_deg is depression below horizontal (optical axis).

    Returns (X, Z) in metres on the ground, or None if the ray is degenerate.
    """
    pitch = math.radians(pitch_deg)
    s, c = math.sin(pitch), math.cos(pitch)
    # Columns = camera basis vectors expressed in world (OpenCV cam: +X right, +Y down, +Z forward).
    R = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=np.float64,
    )
    xn = (u - cx) / fx
    yn = (v - cy) / fy
    d_cam = np.array([xn, yn, 1.0], dtype=np.float64)
    n = float(np.linalg.norm(d_cam))
    if n < 1e-12:
        return None
    d_cam /= n
    d_w = R @ d_cam
    if abs(d_w[1]) < 1e-9:
        return None
    # Ray: (0, h, 0) + t * d_w, intersect Y=0  =>  h + t * d_w[1] = 0
    t = -camera_height_m / d_w[1]
    if t <= 0.0:
        return None
    p = np.array([0.0, camera_height_m, 0.0], dtype=np.float64) + t * d_w
    return float(p[0]), float(p[2])


def lateral_error_normalized(x_m: float, lateral_norm_m: float) -> float:
    """Map lateral offset (m) to [-1, 1] for steering PID (scale ≈ half track width)."""
    if lateral_norm_m <= 1e-9:
        return 0.0
    return _clamp(x_m / lateral_norm_m, -1.0, 1.0)


# Measured on robot — change in code if you remount the camera.
_CAMERA_HEIGHT_M = 0.095
_CAMERA_PITCH_DEG = 39.0
_CAMERA_HORIZONTAL_FOV_DEG = 62.2

# Fixed detection constants (not exposed in config).
_RED_MIN_AREA = 80.0
_BLUE_MIN_AREA = 1500.0
_T_JUNCTION_WIDTH_RATIO = 0.5


@dataclass
class FrameDetection:
    red_found:      bool        # red line detected in ROI
    red_error:      float       # [-1, 1], positive = line is right of centre
    blue_found:     bool        # blue target marker visible
    t_junction:     bool        # wide red stripe = end T-junction
    curve_detected: bool = False  # line curves significantly (top/bottom centroid offset)
    # Pixel-space debug info (full-frame coords); None when line not detected
    red_cx:   int | None = None   # centroid x
    red_cy:   int | None = None   # centroid y
    red_bbox: tuple | None = None # (x, y, w, h) bounding rect


class Perception:
    """
    Reads one camera frame and returns a FrameDetection.

    Red line : HSV mask → largest contour → lateral error (image or ground-plane model)
    Blue target : HSV mask → any contour above min area
    T-junction  : red bounding-rect width > half the frame = horizontal end bar

    When ``geom_enable`` is True, ``red_error`` uses a ground-plane ray (camera pose is
    fixed in code; tune only ``geom_lateral_norm_m`` in config to map metres → ±1).

    ``red_loss_debounce_frames``: require this many consecutive raw misses before reporting
    ``red_found=False`` (holds last smoothed error meanwhile).

    ``red_error_ema_alpha``: low-pass on ``red_error`` when raw red is visible (0 = off).
    """

    def __init__(self, cfg: Config | None = None) -> None:
        c = cfg if cfg is not None else _config_module.get()

        self.width  = c.camera_width
        self.height = c.camera_height
        self.roi_top_ratio = _clamp(c.roi_top_ratio, 0.0, 0.95)

        self._update_hsv_ranges(c)

        self.geom_enable = c.geom_enable
        self.geom_lateral_norm_m = float(c.geom_lateral_norm_m)
        self.red_loss_debounce_frames = max(1, int(c.red_loss_debounce_frames))
        self.red_error_ema_alpha = _clamp(float(c.red_error_ema_alpha), 0.0, 1.0)

        # Raw-red debounce + EMA (see _stabilize_red)
        self._red_miss_streak = 0
        self._tracking_line = False
        self._red_error_ema = 0.0
        self._red_ema_seeded = False
        self._last_t_junction = False
        self._last_frame: np.ndarray | None = None
        self._last_red_mask: np.ndarray | None = None
        self._last_red_cx:   int | None = None
        self._last_red_cy:   int | None = None
        self._last_red_bbox: tuple | None = None

        self._fx = _focal_px(self.width, _CAMERA_HORIZONTAL_FOV_DEG)
        self._fy = self._fx  # square pixels
        self._cx = self.width / 2.0
        self._cy = self.height / 2.0

        self._cam = _RpicamVidCamera(self.width, self.height, c.fps)
        log.info("rpicam-vid camera opened (%dx%d @ %gfps)", self.width, self.height, c.fps)

    def reconfigure(self, cfg: Config) -> None:
        """Apply updated config values without reopening the camera."""
        self.geom_enable              = bool(cfg.geom_enable)
        self.geom_lateral_norm_m      = float(cfg.geom_lateral_norm_m)
        self.red_loss_debounce_frames = max(1, int(cfg.red_loss_debounce_frames))
        self.red_error_ema_alpha      = _clamp(float(cfg.red_error_ema_alpha), 0.0, 1.0)
        self.roi_top_ratio            = _clamp(cfg.roi_top_ratio, 0.0, 0.95)
        self._update_hsv_ranges(cfg)

    def _update_hsv_ranges(self, cfg: Config) -> None:
        """Rebuild HSV mask arrays from config."""
        self._RED_LO1 = np.array([cfg.red_h_lo1, cfg.red_s_min, cfg.red_v_min], np.uint8)
        self._RED_HI1 = np.array([cfg.red_h_hi1, 255,           255          ], np.uint8)
        self._RED_LO2 = np.array([cfg.red_h_lo2, cfg.red_s_min, cfg.red_v_min], np.uint8)
        self._RED_HI2 = np.array([cfg.red_h_hi2, 255,           255          ], np.uint8)
        self._BLUE_LO = np.array([cfg.blue_h_lo, cfg.blue_s_min, cfg.blue_v_min], np.uint8)
        self._BLUE_HI = np.array([cfg.blue_h_hi, 255,            255          ], np.uint8)
        self.red_min_area           = _RED_MIN_AREA
        self.blue_min_area          = _BLUE_MIN_AREA
        self.t_junction_width_ratio = _T_JUNCTION_WIDTH_RATIO
        self.curve_threshold        = float(cfg.curve_threshold)

    # ── Public ────────────────────────────────────────────────────────────────

    def read_frame(self) -> np.ndarray | None:
        frame = self._cam.read()
        self._last_frame = frame
        return frame

    def get_frame(self) -> np.ndarray | None:
        """Return the most recently captured raw frame (BGR). Thread-safe read."""
        return self._last_frame

    def detect(self, frame: np.ndarray) -> FrameDetection:
        h, w = frame.shape[:2]
        roi_y = int(h * self.roi_top_ratio)
        roi   = frame[roi_y:, :]
        hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        raw_found, raw_err, raw_tj, raw_curve = self._detect_red(hsv, w, roi_y)
        red_found, red_error, t_junction = self._stabilize_red(raw_found, raw_err, raw_tj)
        blue_found = self._detect_blue(hsv)

        return FrameDetection(
            red_found=red_found,
            red_error=red_error,
            blue_found=blue_found,
            t_junction=t_junction,
            curve_detected=raw_curve if red_found else False,
            red_cx=self._last_red_cx   if red_found else None,
            red_cy=self._last_red_cy   if red_found else None,
            red_bbox=self._last_red_bbox if red_found else None,
        )

    def get_red_mask_frame(self) -> np.ndarray | None:
        """Return the most recent red HSV mask as a BGR image for web streaming."""
        mask = self._last_red_mask
        if mask is None:
            return None
        return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    def release(self) -> None:
        self._cam.release()

    def draw_debug(self, frame: np.ndarray, det: FrameDetection) -> np.ndarray:
        out = frame.copy()
        h, w = out.shape[:2]
        roi_y = int(h * self.roi_top_ratio)
        cx_frame = w // 2

        # ── Semi-transparent red mask overlay ─────────────────────────────────
        mask = self._last_red_mask
        if mask is not None:
            overlay = out.copy()
            # Expand ROI mask to full frame height for indexing
            full_mask = np.zeros((h, w), np.uint8)
            full_mask[roi_y:, :] = mask
            overlay[full_mask > 0] = (0, 0, 180)
            cv2.addWeighted(overlay, 0.35, out, 0.65, 0, out)

        # ── ROI boundary + vertical centre line ───────────────────────────────
        if roi_y > 0:
            cv2.line(out, (0, roi_y), (w - 1, roi_y), (200, 200, 200), 1)
        cv2.line(out, (cx_frame, roi_y), (cx_frame, h - 1), (255, 255, 255), 2)

        # ── Line detection overlays ────────────────────────────────────────────
        if det.red_found:
            # Bounding box
            if det.red_bbox is not None:
                bx, by, bw, bh = det.red_bbox
                color = (0, 100, 255) if det.t_junction else (0, 255, 0)
                cv2.rectangle(out, (bx, by), (bx + bw, by + bh), color, 2)

            # Centroid + error arrow
            if det.red_cx is not None and det.red_cy is not None:
                cx, cy = det.red_cx, det.red_cy
                # Horizontal error arrow from frame centre to centroid
                cv2.arrowedLine(out, (cx_frame, cy), (cx, cy),
                                (0, 140, 255), 2, tipLength=0.2)
                # Centroid dot
                cv2.circle(out, (cx, cy), 7, (0, 0, 255), -1)
                cv2.circle(out, (cx, cy), 7, (255, 255, 255), 1)

        # ── Lateral error bar (bottom of frame) ───────────────────────────────
        bar_y  = h - 18
        bar_x0 = w // 4
        bar_x1 = 3 * w // 4
        bar_w  = bar_x1 - bar_x0
        # Track
        cv2.line(out, (bar_x0, bar_y), (bar_x1, bar_y), (70, 70, 70), 3)
        # Centre tick
        cv2.line(out, (cx_frame, bar_y - 5), (cx_frame, bar_y + 5), (180, 180, 180), 2)
        # Error indicator
        err_x = int(cx_frame + det.red_error * (bar_w // 2))
        err_x = max(bar_x0, min(bar_x1, err_x))
        dot_color = (0, 220, 0) if det.red_found else (80, 80, 80)
        cv2.circle(out, (err_x, bar_y), 7, dot_color, -1)
        cv2.circle(out, (err_x, bar_y), 7, (255, 255, 255), 1)

        # ── Curve detected border ─────────────────────────────────────────────
        if det.curve_detected:
            cv2.rectangle(out, (0, 0), (w - 1, h - 1), (0, 200, 255), 4)

        # ── Text HUD ──────────────────────────────────────────────────────────
        tags = [
            f"red={'Y' if det.red_found else 'N'}  err={det.red_error:+.2f}",
            f"blue={'Y' if det.blue_found else 'N'}  T={'Y' if det.t_junction else 'N'}  curve={'Y' if det.curve_detected else 'N'}",
        ]
        for i, tag in enumerate(tags):
            cv2.putText(out, tag, (10, 28 + i * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, cv2.LINE_AA)
        return out

    # ── Internals ─────────────────────────────────────────────────────────────

    def _stabilize_red(
        self, raw_found: bool, raw_err: float, raw_t_junction: bool
    ) -> tuple[bool, float, bool]:
        """
        Debounce loss: brief raw misses keep red_found True and hold last smoothed error.

        EMA: when raw red is visible, low-pass red_error to reduce frame-to-frame jitter.
        """
        a = self.red_error_ema_alpha

        if raw_found:
            self._red_miss_streak = 0
            self._tracking_line = True

            if a <= 1e-12:
                smooth = raw_err
            elif not self._red_ema_seeded:
                smooth = raw_err
                self._red_ema_seeded = True
            else:
                smooth = a * raw_err + (1.0 - a) * self._red_error_ema

            self._red_error_ema = smooth
            self._last_t_junction = raw_t_junction
            return True, smooth, raw_t_junction

        # Raw miss
        if not self._tracking_line:
            self._red_miss_streak = 0
            return False, 0.0, False

        self._red_miss_streak += 1
        if self._red_miss_streak < self.red_loss_debounce_frames:
            # Hold line-follow semantics so the state machine does not enter coast/search
            return True, self._red_error_ema, self._last_t_junction

        # Confirmed lost
        self._tracking_line = False
        self._red_miss_streak = 0
        self._red_ema_seeded = False
        self._red_error_ema = 0.0
        self._last_t_junction = False
        return False, 0.0, False

    def _detect_red(
        self, hsv: np.ndarray, frame_w: int, roi_y: int
    ) -> tuple[bool, float, bool, bool]:
        """Returns (found, error, t_junction, curve_detected)."""
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self._RED_LO1, self._RED_HI1),
            cv2.inRange(hsv, self._RED_LO2, self._RED_HI2),
        )
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        self._last_red_mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._last_red_cx = self._last_red_cy = self._last_red_bbox = None
            return False, 0.0, False, False

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.red_min_area:
            self._last_red_cx = self._last_red_cy = self._last_red_bbox = None
            return False, 0.0, False, False

        bx, by, bw, bh = cv2.boundingRect(largest)
        self._last_red_bbox = (bx, roi_y + by, bw, bh)
        t_junction = (bw / frame_w) > self.t_junction_width_ratio

        moments = cv2.moments(largest)
        if moments["m00"] <= 1e-6:
            self._last_red_cx = bx + bw // 2
            self._last_red_cy = roi_y + by + bh // 2
            return True, 0.0, t_junction, False

        cx_roi = moments["m10"] / moments["m00"]
        cy_roi = moments["m01"] / moments["m00"]
        self._last_red_cx = int(cx_roi)
        self._last_red_cy = int(roi_y + cy_roi)

        # Bottom of blob → ground point nearer the wheels than the centroid
        pts = largest.reshape(-1, 2)
        v_max = float(np.max(pts[:, 1]))
        row = pts[pts[:, 1] >= v_max - 1.0]
        u_pix = float(np.mean(row[:, 0])) if len(row) > 0 else cx_roi
        v_pix = float(roi_y + v_max)

        image_err = _clamp(
            (cx_roi - frame_w / 2.0) / (frame_w / 2.0), -1.0, 1.0
        )

        # Curve detection: compare top-half vs bottom-half centroid X.
        # A straight line has roughly the same centroid X in both halves;
        # a curve shifts the top centroid left or right relative to the bottom.
        roi_h = hsv.shape[0]
        mid_y = roi_h / 2.0
        top_pts = pts[pts[:, 1] < mid_y]
        bot_pts = pts[pts[:, 1] >= mid_y]
        curve_detected = False
        if len(top_pts) >= 5 and len(bot_pts) >= 5:
            top_cx = float(np.mean(top_pts[:, 0]))
            bot_cx = float(np.mean(bot_pts[:, 0]))
            curve_detected = abs(top_cx - bot_cx) / frame_w > self.curve_threshold

        if not self.geom_enable:
            return True, image_err, t_junction, curve_detected

        g = ground_xz_from_pixel(
            u_pix,
            v_pix,
            camera_height_m=_CAMERA_HEIGHT_M,
            pitch_deg=_CAMERA_PITCH_DEG,
            fx=self._fx,
            fy=self._fy,
            cx=self._cx,
            cy=self._cy,
        )
        if g is None:
            return True, image_err, t_junction, curve_detected

        x_m, _ = g
        err = lateral_error_normalized(x_m, self.geom_lateral_norm_m)
        return True, err, t_junction, curve_detected

    def _detect_blue(self, hsv: np.ndarray) -> bool:
        """Returns True when a blue marker of sufficient area is visible."""
        mask      = cv2.inRange(hsv, self._BLUE_LO, self._BLUE_HI)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(cv2.contourArea(c) >= self.blue_min_area for c in contours)
