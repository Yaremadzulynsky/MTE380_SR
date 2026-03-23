"""Camera perception: red line, blue target, and T-junction detection."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

import cv2
import numpy as np
from picamera2 import Picamera2


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


# Measured on robot — change in code if you remount the camera (not in pid_config).
_CAMERA_HEIGHT_M = 0.095
_CAMERA_PITCH_DEG = 39.0
_CAMERA_HORIZONTAL_FOV_DEG = 62.2


@dataclass
class FrameDetection:
    red_found:  bool        # red line detected in ROI
    red_error:  float       # [-1, 1], positive = line is right of centre
    blue_found: bool        # blue target marker visible
    t_junction: bool        # wide red stripe = end T-junction


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

    # ── HSV colour ranges ─────────────────────────────────────────────────────
    # Red wraps at 0/179 in OpenCV H (0–180). Two bands: orange-red and magenta-red.
    # Previous S≥100, V≥70, H≤12 produced sparse masks on real tape (AWB/exposure).
    _RED_LO1 = np.array([  0,  50,  45], np.uint8)
    _RED_HI1 = np.array([ 20, 255, 255], np.uint8)
    _RED_LO2 = np.array([160,  50,  45], np.uint8)
    _RED_HI2 = np.array([179, 255, 255], np.uint8)

    # Blue tuned to be a bit more forgiving but still reject washed-out noise.
    _BLUE_LO = np.array([ 95, 120,  70], np.uint8)
    _BLUE_HI = np.array([135, 255, 255], np.uint8)

    def __init__(
        self,
        width:  int   = 640,
        height: int   = 480,
        roi_top_ratio: float = 0.0,   # 0 = full frame; else ignore top fraction (e.g. 0.5 = bottom half)
        red_min_area:  float = 50.0,
        blue_min_area: float = 1500.0,  # require a substantial blue patch to avoid false triggers
        t_junction_width_ratio: float = 0.5,  # red bbox > this fraction of frame = T-junction
        camera_channel_order: Literal["rgb", "bgr"] = "bgr",
        geom_enable: bool = True,
        geom_lateral_norm_m: float = 0.10,
        red_loss_debounce_frames: int = 4,
        red_error_ema_alpha: float = 0.35,
    ) -> None:
        self.width  = width
        self.height = height
        self.roi_top_ratio = _clamp(roi_top_ratio, 0.0, 0.95)
        self.red_min_area  = red_min_area
        self.blue_min_area = blue_min_area
        self.t_junction_width_ratio = t_junction_width_ratio
        # Picamera2 main stream is often labeled "RGB888" but the buffer is frequently BGR order
        # on Raspberry Pi; using COLOR_RGB2BGR then BGR2HSV swaps R/B and breaks hue detection.
        self._camera_channel_order = camera_channel_order

        self.geom_enable = geom_enable
        self.geom_lateral_norm_m = float(geom_lateral_norm_m)
        self.red_loss_debounce_frames = max(1, int(red_loss_debounce_frames))
        self.red_error_ema_alpha = _clamp(float(red_error_ema_alpha), 0.0, 1.0)

        # Raw-red debounce + EMA (see _stabilize_red)
        self._red_miss_streak = 0
        self._tracking_line = False
        self._red_error_ema = 0.0
        self._red_ema_seeded = False
        self._last_t_junction = False

        self._fx = _focal_px(width, _CAMERA_HORIZONTAL_FOV_DEG)
        self._fy = self._fx  # square pixels
        self._cx = width / 2.0
        self._cy = height / 2.0

        self._cam = Picamera2()
        cfg = self._cam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"},
            buffer_count=2,
        )
        self._cam.configure(cfg)
        self._cam.start()

    def configure_geometry(
        self,
        *,
        geom_enable: bool | None = None,
        geom_lateral_norm_m: float | None = None,
    ) -> None:
        """Update geometry toggles from pid_config without reopening the camera."""
        if geom_enable is not None:
            self.geom_enable = bool(geom_enable)
        if geom_lateral_norm_m is not None:
            self.geom_lateral_norm_m = float(geom_lateral_norm_m)

    def configure_red_stability(
        self,
        *,
        red_loss_debounce_frames: int | None = None,
        red_error_ema_alpha: float | None = None,
    ) -> None:
        if red_loss_debounce_frames is not None:
            self.red_loss_debounce_frames = max(1, int(red_loss_debounce_frames))
        if red_error_ema_alpha is not None:
            self.red_error_ema_alpha = _clamp(float(red_error_ema_alpha), 0.0, 1.0)

    # ── Public ────────────────────────────────────────────────────────────────

    def read_frame(self) -> np.ndarray | None:
        raw = self._cam.capture_array("main")
        if self._camera_channel_order == "rgb":
            return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(raw)

    def detect(self, frame: np.ndarray) -> FrameDetection:
        h, w = frame.shape[:2]
        roi_y = int(h * self.roi_top_ratio)
        roi   = frame[roi_y:, :]
        hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        raw_found, raw_err, raw_tj = self._detect_red(hsv, w, roi_y)
        red_found, red_error, t_junction = self._stabilize_red(raw_found, raw_err, raw_tj)
        blue_found = self._detect_blue(hsv)

        return FrameDetection(
            red_found=red_found,
            red_error=red_error,
            blue_found=blue_found,
            t_junction=t_junction,
        )

    def red_mask_full(self, frame: np.ndarray) -> np.ndarray:
        """
        Full-frame uint8 mask (0/255) using the same red HSV bands and morphology as
        ``_detect_red`` — for side-by-side debug display with the camera view.
        """
        h, w = frame.shape[:2]
        roi_y = int(h * self.roi_top_ratio)
        roi = frame[roi_y:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        m = self._build_red_mask(hsv)
        full = np.zeros((h, w), dtype=np.uint8)
        full[roi_y:, :] = m
        return full

    def release(self) -> None:
        self._cam.stop()
        self._cam.close()

    def draw_debug(self, frame: np.ndarray, det: FrameDetection) -> np.ndarray:
        out = frame.copy()
        h, w = out.shape[:2]
        roi_y = int(h * self.roi_top_ratio)

        # Full frame: one vertical centre line. With ROI: grey bar + centre line in ROI only.
        if roi_y <= 0:
            cv2.line(out, (w // 2, 0), (w // 2, h - 1), (255, 255, 255), 2)
        else:
            cv2.line(out, (0, roi_y), (w - 1, roi_y), (200, 200, 200), 1)
            cv2.line(out, (w // 2, roi_y), (w // 2, h - 1), (255, 255, 255), 2)

        tags = [
            f"red={'Y' if det.red_found else 'N'}  err={det.red_error:+.2f}",
            f"blue={'Y' if det.blue_found else 'N'}",
            f"T={'Y' if det.t_junction else 'N'}",
        ]
        for i, tag in enumerate(tags):
            cv2.putText(out, tag, (10, 30 + i * 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
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

    def _build_red_mask(self, hsv: np.ndarray) -> np.ndarray:
        """Binary mask after inRange + morphology — shared by _detect_red and red_mask_full."""
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self._RED_LO1, self._RED_HI1),
            cv2.inRange(hsv, self._RED_LO2, self._RED_HI2),
        )
        # Light open removes salt noise; heavy 5×5 open was shredding thin vertical tape.
        k3 = np.ones((3, 3), np.uint8)
        k5 = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5, iterations=2)
        return mask

    def _detect_red(
        self, hsv: np.ndarray, frame_w: int, roi_y: int
    ) -> tuple[bool, float, bool]:
        """Returns (found, error, t_junction)."""
        mask = self._build_red_mask(hsv)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 0.0, False

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.red_min_area:
            return False, 0.0, False

        _, _, bw, _ = cv2.boundingRect(largest)
        t_junction  = (bw / frame_w) > self.t_junction_width_ratio

        moments = cv2.moments(largest)
        if moments["m00"] <= 1e-6:
            return True, 0.0, t_junction

        cx_roi = moments["m10"] / moments["m00"]
        cy_roi = moments["m01"] / moments["m00"]

        # Bottom of blob → ground point nearer the wheels than the centroid
        pts = largest.reshape(-1, 2)
        v_max = float(np.max(pts[:, 1]))
        row = pts[pts[:, 1] >= v_max - 1.0]
        u_pix = float(np.mean(row[:, 0])) if len(row) > 0 else cx_roi
        v_pix = float(roi_y + v_max)

        image_err = _clamp(
            (cx_roi - frame_w / 2.0) / (frame_w / 2.0), -1.0, 1.0
        )

        if not self.geom_enable:
            return True, image_err, t_junction

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
            return True, image_err, t_junction

        x_m, _ = g
        err = lateral_error_normalized(x_m, self.geom_lateral_norm_m)
        return True, err, t_junction

    def _detect_blue(self, hsv: np.ndarray) -> bool:
        """Returns True when a blue marker of sufficient area is visible."""
        mask      = cv2.inRange(hsv, self._BLUE_LO, self._BLUE_HI)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(cv2.contourArea(c) >= self.blue_min_area for c in contours)
