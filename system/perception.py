"""Camera perception: red line, blue target, and green stop-marker detection."""
from __future__ import annotations

from dataclasses import dataclass
import threading

import cv2
import numpy as np
from picamera2 import Picamera2

import config as _config_module
from config import Config


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _u_on_line_at_v(
    u_c: float,
    v_c: float,
    u_b: float,
    v_b: float,
    v_target: float,
) -> float:
    """Linear u(v) along centroid → blob bottom; returns u at v_target (full-frame v)."""
    dv = v_b - v_c
    if abs(dv) < 1e-6:
        return u_b
    return float(u_c + (u_b - u_c) * (v_target - v_c) / dv)


# Fixed detection constants (not exposed in config).
_BLUE_MIN_AREA  = 1500.0
_GREEN_MIN_AREA = 1500.0


@dataclass
class FrameDetection:
    red_found:   bool       # red line detected in ROI
    red_error:   float      # [-1, 1], positive = line is right of centre
    blue_found:  bool       # blue target marker visible
    green_found: bool       # green stop marker visible
    blue_cx_norm: float | None = None  # blue centroid x in [-1, 1], +ve = right; None if not found
    # Full-frame pixels for overlays (None when red not found / not yet tracked)
    tape_cx_px: float | None = None   # horizontal centre of tape (mask centroid)
    tape_cy_px: float | None = None   # vertical centre of mask centroid
    track_x_px: float | None = None   # lateral anchor: u used for red_error (extrapolated or blob bottom)
    track_y_px: float | None = None
    # Curvature from N-strip polynomial fit (only valid when red_found)
    curvature:  float = 0.0   # signed: +ve = curves right, -ve = left
    curve_heading: float = 0.0  # local heading slope at robot position (+ve = angled right)
    curve_conf: float = 0.0   # fraction of strips that had valid data [0, 1]
    curve_pts: list | None = None  # full-frame (x, y) of each strip centroid
    # Horizontal red pixel balance: (right_px - left_px) / total_px  in [-1, 1]
    # +1 = all red on right half, -1 = all red on left half, 0 = balanced / not found
    red_horiz_balance: float = 0.0
    red_blob_area: float = 0.0  # area of the largest red contour in pixels² (0 when not found)


class Perception:
    """
    Reads one camera frame and returns a FrameDetection.

    Red line    : HSV mask → largest contour → lateral error in image space
    Blue target : HSV mask → any contour above min area
    Green stop  : HSV mask → any contour above min area

    Lateral error uses the blob **bottom** as the near-wheel anchor. The mask **centroid**
    and bottom define a line in the image; ``line_axle_extrap`` extends that line downward
    (larger v) by ``line_axle_extrap * (v_bottom - v_centroid)`` so steering references a
    point closer to the wheelbase on curves. ``0`` = use the blob bottom only.

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

        self.red_loss_debounce_frames = max(1, int(c.red_loss_debounce_frames))
        self.red_error_ema_alpha = _clamp(float(c.red_error_ema_alpha), 0.0, 1.0)
        self._red_min_area            = max(1, int(c.red_min_area_px))
        self._red_mask_min_blob_px    = max(0, int(c.red_mask_min_blob_px))
        self._curve_n_strips = max(2, int(c.curve_n_strips))
        self._error_heading_weight   = float(c.error_heading_weight)
        self._error_curvature_weight = float(c.error_curvature_weight)
        self._camera_rotation_deg    = float(c.camera_rotation_deg)

        # Raw-red debounce + EMA (see _stabilize_red)
        self._red_miss_streak = 0
        self._tracking_line = False
        self._red_error_ema = 0.0
        self._red_ema_seeded = False
        self._last_tape_cx: float | None = None
        self._last_tape_cy: float | None = None
        self._last_track_x: float | None = None
        self._last_track_y: float | None = None

        self._last_frame:         np.ndarray | None = None
        self._last_red_mask:      np.ndarray | None = None
        self._last_red_blob_area: float = 0.0
        self._last_green_mask:  np.ndarray | None = None
        self._last_blue_mask:   np.ndarray | None = None
        self.mask_channel: str = "red"   # "red" | "green" | "blue"

        # Background capture — camera runs continuously; _capture_buf holds latest frame.
        self._capture_buf:  np.ndarray | None = None
        self._capture_lock  = threading.Lock()
        self._capture_event = threading.Event()
        self._capture_stop  = threading.Event()

        self._cam = Picamera2()
        frame_us = int(1_000_000 / max(c.fps, 1.0))
        cam_cfg = self._cam.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
            buffer_count=4,
            controls={"FrameDurationLimits": (frame_us, frame_us)},
        )
        self._cam.configure(cam_cfg)
        self._cam.start()

        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="cam-capture"
        )
        self._capture_thread.start()

    def reconfigure(self, cfg: Config) -> None:
        """Apply updated config values without reopening the camera."""
        self.red_loss_debounce_frames = max(1, int(cfg.red_loss_debounce_frames))
        self.red_error_ema_alpha      = _clamp(float(cfg.red_error_ema_alpha), 0.0, 1.0)
        self._red_min_area            = max(1, int(cfg.red_min_area_px))
        self._red_mask_min_blob_px    = max(0, int(cfg.red_mask_min_blob_px))
        self.roi_top_ratio            = _clamp(cfg.roi_top_ratio, 0.0, 0.95)
        self._curve_n_strips          = max(2, int(cfg.curve_n_strips))
        self._error_heading_weight    = float(cfg.error_heading_weight)
        self._error_curvature_weight  = float(cfg.error_curvature_weight)
        self._camera_rotation_deg     = float(cfg.camera_rotation_deg)
        self._update_hsv_ranges(cfg)

    def _update_hsv_ranges(self, cfg: Config) -> None:
        """Rebuild HSV mask arrays from config."""
        self._RED_LO1 = np.array([cfg.red_h_lo1, cfg.red_s_min, cfg.red_v_min], np.uint8)
        self._RED_HI1 = np.array([cfg.red_h_hi1, 255,           255          ], np.uint8)
        self._RED_LO2 = np.array([cfg.red_h_lo2, cfg.red_s_min, cfg.red_v_min], np.uint8)
        self._RED_HI2 = np.array([cfg.red_h_hi2, 255,           255          ], np.uint8)
        self._BLUE_LO  = np.array([cfg.blue_h_lo,  cfg.blue_s_min,  cfg.blue_v_min ], np.uint8)
        self._BLUE_HI  = np.array([cfg.blue_h_hi,  255,             255            ], np.uint8)
        self._GREEN_LO = np.array([cfg.green_h_lo, cfg.green_s_min, cfg.green_v_min], np.uint8)
        self._GREEN_HI = np.array([cfg.green_h_hi, 255,             255            ], np.uint8)
        self.line_axle_extrap = max(0.0, float(cfg.line_axle_extrap))

    # ── Public ────────────────────────────────────────────────────────────────

    def read_frame(self) -> np.ndarray | None:
        """Block until a new frame is available from the background capture thread."""
        self._capture_event.wait()
        self._capture_event.clear()
        with self._capture_lock:
            frame = self._capture_buf
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

        raw_found, raw_err, tcx, tcy, trx, try_ = self._detect_red(hsv, w, roi_y)
        (
            red_found,
            red_error,
            tape_cx_px,
            tape_cy_px,
            track_x_px,
            track_y_px,
        ) = self._stabilize_red(raw_found, raw_err, tcx, tcy, trx, try_)
        blue_found, blue_cx_norm = self._detect_blue(hsv)
        green_found = self._detect_green(hsv)

        if raw_found and self._last_red_mask is not None:
            curvature, curve_heading, curve_conf, curve_pts = self._detect_curvature(
                self._last_red_mask, w, roi_y
            )
            # Blend tangent (heading) and curvature into error for feedforward steering
            red_error = _clamp(
                red_error
                + self._error_heading_weight   * curve_heading
                + self._error_curvature_weight * curvature,
                -1.0, 1.0,
            )
            # Horizontal red pixel balance: (right - left) / total  in [-1, 1]
            mask = self._last_red_mask
            half = mask.shape[1] // 2
            left_px  = int(np.count_nonzero(mask[:, :half]))
            right_px = int(np.count_nonzero(mask[:, half:]))
            total_px = left_px + right_px
            red_horiz_balance = (right_px - left_px) / total_px if total_px > 0 else 0.0
        else:
            curvature = curve_heading = curve_conf = 0.0
            curve_pts = []
            red_horiz_balance = 0.0

        return FrameDetection(
            red_found=red_found,
            red_error=red_error,
            blue_found=blue_found,
            blue_cx_norm=blue_cx_norm,
            green_found=green_found,
            tape_cx_px=tape_cx_px,
            tape_cy_px=tape_cy_px,
            track_x_px=track_x_px,
            track_y_px=track_y_px,
            curvature=curvature,
            curve_heading=curve_heading,
            curve_conf=curve_conf,
            curve_pts=curve_pts if curve_pts else None,
            red_horiz_balance=red_horiz_balance,
            red_blob_area=self._last_red_blob_area,
        )

    def get_mask_frame(self) -> np.ndarray | None:
        """Return the most recent HSV mask for the active channel as a BGR image."""
        if self.mask_channel == "green":
            mask = self._last_green_mask
        elif self.mask_channel == "blue":
            mask = self._last_blue_mask
        else:
            mask = self._last_red_mask
        if mask is None:
            return None
        return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    def release(self) -> None:
        self._capture_stop.set()
        self._capture_thread.join(timeout=2.0)
        self._cam.stop()
        self._cam.close()

    def draw_debug(self, frame: np.ndarray, det: FrameDetection) -> np.ndarray:
        out = frame.copy()
        h, w = out.shape[:2]
        roi_y = int(h * self.roi_top_ratio)

        # Centre lines:
        #   white  = frame centre (vertical reference)
        #   yellow = tangent-adjusted zero-error line, tilted to match curve_heading
        #
        # curve_heading (b) = dx_norm/dt at t=0 (near end).
        # Over half the frame height, x shifts by ±b * half_w/2:
        #   top endpoint  (far,  t→1): adj_cx + b * half_w/2  (shifts right when b>0)
        #   bottom endpoint (near, t=0): adj_cx - b * half_w/2
        half_w_f = w / 2.0
        adj_cx_f = half_w_f - self._error_heading_weight * det.curve_heading * half_w_f
        dx_half  = det.curve_heading * half_w_f / 2.0
        y_top    = roi_y if roi_y > 0 else 0
        y_bot    = h - 1
        x_top    = int(round(adj_cx_f + dx_half))
        x_bot    = int(round(adj_cx_f - dx_half))
        x_top    = max(0, min(w - 1, x_top))
        x_bot    = max(0, min(w - 1, x_bot))

        if roi_y <= 0:
            cv2.line(out, (w // 2, 0), (w // 2, h - 1), (255, 255, 255), 1)
        else:
            cv2.line(out, (0, roi_y), (w - 1, roi_y), (200, 200, 200), 1)
            cv2.line(out, (w // 2, roi_y), (w // 2, h - 1), (255, 255, 255), 1)
        cv2.line(out, (x_top, y_top), (x_bot, y_bot), (0, 255, 255), 2)

        tags = [
            f"red={'Y' if det.red_found else 'N'}  err={det.red_error:+.2f}",
            f"blue={'Y' if det.blue_found else 'N'}  green={'Y' if det.green_found else 'N'}",
            f"curv={det.curvature:+.2f}  conf={det.curve_conf:.0%}",
        ]
        for i, tag in enumerate(tags):
            cv2.putText(out, tag, (10, 30 + i * 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)

        if det.curve_pts and len(det.curve_pts) >= 2:
            ipts = [(int(round(x)), int(round(y))) for x, y in det.curve_pts]
            for a, b in zip(ipts, ipts[1:]):
                cv2.line(out, a, b, (0, 165, 255), 2, cv2.LINE_AA)
            for pt in ipts:
                cv2.circle(out, pt, 4, (0, 200, 255), -1, cv2.LINE_AA)

        if det.red_found and det.tape_cx_px is not None:
            tcx = int(round(det.tape_cx_px))
            tcx = max(0, min(w - 1, tcx))
            ty0 = roi_y if roi_y > 0 else 0
            # Tape centreline (vertical)
            cv2.line(out, (tcx, ty0), (tcx, h - 1), (0, 255, 180), 2, cv2.LINE_AA)
            if det.track_x_px is not None and det.track_y_px is not None:
                tx = int(round(det.track_x_px))
                ty = int(round(det.track_y_px))
                tx = max(0, min(w - 1, tx))
                ty = max(0, min(h - 1, ty))
                # Lateral anchor used for steering (extrapolated or blob bottom)
                cv2.circle(out, (tx, ty), 7, (255, 180, 0), 2, cv2.LINE_AA)
                cv2.circle(out, (tx, ty), 2, (255, 220, 100), -1, cv2.LINE_AA)
        return out

    # ── Internals ─────────────────────────────────────────────────────────────

    def _capture_loop(self) -> None:
        """Background thread: capture frames continuously so the camera never waits on processing."""
        while not self._capture_stop.is_set():
            raw = self._cam.capture_array("main")
            frame = np.ascontiguousarray(raw)
            if self._camera_rotation_deg != 0.0:
                h, w = frame.shape[:2]
                M = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), -self._camera_rotation_deg, 1.0)
                frame = cv2.warpAffine(frame, M, (w, h))
            with self._capture_lock:
                self._capture_buf = frame
            self._capture_event.set()

    def _stabilize_red(
        self,
        raw_found: bool,
        raw_err: float,
        tape_cx: float | None,
        tape_cy: float | None,
        track_x: float | None,
        track_y: float | None,
    ) -> tuple[bool, float, float | None, float | None, float | None, float | None]:
        """
        Debounce loss: brief raw misses keep red_found True and hold last smoothed error.

        EMA: when raw red is visible, low-pass red_error to reduce frame-to-frame jitter.

        Tape / track pixels follow the latest raw detection while visible; while debouncing
        loss, the last known geometry is held for overlays.
        """
        a = self.red_error_ema_alpha

        if raw_found:
            self._red_miss_streak = 0
            self._tracking_line = True
            self._last_tape_cx = tape_cx
            self._last_tape_cy = tape_cy
            self._last_track_x = track_x
            self._last_track_y = track_y

            if a <= 1e-12:
                smooth = raw_err
            elif not self._red_ema_seeded:
                smooth = raw_err
                self._red_ema_seeded = True
            else:
                smooth = a * raw_err + (1.0 - a) * self._red_error_ema

            self._red_error_ema = smooth
            return True, smooth, tape_cx, tape_cy, track_x, track_y

        # Raw miss
        if not self._tracking_line:
            self._red_miss_streak = 0
            return False, 0.0, None, None, None, None

        self._red_miss_streak += 1
        if self._red_miss_streak < self.red_loss_debounce_frames:
            return (
                True,
                self._red_error_ema,
                self._last_tape_cx,
                self._last_tape_cy,
                self._last_track_x,
                self._last_track_y,
            )

        # Confirmed lost
        self._tracking_line = False
        self._red_miss_streak = 0
        self._red_ema_seeded = False
        self._red_error_ema = 0.0
        self._last_tape_cx = None
        self._last_tape_cy = None
        self._last_track_x = None
        self._last_track_y = None
        return False, 0.0, None, None, None, None

    def _build_red_mask(self, hsv: np.ndarray) -> np.ndarray:
        """Binary mask after inRange + morphology — shared by _detect_red and get_red_mask_frame."""
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self._RED_LO1, self._RED_HI1),
            cv2.inRange(hsv, self._RED_LO2, self._RED_HI2),
        )
        # Light open removes salt noise; 5×5 close fills tape gaps.
        k3 = np.ones((3, 3), np.uint8)
        k5 = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5, iterations=2)
        # Drop connected components smaller than the configured minimum blob size.
        if self._red_mask_min_blob_px > 0:
            n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            areas = stats[1:, cv2.CC_STAT_AREA]          # skip background label 0
            keep  = np.where(areas >= self._red_mask_min_blob_px)[0] + 1
            mask  = np.isin(labels, keep).astype(np.uint8) * 255
        return mask

    def _detect_curvature(
        self, mask: np.ndarray, frame_w: int, roi_y: int
    ) -> tuple[float, float, float, list]:
        """
        Split the ROI mask into _curve_n_strips horizontal bands and fit a
        degree-2 polynomial  x_norm = a·t² + b·t + c  where:
          t = 0  → bottom of ROI (near robot)
          t = 1  → top of ROI (far ahead)
          x_norm → lateral centroid normalised to [-1, 1] from image centre

        Returns (curvature=a, heading=b, confidence, pts).
          curvature > 0  → line curves right ahead
          heading   > 0  → line already angled right at the robot
          pts            → list of full-frame (x, y) for each valid strip centroid
        """
        h, w = mask.shape[:2]
        if h < self._curve_n_strips or w < 2:
            return 0.0, 0.0, 0.0, []

        half_w     = w / 2.0
        strip_h    = h / self._curve_n_strips
        xs: list[float] = []
        ts: list[float] = []
        pts: list[tuple[float, float]] = []

        for i in range(self._curve_n_strips):
            y0 = int(round(i * strip_h))
            y1 = min(int(round((i + 1) * strip_h)), h)
            strip = mask[y0:y1, :]
            px_cols = np.flatnonzero(np.any(strip > 0, axis=0))
            if len(px_cols) < 5:
                continue
            cx = float(np.mean(px_cols))
            mid_y = (y0 + y1) / 2.0
            pts.append((cx, roi_y + mid_y))
            # t=0 at bottom (near), t=1 at top (far)
            t = 1.0 - mid_y / h
            xs.append((cx - half_w) / half_w)
            ts.append(t)

        conf = len(xs) / self._curve_n_strips
        if len(xs) < 3:
            return 0.0, 0.0, conf, pts

        a, b, c = np.polyfit(ts, xs, 2)

        # Curvature = quadratic coefficient from the best-fit polynomial.
        # This is stable because polyfit minimises error across all strips globally,
        # rather than amplifying per-strip noise via finite differences.
        curvature = float(a)

        # Heading: Gaussian-weighted average of segment slopes.
        # Sort by t (near=0 to far=1) for consistent direction.
        paired = sorted(zip(ts, xs))
        t_s    = [p[0] for p in paired]
        x_s    = [p[1] for p in paired]
        slopes = [(x_s[i+1] - x_s[i]) / (t_s[i+1] - t_s[i])
                  for i in range(len(t_s) - 1)
                  if (t_s[i+1] - t_s[i]) > 1e-9]

        # Use the slope of the segment whose midpoint is closest to t=0.5 (frame centre).
        if slopes:
            seg_mids = [(t_s[i] + t_s[i + 1]) / 2.0 for i in range(len(slopes))]
            center_idx = min(range(len(seg_mids)), key=lambda i: abs(seg_mids[i] - 0.5))
            heading = slopes[center_idx]
        else:
            heading = float(b)

        return curvature, heading, conf, pts

    def _detect_red(
        self, hsv: np.ndarray, frame_w: int, roi_y: int
    ) -> tuple[bool, float, float | None, float | None, float | None, float | None]:
        """
        Returns (found, error, tape_cx, tape_cy, track_x, track_y)
        with tape/track in full-frame pixel coordinates.
        """
        nz = (False, 0.0, None, None, None, None)

        mask = self._build_red_mask(hsv)
        self._last_red_mask = mask
        self._last_red_blob_area = 0.0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return nz

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        self._last_red_blob_area = float(area)
        if area < self._red_min_area:
            return nz

        bx, by, bw, bh = cv2.boundingRect(largest)
        moments = cv2.moments(largest)
        if moments["m00"] <= 1e-6:
            tape_cx = float(bx + bw * 0.5)
            tape_cy = float(roi_y + by + bh * 0.5)
            return True, 0.0, tape_cx, tape_cy, tape_cx, tape_cy

        cx_roi = moments["m10"] / moments["m00"]
        cy_roi = moments["m01"] / moments["m00"]
        tape_cx = float(cx_roi)
        tape_cy = float(roi_y + cy_roi)

        err = _clamp((cx_roi - frame_w / 2.0) / (frame_w / 2.0), -1.0, 1.0)
        return True, err, tape_cx, tape_cy, tape_cx, tape_cy

    @staticmethod
    def _denoise_blob_mask(mask: np.ndarray) -> np.ndarray:
        """Open (remove salt noise) then close (fill small holes) for blob targets."""
        k3 = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k3, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k3, iterations=1)
        return mask

    def _detect_blue(self, hsv: np.ndarray) -> tuple[bool, float | None]:
        """Returns (found, cx_norm) for the largest qualifying blue contour.

        cx_norm is the centroid x normalised to [-1, 1] (positive = right of centre).
        cx_norm is None when no qualifying contour exists.
        """
        mask = cv2.inRange(hsv, self._BLUE_LO, self._BLUE_HI)
        mask = self._denoise_blob_mask(mask)
        self._last_blue_mask = mask
        w = hsv.shape[1]
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) >= _BLUE_MIN_AREA]
        if not valid:
            return False, None
        M = cv2.moments(max(valid, key=cv2.contourArea))
        if M["m00"] == 0:
            return True, None
        cx_norm = _clamp((M["m10"] / M["m00"] - w / 2.0) / (w / 2.0), -1.0, 1.0)
        return True, cx_norm

    def _detect_green(self, hsv: np.ndarray) -> bool:
        """Returns True when a green marker of sufficient area is visible."""
        mask = cv2.inRange(hsv, self._GREEN_LO, self._GREEN_HI)
        mask = self._denoise_blob_mask(mask)
        self._last_green_mask = mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(cv2.contourArea(c) >= _GREEN_MIN_AREA for c in contours)
