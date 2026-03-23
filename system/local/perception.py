"""Camera perception: red line, blue target, and T-junction detection."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import cv2
import numpy as np
from picamera2 import Picamera2


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


@dataclass
class FrameDetection:
    red_found:  bool        # red line detected in ROI
    red_error:  float       # [-1, 1], positive = line is right of centre
    blue_found: bool        # blue target marker visible
    t_junction: bool        # wide red stripe = end T-junction
    # Full-frame pixels for overlays (None when red not found / not yet tracked)
    tape_cx_px: float | None = None   # horizontal centre of tape (mask centroid)
    tape_cy_px: float | None = None   # vertical centre of mask centroid
    track_x_px: float | None = None   # lateral anchor: u used for red_error (extrapolated or blob bottom)
    track_y_px: float | None = None


class Perception:
    """
    Reads one camera frame and returns a FrameDetection.

    Red line : HSV mask → largest contour → lateral error in image space
    Blue target : HSV mask → any contour above min area
    T-junction  : red bounding-rect width > half the frame = horizontal end bar

    Lateral error uses the blob **bottom** as the near-wheel anchor. The mask **centroid**
    and bottom define a line in the image; ``line_axle_extrap`` extends that line downward
    (larger v) by ``line_axle_extrap * (v_bottom - v_centroid)`` so steering references a
    point closer to the wheelbase on curves. ``0`` = use the blob bottom only.

    ``red_loss_debounce_frames``: require this many consecutive raw misses before reporting
    ``red_found=False`` (holds last smoothed error meanwhile).

    ``red_error_ema_alpha``: low-pass on ``red_error`` when raw red is visible (0 = off).
    """

    # ── HSV colour ranges ─────────────────────────────────────────────────────
    # Red wraps at 0/179 in OpenCV H (0–180). Two bands: orange-red and magenta-red.
    # Midpoint between loose (wood noise) and tight (thin mask on pink tape).
    # Stricter S/V + narrower H: wood grain is less saturated than tape; orange-brown sits in wider H1.

    _RED_LO1 = np.array([0, 155, 84], np.uint8)
    _RED_HI1 = np.array([10, 255, 255], np.uint8)
    _RED_LO2 = np.array([161, 155, 84], np.uint8)
    _RED_HI2 = np.array([179, 255, 255], np.uint8)
    # Blue tuned to be a bit more forgiving but still reject washed-out noise.
    _BLUE_LO = np.array([ 95, 120,  70], np.uint8)
    _BLUE_HI = np.array([135, 255, 255], np.uint8)

    def __init__(
        self,
        width:  int   = 640,
        height: int   = 480,
        roi_top_ratio: float = 0.0,   # 0 = full frame; else ignore top fraction (e.g. 0.5 = bottom half)
        red_min_area:  float = 65.0,
        blue_min_area: float = 1500.0,  # require a substantial blue patch to avoid false triggers
        t_junction_width_ratio: float = 0.5,  # red bbox > this fraction of frame = T-junction
        camera_channel_order: Literal["rgb", "bgr"] = "bgr",
        line_axle_extrap: float = 0.0,
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

        self.line_axle_extrap = max(0.0, float(line_axle_extrap))
        self.red_loss_debounce_frames = max(1, int(red_loss_debounce_frames))
        self.red_error_ema_alpha = _clamp(float(red_error_ema_alpha), 0.0, 1.0)

        # Raw-red debounce + EMA (see _stabilize_red)
        self._red_miss_streak = 0
        self._tracking_line = False
        self._red_error_ema = 0.0
        self._red_ema_seeded = False
        self._last_t_junction = False
        self._last_tape_cx: float | None = None
        self._last_tape_cy: float | None = None
        self._last_track_x: float | None = None
        self._last_track_y: float | None = None

        self._cam = Picamera2()
        cfg = self._cam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"},
            buffer_count=2,
        )
        self._cam.configure(cfg)
        self._cam.start()

    def configure_line_error(self, *, line_axle_extrap: float | None = None) -> None:
        """Update line-error extrapolation from pid_config without reopening the camera."""
        if line_axle_extrap is not None:
            self.line_axle_extrap = max(0.0, float(line_axle_extrap))

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

        raw_found, raw_err, raw_tj, tcx, tcy, trx, try_ = self._detect_red(hsv, w, roi_y)
        (
            red_found,
            red_error,
            t_junction,
            tape_cx_px,
            tape_cy_px,
            track_x_px,
            track_y_px,
        ) = self._stabilize_red(raw_found, raw_err, raw_tj, tcx, tcy, trx, try_)
        blue_found = self._detect_blue(hsv)

        return FrameDetection(
            red_found=red_found,
            red_error=red_error,
            blue_found=blue_found,
            t_junction=t_junction,
            tape_cx_px=tape_cx_px,
            tape_cy_px=tape_cy_px,
            track_x_px=track_x_px,
            track_y_px=track_y_px,
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

        if det.red_found and det.tape_cx_px is not None:
            tcx = int(round(det.tape_cx_px))
            tcx = max(0, min(w - 1, tcx))
            # Tape centreline (vertical) — match ROI extent when using roi_top_ratio
            ty0 = roi_y if roi_y > 0 else 0
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

    def _stabilize_red(
        self,
        raw_found: bool,
        raw_err: float,
        raw_t_junction: bool,
        tape_cx: float | None,
        tape_cy: float | None,
        track_x: float | None,
        track_y: float | None,
    ) -> tuple[bool, float, bool, float | None, float | None, float | None, float | None]:
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
            self._last_t_junction = raw_t_junction
            return True, smooth, raw_t_junction, tape_cx, tape_cy, track_x, track_y

        # Raw miss
        if not self._tracking_line:
            self._red_miss_streak = 0
            return False, 0.0, False, None, None, None, None

        self._red_miss_streak += 1
        if self._red_miss_streak < self.red_loss_debounce_frames:
            # Hold line-follow semantics so the state machine does not enter coast/search
            return (
                True,
                self._red_error_ema,
                self._last_t_junction,
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
        self._last_t_junction = False
        self._last_tape_cx = None
        self._last_tape_cy = None
        self._last_track_x = None
        self._last_track_y = None
        return False, 0.0, False, None, None, None, None

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
    ) -> tuple[bool, float, bool, float | None, float | None, float | None, float | None]:
        """
        Returns (found, error, t_junction, tape_cx, tape_cy, track_x, track_y)
        with tape/track in full-frame pixel coordinates.
        """
        nz = (False, 0.0, False, None, None, None, None)

        mask = self._build_red_mask(hsv)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return nz

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.red_min_area:
            return nz

        bx, by, bw, bh = cv2.boundingRect(largest)
        t_junction = (bw / frame_w) > self.t_junction_width_ratio

        moments = cv2.moments(largest)
        if moments["m00"] <= 1e-6:
            tape_cx = float(bx + bw * 0.5)
            tape_cy = float(roi_y + by + bh * 0.5)
            return True, 0.0, t_junction, tape_cx, tape_cy, tape_cx, tape_cy

        cx_roi = moments["m10"] / moments["m00"]
        cy_roi = moments["m01"] / moments["m00"]
        tape_cx = float(cx_roi)
        tape_cy = float(roi_y + cy_roi)

        # Bottom of blob → ground point nearer the wheels than the centroid
        pts = largest.reshape(-1, 2)
        v_max = float(np.max(pts[:, 1]))
        row = pts[pts[:, 1] >= v_max - 1.0]
        u_pix = float(np.mean(row[:, 0])) if len(row) > 0 else cx_roi
        v_pix = float(roi_y + v_max)

        u_c = float(cx_roi)
        v_c = float(roi_y + cy_roi)
        v_span = v_pix - v_c
        v_target = v_pix + self.line_axle_extrap * v_span
        v_target = _clamp(v_target, 0.0, float(self.height - 1))
        u_steer = _u_on_line_at_v(u_c, v_c, u_pix, v_pix, v_target)
        err = _clamp((u_steer - frame_w / 2.0) / (frame_w / 2.0), -1.0, 1.0)
        return True, err, t_junction, tape_cx, tape_cy, float(u_steer), float(v_target)

    def _detect_blue(self, hsv: np.ndarray) -> bool:
        """Returns True when a blue marker of sufficient area is visible."""
        mask      = cv2.inRange(hsv, self._BLUE_LO, self._BLUE_HI)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(cv2.contourArea(c) >= self.blue_min_area for c in contours)
