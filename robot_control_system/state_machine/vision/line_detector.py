"""
vision/line_detector.py

Reads frames from a PiCamera (or fallback camera source) and detects a red
line, returning a unit direction vector tangent to that line plus the true
perpendicular lateral distance from the robot to the line.

Direction vector convention — matches robot.add_direction(x, y):
  x : lateral  [-1, 1]   left = -1, right = +1
  y : forward  always 1.0 when a line is detected

Lateral distance convention:
  lateral_distance_px > 0  →  line is to the RIGHT of the robot  (steer right)
  lateral_distance_px < 0  →  line is to the LEFT  of the robot  (steer left)

  The distance is the true perpendicular distance from the bottom-centre of the
  image (the robot's current ground position) to the fitted line.  This is
  geometrically correct even when the line is diagonal, unlike a simple centroid
  x-offset.

  Pass pixels_per_meter to LineDetector to also get lateral_distance_m.

Usage
-----
    detector = LineDetector(pixels_per_meter=1280)   # PiCamera2, calibrated
    detector = LineDetector(source=0)                # webcam for testing
    detector.start()

    while True:
        result = detector.get_result()
        if result:
            robot.add_direction(*result.direction)
            print(result.lateral_distance_px, result.lateral_distance_m)

    detector.stop()
"""

import logging
import math
import subprocess
import sys as _sys, pathlib as _pathlib
_sys.path.insert(0, str(_pathlib.Path(__file__).parent.parent.parent))
import config as _config
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple, Union

import cv2
import numpy as np

# ── Pi camera via rpicam-vid subprocess ───────────────────────────────────────

_JPEG_SOI = bytes([0xFF, 0xD8])
_JPEG_EOI = bytes([0xFF, 0xD9])


class _RpicamVidCamera:
    """Pi Camera via rpicam-vid subprocess (MJPEG to stdout). No Picamera2/libcamera lock."""

    def __init__(self, width: int = 640, height: int = 480, fps: float = 30.0) -> None:
        self._lock    = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._stop    = False
        self._eof     = False
        self._buffer  = bytearray()

        cmd = [
            'rpicam-vid', '-t', '0', '-o', '-',
            '--codec', 'mjpeg', '-n',
            '--width', str(width), '--height', str(height),
            '--framerate', str(int(fps)),
        ]
        try:
            self._proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                          stderr=subprocess.PIPE, bufsize=0)
        except FileNotFoundError as e:
            raise RuntimeError('rpicam-vid not found — install rpicam-apps') from e

        threading.Thread(target=self._log_stderr, daemon=True).start()

        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _log_stderr(self) -> None:
        assert self._proc.stderr
        for line in self._proc.stderr:
            log.warning('rpicam-vid: %s', line.decode(errors='replace').rstrip())

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

log = logging.getLogger(__name__)

# ── Config (loaded from config.yaml) ─────────────────────────────────────────
_vc = _config.get()['vision']
FRAME_WIDTH  = _vc['frame_width']
FRAME_HEIGHT = _vc['frame_height']
# HSV thresholds and min_mask_pixels are loaded per-instance in __init__ so
# they can be mutated at runtime via set_vision_params().


@dataclass
class LineResult:
    """Result of one frame's line detection."""
    direction: Tuple[float, float]       # (x, y) unit tangent vector in robot frame
    angle_deg: float                     # angle of line from vertical; + = leaning right
    lateral_distance_px: float           # perpendicular dist from robot to line (pixels)
                                         #   + = line is RIGHT of robot (steer right)
                                         #   - = line is LEFT  of robot (steer left)
    lateral_distance_m: Optional[float]  # same in metres; None if pixels_per_meter not set
    mask: Optional[np.ndarray] = field(default=None, repr=False)   # binary red mask (debug)
    frame: Optional[np.ndarray] = field(default=None, repr=False)  # annotated frame (debug)
    sampled_pixels: Optional[list] = field(default=None, repr=False)  # [(u,v)…] N sampled red pixels
    segments: Optional[list] = field(default=None, repr=False)     # [(cx,cy)…] near→far strip centroids
    curvature: Optional[float] = field(default=None, repr=False)   # rad; + = curves right ahead


class LineDetector:
    """
    Background thread that captures camera frames, detects the red line, and
    exposes the latest result via get_result() / get_direction().

    Parameters
    ----------
    source : None | int | str
        None  → use Picamera2 (Raspberry Pi camera)
        int   → cv2.VideoCapture index, e.g. 0 for the first webcam
        str   → cv2.VideoCapture path, e.g. a video file
    debug : bool
        If True, LineResult.mask and LineResult.frame are populated with
        annotated images (useful for tuning via cv2.imshow).
    """

    def __init__(self, *, source: Optional[Union[int, str]] = None,
                 backend: int = 0,
                 debug: bool = False,
                 pixels_per_meter: Optional[float] = None):
        """
        Parameters
        ----------
        source : None | int | str
            None  → Picamera2 (Raspberry Pi)
            int   → cv2.VideoCapture index (e.g. 0 for first webcam)
            str   → cv2.VideoCapture path (e.g. video file)
        backend : int
            cv2 capture backend (e.g. cv2.CAP_DSHOW on Windows). 0 = default.
        debug : bool
            Populate LineResult.mask and LineResult.frame with annotated images.
        pixels_per_meter : float | None
            Ground-plane pixel density at the robot's position.  When set,
            lateral_distance_m is computed as lateral_distance_px / pixels_per_meter.
            Calibrate by placing a known-length object on the ground under the camera
            and counting how many pixels it spans.
        """
        self._source          = source
        self._backend         = backend
        self._debug           = debug
        self._pixels_per_meter = (pixels_per_meter
                                  or _config.get()['vision'].get('pixels_per_meter'))
        self._picam   = None   # Picamera2 instance (Pi path)
        self._cap     = None   # cv2.VideoCapture instance (fallback path)
        self._lock    = threading.Lock()
        self._result: Optional[LineResult] = None
        self._frame:  Optional[np.ndarray] = None   # latest BGR frame (annotated if debug=True)
        self._mask:   Optional[np.ndarray] = None   # latest binary red mask
        self._detection_rate: float = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Runtime-tunable HSV thresholds (copies so config is not mutated)
        _vc = _config.get()['vision']
        rc = _vc['red_hsv']
        self._hsv_lower1 = np.array(rc['lower1'], dtype=np.uint8).copy()
        self._hsv_upper1 = np.array(rc['upper1'], dtype=np.uint8).copy()
        self._hsv_lower2 = np.array(rc['lower2'], dtype=np.uint8).copy()
        self._hsv_upper2 = np.array(rc['upper2'], dtype=np.uint8).copy()
        self._s_min: int = int(self._hsv_lower1[1])
        self._v_min: int = int(self._hsv_lower1[2])
        # Derive hue center/tolerance from loaded arrays (OpenCV H scale 0-179)
        if int(self._hsv_lower1[0]) == 0 and int(self._hsv_lower2[0]) > 0:
            # wrapping case (e.g. red): [0, upper1_h] ∪ [lower2_h, 179]
            self._hue_tol    = max(int(self._hsv_upper1[0]), 179 - int(self._hsv_lower2[0]))
            self._hue_center = 0
        else:
            self._hue_center = (int(self._hsv_lower1[0]) + int(self._hsv_upper1[0])) // 2
            self._hue_tol    = (int(self._hsv_upper1[0]) - int(self._hsv_lower1[0])) // 2
        self._min_mask_pixels: int = _vc['min_mask_pixels']
        self._cam_fwd_m: float = float(_vc.get('camera_forward_m', 0.15))
        self._n_segments: int = int(_vc.get('line_segments', 4))
        self._smoothed_theta: Optional[float] = None  # robot-frame smoothed turn angle

    # ── Public API ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name='line_detector')
        self._thread.start()
        log.info('LineDetector started (source=%s)', self._source)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        self._release_camera()
        log.info('LineDetector stopped')

    def get_result(self) -> Optional[LineResult]:
        """Return the most recent detection result, or None if no line was found."""
        with self._lock:
            return self._result

    def get_stats(self) -> dict:
        """Return detection statistics."""
        with self._lock:
            return {'detection_rate': round(self._detection_rate, 3)}

    def get_vision_params(self) -> dict:
        """Return current tunable detection parameters."""
        with self._lock:
            return {
                's_min':           self._s_min,
                'v_min':           self._v_min,
                'min_mask_pixels': self._min_mask_pixels,
                'hue_center':      self._hue_center,
                'hue_tol':         self._hue_tol,
            }

    def set_vision_params(self, *,
                          s_min: Optional[int] = None,
                          v_min: Optional[int] = None,
                          min_mask_pixels: Optional[int] = None,
                          hue_center: Optional[int] = None,
                          hue_tol: Optional[int] = None) -> None:
        """Update detection parameters at runtime (thread-safe)."""
        with self._lock:
            if s_min is not None:
                self._s_min = int(max(0, min(255, s_min)))
            if v_min is not None:
                self._v_min = int(max(0, min(255, v_min)))
            if hue_center is not None:
                self._hue_center = int(max(0, min(179, hue_center)))
            if hue_tol is not None:
                self._hue_tol = int(max(0, min(89, hue_tol)))
            if min_mask_pixels is not None:
                self._min_mask_pixels = max(0, int(min_mask_pixels))
            self._apply_hue()

    def _apply_hue(self) -> None:
        """Recompute HSV range arrays from current hue_center, hue_tol, s_min, v_min."""
        s, v = self._s_min, self._v_min
        lo = self._hue_center - self._hue_tol
        hi = self._hue_center + self._hue_tol
        if lo < 0:
            self._hsv_lower1 = np.array([0,        s, v],   dtype=np.uint8)
            self._hsv_upper1 = np.array([hi,      255, 255], dtype=np.uint8)
            self._hsv_lower2 = np.array([lo + 180, s, v],   dtype=np.uint8)
            self._hsv_upper2 = np.array([179,     255, 255], dtype=np.uint8)
        elif hi > 179:
            self._hsv_lower1 = np.array([lo,        s, v],   dtype=np.uint8)
            self._hsv_upper1 = np.array([179,      255, 255], dtype=np.uint8)
            self._hsv_lower2 = np.array([0,         s, v],   dtype=np.uint8)
            self._hsv_upper2 = np.array([hi - 180, 255, 255], dtype=np.uint8)
        else:
            self._hsv_lower1 = np.array([lo, s, v],   dtype=np.uint8)
            self._hsv_upper1 = np.array([hi, 255, 255], dtype=np.uint8)
            self._hsv_lower2 = np.array([lo, s, v],   dtype=np.uint8)
            self._hsv_upper2 = np.array([hi, 255, 255], dtype=np.uint8)
        _config.update({
            'vision': {
                'min_mask_pixels': self._min_mask_pixels,
                'red_hsv': {
                    'lower1': self._hsv_lower1.tolist(),
                    'upper1': self._hsv_upper1.tolist(),
                    'lower2': self._hsv_lower2.tolist(),
                    'upper2': self._hsv_upper2.tolist(),
                },
            },
        })

    def set_smoothed_heading(self, theta: Optional[float]) -> None:
        """Set the smoothed robot-frame turn angle for camera overlay annotation.
        theta = odometry.heading - smoothed_target_heading  (positive = turn right).
        Call this each tick from the state machine.
        """
        with self._lock:
            self._smoothed_theta = theta

    def get_direction(self) -> Optional[Tuple[float, float]]:
        """Convenience wrapper — returns (x, y) direction or None."""
        result = self.get_result()
        return result.direction if result else None

    def process_frame(self, bgr: np.ndarray) -> Optional[LineResult]:
        """
        Run line detection on a single BGR frame and update internal state.

        Intended for phone-push mode where the caller supplies frames directly
        instead of letting the background thread read from a camera.  Safe to
        call from any thread.  Returns the detection result (or None).
        """
        result = self._detect(bgr)
        display = result.frame if (result and result.frame is not None) else bgr
        with self._lock:
            self._result = result
            self._frame  = display
            self._mask   = result.mask if result is not None else self._build_red_mask(bgr)
        return result

    def get_frame(self) -> Optional[np.ndarray]:
        """Return the latest BGR frame (annotated if debug=True, else raw)."""
        with self._lock:
            return self._frame

    def get_mask_frame(self) -> Optional[np.ndarray]:
        """Return the latest red mask as a BGR image (white = detected red)."""
        with self._lock:
            if self._mask is None:
                return None
            return cv2.cvtColor(self._mask, cv2.COLOR_GRAY2BGR)

    # ── Camera helpers ─────────────────────────────────────────────────────────

    def _open_camera(self) -> None:
        if self._source is None:
            # Raspberry Pi path — rpicam-vid subprocess (no libcamera lock)
            self._picam = _RpicamVidCamera(FRAME_WIDTH, FRAME_HEIGHT)
            log.info('rpicam-vid opened (%dx%d)', FRAME_WIDTH, FRAME_HEIGHT)
        else:
            self._cap = (cv2.VideoCapture(self._source, self._backend)
                         if self._backend else cv2.VideoCapture(self._source))
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            if not self._cap.isOpened():
                raise RuntimeError(f'Cannot open camera source: {self._source!r}')
            log.info('cv2.VideoCapture opened (source=%s, backend=%s)',
                     self._source, self._backend)

    def _read_frame(self) -> Optional[np.ndarray]:
        """Return the next BGR frame, or None on failure."""
        if self._picam is not None:
            return self._picam.read()
        if self._cap is not None:
            ok, frame = self._cap.read()
            return frame if ok else None
        return None

    def _release_camera(self) -> None:
        if self._picam is not None:
            self._picam.release()
            self._picam = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    # ── Detection ──────────────────────────────────────────────────────────────

    def _build_red_mask(self, bgr: np.ndarray) -> np.ndarray:
        """Return a binary mask isolating red pixels."""
        hsv   = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self._hsv_lower1, self._hsv_upper1)
        mask2 = cv2.inRange(hsv, self._hsv_lower2, self._hsv_upper2)
        mask  = cv2.bitwise_or(mask1, mask2)
        # Remove small noise, fill small holes
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _detect(self, frame: np.ndarray) -> Optional[LineResult]:
        h, w = frame.shape[:2]
        img_cx = w / 2.0

        mask = self._build_red_mask(frame)

        if int(cv2.countNonZero(mask)) < self._min_mask_pixels:
            return None

        # ── Contour extraction ────────────────────────────────────────────────
        # Find contours of the red region; use the largest one as the line.
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < self._min_mask_pixels:
            return None

        # ── Centroid via moments ──────────────────────────────────────────────
        M = cv2.moments(contour)
        if M['m00'] < 1:
            return None
        centroid_x = M['m10'] / M['m00']
        centroid_y = M['m01'] / M['m00']

        # ── Direction via fitLine on largest-contour pixels only ─────────────
        # Fill the largest contour into a clean mask so that stray red pixels
        # elsewhere in the frame cannot influence the direction fit.
        contour_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, cv2.FILLED)
        all_pts = cv2.findNonZero(contour_mask)
        if all_pts is None:
            return None
        [vx], [vy], [x0], [y0] = cv2.fitLine(all_pts, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy, x0, y0 = float(vx), float(vy), float(x0), float(y0)

        # Ensure tangent points "forward" (upward in image = negative vy).
        if vy > 0:
            vx, vy = -vx, -vy

        # Global fitLine angle (used for lateral distance and fallback direction).
        angle_rad = math.atan2(vx, -vy)
        angle_deg = math.degrees(angle_rad)
        direction = (math.sin(angle_rad), math.cos(angle_rad))
        # Near-segment tangent will override direction below once segments are computed.

        # ── Lateral distance ──────────────────────────────────────────────────
        # Perpendicular distance from bottom-centre of the image (robot ground
        # position) to the fitted line, signed: + = line right, - = line left.
        # The line passes through (x0, y0) with unit tangent (vx, vy); the
        # centroid is used as the anchor point (x0, y0) for the fit so the
        # lateral reading reflects the true centre of the detected region.
        ref_x, ref_y = img_cx, float(h - 1)
        lateral_px = (ref_x - centroid_x) * vy - (ref_y - centroid_y) * vx
        lateral_m  = (lateral_px / self._pixels_per_meter
                      if self._pixels_per_meter is not None else None)

        # ── Sample largest-contour pixels for world-space splatter ────────────
        _N    = 30
        n_pts = len(all_pts)
        if n_pts > _N:
            idx     = np.random.choice(n_pts, _N, replace=False)
            sampled = [(int(all_pts[i][0][0]), int(all_pts[i][0][1])) for i in idx]
        else:
            sampled = [(int(p[0][0]), int(p[0][1])) for p in all_pts]

        # ── Segmented curvature detection ─────────────────────────────────────
        strip_h = max(1, h // self._n_segments)
        min_strip_px = max(5, self._min_mask_pixels // (self._n_segments * 2))
        segments: list = []
        for i in range(self._n_segments):
            y1 = h - (i + 1) * strip_h
            y2 = h - i * strip_h
            strip = contour_mask[max(0, y1):y2, :]
            if cv2.countNonZero(strip) >= min_strip_px:
                Mseg = cv2.moments(strip)
                if Mseg['m00'] > 0:
                    cx = Mseg['m10'] / Mseg['m00']
                    cy = max(0, y1) + Mseg['m01'] / Mseg['m00']
                    segments.append((float(cx), float(cy)))

        curvature: Optional[float] = None
        if len(segments) >= 2:
            dx_n = segments[1][0] - segments[0][0]
            dy_n = segments[1][1] - segments[0][1]
            mag_n = math.hypot(dx_n, dy_n)
            if mag_n > 0:
                seg_vx = dx_n / mag_n
                seg_vy = dy_n / mag_n
                # Ensure it points upward (forward in image).
                if seg_vy > 0:
                    seg_vx, seg_vy = -seg_vx, -seg_vy
                seg_angle_rad = math.atan2(seg_vx, -seg_vy)
                angle_deg = math.degrees(seg_angle_rad)
                direction  = (math.sin(seg_angle_rad), math.cos(seg_angle_rad))
                angle_rad  = seg_angle_rad   # keep consistent for annotation theta

        if len(segments) >= 3:
            dx_f = segments[-1][0] - segments[-2][0]
            dy_f = segments[-1][1] - segments[-2][1]
            a_near = math.atan2(segments[1][0] - segments[0][0],
                                -(segments[1][1] - segments[0][1]))
            a_far  = math.atan2(dx_f, -dy_f)
            curvature = (a_far - a_near + math.pi) % (2 * math.pi) - math.pi

        # ── Annotation ────────────────────────────────────────────────────────
        ARROW_LEN      = 80
        DEADZONE_M     = 0.01    # metres — matches machine._LATERAL_DEADZONE_M
        FONT           = cv2.FONT_HERSHEY_SIMPLEX
        FONT_SCALE     = 0.42
        THICKNESS      = 1

        annotated  = frame.copy()
        robot_pt   = (int(ref_x), int(ref_y))

        # Target direction in image space: same formula as StateMachine._update_target_heading.
        # Robot forward = (0, -1) in image. Turning right by theta gives (sin θ, -cos θ).
        # theta = line_angle_rad + lat_corr (both expressed as right-turn-from-forward).
        #   line_angle_rad = angle_rad (positive = line tilts right = robot must turn right)
        #   lat_corr = atan2(lat_m, cam_fwd_m) if |lat_m| > deadzone else 0
        if lateral_m is not None and abs(lateral_m) >= DEADZONE_M:
            lat_corr = math.atan2(lateral_m, self._cam_fwd_m)
        else:
            lat_corr = 0.0
        theta = angle_rad + lat_corr
        tx = math.sin(theta)
        ty = -math.cos(theta)
        tgt_end = (int(ref_x + tx * ARROW_LEN), int(ref_y + ty * ARROW_LEN))

        # Raw target arrow (faint, solid thin)
        cv2.line(annotated, robot_pt, tgt_end, (0, 60, 160), 1)
        cv2.putText(annotated, 'RAW', (tgt_end[0] + 4, tgt_end[1]),
                    FONT, FONT_SCALE, (0, 60, 160), THICKNESS)

        # Smoothed target arrow (bright orange, thicker) — from state machine filter
        with self._lock:
            smoothed_theta = self._smoothed_theta
        if smoothed_theta is not None:
            sx = math.sin(smoothed_theta)
            sy = -math.cos(smoothed_theta)
            sm_end = (int(ref_x + sx * ARROW_LEN), int(ref_y + sy * ARROW_LEN))
            cv2.arrowedLine(annotated, robot_pt, sm_end, (0, 100, 255), 3, tipLength=0.25)
            cv2.putText(annotated, 'TARGET', (sm_end[0] + 4, sm_end[1]),
                        FONT, FONT_SCALE, (0, 100, 255), THICKNESS)

        cv2.circle(annotated, robot_pt, 5, (255, 255, 255), -1)

        # Segment centroids (always drawn when segments are available)
        SEG_COLOR = (180, 255, 100)   # lime green
        for idx, (scx, scy) in enumerate(segments):
            pt = (int(scx), int(scy))
            cv2.circle(annotated, pt, 4, SEG_COLOR, -1)
            if idx > 0:
                prev = (int(segments[idx - 1][0]), int(segments[idx - 1][1]))
                cv2.line(annotated, prev, pt, SEG_COLOR, 1)
        if curvature is not None:
            curv_deg = math.degrees(curvature)
            cv2.putText(annotated, f'curv={curv_deg:+.1f}d',
                        (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                        SEG_COLOR, 1)

        # Contour outline
        cv2.drawContours(annotated, [contour], -1, (0, 180, 180), 1)

        # Fitted line across the full frame
        t = float(max(w, h))
        cv2.line(annotated,
                 (int(centroid_x - vx * t), int(centroid_y - vy * t)),
                 (int(centroid_x + vx * t), int(centroid_y + vy * t)),
                 (0, 120, 0), 1)

        # Centroid dot
        cv2.circle(annotated, (int(centroid_x), int(centroid_y)), 5, (0, 255, 255), -1)

        # Robot forward arrow
        fwd_end = (int(ref_x), int(ref_y - ARROW_LEN))
        cv2.arrowedLine(annotated, robot_pt, fwd_end, (255, 255, 255), 2, tipLength=0.2)
        cv2.putText(annotated, 'FWD', (fwd_end[0] + 4, fwd_end[1]),
                    FONT, FONT_SCALE, (255, 255, 255), THICKNESS)

        # Tangent arrow
        tan_end = (int(ref_x + vx * ARROW_LEN), int(ref_y + vy * ARROW_LEN))
        cv2.arrowedLine(annotated, robot_pt, tan_end, (0, 230, 0), 2, tipLength=0.25)
        cv2.putText(annotated, 'TANGENT', (tan_end[0] + 4, tan_end[1]),
                    FONT, FONT_SCALE, (0, 230, 0), THICKNESS)

        # Lateral correction arrow: perpendicular toward line centre
        lat_nx = math.copysign(1.0, lateral_px) * (-vy)
        lat_ny = math.copysign(1.0, lateral_px) * vx
        lat_draw_len = min(1.0, abs(lat_corr) / (math.pi / 4)) * ARROW_LEN
        lat_end = (int(ref_x + lat_nx * lat_draw_len), int(ref_y + lat_ny * lat_draw_len))
        if lat_draw_len > 2:
            cv2.arrowedLine(annotated, robot_pt, lat_end, (255, 220, 0), 2, tipLength=0.25)
        cv2.putText(annotated, 'LATERAL',
                    (lat_end[0] + 4, lat_end[1]),
                    FONT, FONT_SCALE, (255, 220, 0), THICKNESS)

        lat_label = (f'{lateral_px:+.1f}px'
                     + (f' / {lateral_m:+.3f}m' if lateral_m is not None else ''))
        cv2.putText(
            annotated,
            f'angle={angle_deg:+.1f}  lat={lat_label}  dir=({direction[0]:+.2f},{direction[1]:+.2f})',
            (8, 20), FONT, 0.45, (255, 255, 255), THICKNESS,
        )

        legend = [
            ((0, 180, 180),   '(0) CONTOUR EDGES'),
            ((0, 255, 255),   '(1) CENTROID'),
            ((0, 230, 0),     '(2) TANGENT (fitLine on edges)'),
            ((255, 220, 0),   f'(3) LATERAL (deadzone={DEADZONE_M}m)'),
            ((0, 100, 255),   '(4) TARGET heading'),
            ((180, 255, 100), '(5) SEGMENTS / near tangent'),
        ]
        for i, (colour, text) in enumerate(legend):
            y = h - 12 - i * 16
            cv2.rectangle(annotated, (8, y - 9), (18, y + 1), colour, -1)
            cv2.putText(annotated, text, (22, y), FONT, FONT_SCALE, colour, THICKNESS)

        return LineResult(
            direction=direction,
            angle_deg=angle_deg,
            lateral_distance_px=lateral_px,
            lateral_distance_m=lateral_m,
            mask=mask,
            frame=annotated,
            sampled_pixels=sampled,
            segments=segments if segments else None,
            curvature=curvature,
        )

    # ── Background thread ──────────────────────────────────────────────────────

    def _loop(self) -> None:
        try:
            self._open_camera()
        except Exception:
            log.exception('Failed to open camera')
            return

        frames_total    = 0
        frames_detected = 0

        while self._running:
            frame = self._read_frame()
            if frame is None:
                log.warning('Failed to read frame — retrying')
                time.sleep(0.05)
                continue

            frames_total += 1
            result = self._detect(frame)
            if result is not None:
                frames_detected += 1
            display = result.frame if (result and result.frame is not None) else frame
            with self._lock:
                self._result = result
                self._frame  = display
                self._mask   = result.mask if result is not None else self._build_red_mask(frame)
                self._detection_rate = frames_detected / frames_total

        self._release_camera()
