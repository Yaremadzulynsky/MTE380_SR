import logging
import math
import threading
import time

from .bridge      import SerialBridge
from .heading_pid import HeadingPID, MOTOR_DEADBAND
from .speed_pid   import SpeedPID

# ── Config ────────────────────────────────────────────────────────────────────

RATE_HZ   = 20.0
MAX_SPEED = 0.2

HEARTBEAT_TIMEOUT = 3.0   # seconds before warning

#TODO IMPLEMENT LAG CONTROLLER FOR TURNING SPEED

# ── Wheel geometry (update these for your robot) ──────────────────────────────
TICKS_PER_REV    = 680    # encoder ticks per full wheel revolution
WHEEL_DIAMETER_M = 0.08  # wheel diameter in metres
WHEEL_BASE_M     = 0.188   # centre-to-centre distance between wheels in metres
WHEEL_CIRC_M     = math.pi * WHEEL_DIAMETER_M

# ── Helpers ───────────────────────────────────────────────────────────────────

def _apply_deadband(value: float) -> float:
    if abs(value) < 1e-6:
        return 0.0
    return math.copysign(max(abs(value), MOTOR_DEADBAND), value)


# ── Robot ─────────────────────────────────────────────────────────────────────

log = logging.getLogger('robot')


class Robot:

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        *,
        use_encoder_heading_odometry: bool = True,
    ):
        self._bridge = SerialBridge(port, baud)
        self._bridge.on_heartbeat = self._on_heartbeat
        self._bridge.on_encoders  = self._on_encoders

        self._heading_pid = HeadingPID()
        self._speed_pid   = SpeedPID()
        self.max_speed    = MAX_SPEED  # settable at runtime via set_max_speed()
        # When False, _heading_fb is only updated via set_heading_feedback (e.g. vision).
        self._use_encoder_heading_odometry = bool(use_encoder_heading_odometry)

        self._lock = threading.Lock()
        self._target_heading = 0.0  # radians — updated incrementally via set_direction
        self._speed_scale    = 0.0  # [-1, 1] direct motor speed
        self._rotation_scale = 1.0  # [0, 1] multiplier applied to angular_z
        self._motor_override: tuple[float, float] | None = None  # (left, right)
        self._heading_fb = 0.0   # radians — encoder odometry and/or set_heading_feedback
        self._linear_speed = 0.0  # m/s — forward speed from encoders

        # Lag controller coefficients (0 = off, 0.9 = heavy smoothing).
        # turn_lag filters angular_z; speed_lag filters linear_x.
        self._turn_lag_alpha:  float = 0.0
        self._speed_lag_alpha: float = 0.0
        # Lag filter state (running weighted averages).
        self._angular_z_lag: float = 0.0
        self._linear_x_lag:  float = 0.0

        self._enc_left      = 0
        self._enc_right     = 0
        self._last_enc_time = 0.0

        self._last_heartbeat = 0.0
        self._running = False

    # ── Public API ────────────────────────────────────────────────────────────

    def set_direction(self, x: float, y: float):
        """Update target heading relative to current heading.

        delta = atan2(-x, abs(y))
        target_heading = current_heading + delta

        x=0, y=1  → delta=0°, hold heading
        x=1, y=0  → delta=-90°
        x=0, y=0  → no update, hold heading
        """
        with self._lock:
            self._motor_override = None
            if abs(x) > 1e-6 or abs(y) > 1e-6:
                delta = math.atan2(-x, abs(y))
                self._target_heading = self._heading_fb + delta

    def set_speed(self, speed: float):
        """Set forward/backward speed. -1 = full reverse, 0 = stop, 1 = full forward."""
        with self._lock:
            self._speed_scale = max(-1.0, min(1.0, speed))

    def set_max_speed(self, value: float):
        """Cap total motor output. 0.2 = 20% PWM max. Applies to all wheel commands."""
        self.max_speed = max(0.0, min(1.0, float(value)))

    def set_rotation_scale(self, scale: float):
        """Set rotation speed as a fraction of max angular output. 0 = no rotation, 1 = full."""
        with self._lock:
            self._rotation_scale = max(0.0, min(1.0, scale))

    def set_gains(self, kp: float, ki: float, kd: float):
        """Set heading PID gains live."""
        self._heading_pid.set_gains(kp, ki, kd)
        log.info('heading PID gains → kp=%.3f ki=%.3f kd=%.3f', kp, ki, kd)

    def get_heading_gains(self) -> tuple[float, float, float]:
        """Return (kp, ki, kd) for heading PID (maps to control-screen P/I/D paths)."""
        return self._heading_pid.get_gains()

    def set_speed_gains(self, kp: float, ki: float, kd: float):
        """Set speed PID gains live."""
        self._speed_pid.set_gains(kp, ki, kd)
        log.info('speed PID gains → kp=%.3f ki=%.3f kd=%.3f', kp, ki, kd)

    def set_heading_feedback(self, radians: float):
        """Set current heading (rad) for the heading PID. Required each frame when
        use_encoder_heading_odometry is False (e.g. vision path heading).
        Resets the PID integrator if the heading jumps by more than ~8° to avoid
        integrator windup when the path is reacquired after a miss."""
        with self._lock:
            if abs(radians - self._heading_fb) > 0.14:  # ~8°
                self._heading_pid.reset()
            self._heading_fb = float(radians)

    def set_turn_lag(self, alpha: float) -> None:
        """Set lag filter coefficient for angular (turn) output.
        0 = no lag (pass-through), 0.5–0.8 = noticeable smoothing, max 0.95."""
        with self._lock:
            self._turn_lag_alpha = max(0.0, min(0.95, float(alpha)))

    def set_speed_lag(self, alpha: float) -> None:
        """Set lag filter coefficient for linear (speed) output.
        0 = no lag (pass-through), 0.5–0.8 = noticeable smoothing, max 0.95."""
        with self._lock:
            self._speed_lag_alpha = max(0.0, min(0.95, float(alpha)))

    def get_lag_params(self) -> dict[str, float]:
        """Return current lag coefficients: {"turn": α, "speed": α}."""
        with self._lock:
            return {"turn": self._turn_lag_alpha, "speed": self._speed_lag_alpha}

    def set_use_encoder_heading_odometry(self, enabled: bool) -> None:
        """If True, integrate yaw from wheel encoders in _on_encoders. If False, only
        set_heading_feedback updates _heading_fb."""
        with self._lock:
            self._use_encoder_heading_odometry = bool(enabled)

    @property
    def uses_encoder_heading_odometry(self) -> bool:
        with self._lock:
            return self._use_encoder_heading_odometry

    def get_heading_rad(self) -> tuple[float, float]:
        """Return (current_heading_rad, target_heading_rad)."""
        with self._lock:
            return float(self._heading_fb), float(self._target_heading)

    def get_heading(self) -> tuple[float, float]:
        """Return (current_heading_deg, target_heading_deg)."""
        with self._lock:
            return math.degrees(self._heading_fb), math.degrees(self._target_heading)

    def get_encoders(self) -> tuple[int, int]:
        """Return (left_ticks, right_ticks)."""
        with self._lock:
            return self._enc_left, self._enc_right

    def drive_ticks(self, left_delta: int, right_delta: int,
                    kp: float = 0.001, kd: float = 0.0, deadband: int = 10, timeout: float = 10.0):
        """Drive both wheels by delta ticks using independent PD loops. Blocks until done or timeout."""
        with self._lock:
            target_left  = self._enc_left  + left_delta
            target_right = self._enc_right + right_delta
            self._motor_override = (0.0, 0.0)

        prev_left_err  = 0
        prev_right_err = 0
        prev_time      = time.monotonic()

        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                dt  = now - prev_time

                with self._lock:
                    cur_left  = self._enc_left
                    cur_right = self._enc_right

                left_err  = target_left  - cur_left
                right_err = target_right - cur_right

                left_deriv  = (left_err  - prev_left_err)  / dt if dt > 0 else 0.0
                right_deriv = (right_err - prev_right_err) / dt if dt > 0 else 0.0

                left_out  = _apply_deadband(kp * left_err  + kd * left_deriv)  if abs(left_err)  > deadband else 0.0
                right_out = _apply_deadband(kp * right_err + kd * right_deriv) if abs(right_err) > deadband else 0.0

                print(f'\r  L err={left_err:6d} out={left_out:+.3f}   R err={right_err:6d} out={right_out:+.3f}',
                      end='', flush=True)

                prev_left_err  = left_err
                prev_right_err = right_err
                prev_time      = now

                if abs(left_err) <= deadband and abs(right_err) <= deadband:
                    print(f'\n  done (L err={left_err}  R err={right_err})')
                    break

                with self._lock:
                    self._motor_override = (left_out, right_out)

                time.sleep(0.05)
            else:
                print(f'\n  timeout — L err={left_err}  R err={right_err}')
        finally:
            with self._lock:
                self._motor_override = None
            self._bridge.send_drive(0.0, 0.0)

    def set_motors(self, left: float, right: float):
        """Directly set left/right motor speeds [-1, 1], bypassing PID."""
        with self._lock:
            self._motor_override = (left, right)
            self._heading_pid.reset()

    def set_claw(self, angle: float):
        self._bridge.send_claw(angle)

    def turn_by_degrees(
        self,
        degrees: float,
        speed: float = 0.35,
        tolerance_deg: float = 3.0,
        timeout: float = 8.0,
        settle_time_s: float = 0.2,
    ) -> dict:
        """Turn robot by requested degrees using _heading_fb (encoders and/or vision).

        If use_encoder_heading_odometry is False, something must update heading during
        the turn (e.g. vision) or feedback will not reflect the real pose."""
        def _wrap_pi(angle: float) -> float:
            return (angle + math.pi) % (2.0 * math.pi) - math.pi

        req_deg = float(degrees)
        req_rad = math.radians(req_deg)
        cmd_speed = max(0.1, min(1.0, abs(float(speed))))
        tol_rad = math.radians(max(0.5, abs(float(tolerance_deg))))
        timeout_s = max(0.5, float(timeout))
        settle_s = max(0.05, float(settle_time_s))

        with self._lock:
            start_heading = float(self._heading_fb)
            start_target = float(self._target_heading)
            start_left = int(self._enc_left)
            start_right = int(self._enc_right)
            target_heading = _wrap_pi(start_heading + req_rad)
            self._target_heading = target_heading
            self._speed_scale = 0.0
            self._rotation_scale = 1.0

        start_time = time.monotonic()
        within_since = None
        last_err = None

        try:
            while time.monotonic() - start_time < timeout_s:
                with self._lock:
                    current_heading = float(self._heading_fb)
                err = _wrap_pi(target_heading - current_heading)
                last_err = err

                if abs(err) <= tol_rad:
                    if within_since is None:
                        within_since = time.monotonic()
                    elif time.monotonic() - within_since >= settle_s:
                        break
                else:
                    within_since = None

                # Proportional angular command with saturation; sign comes from heading error.
                ang_cmd = max(-cmd_speed, min(cmd_speed, 1.2 * err))
                left = _apply_deadband(-ang_cmd)
                right = _apply_deadband(ang_cmd)
                with self._lock:
                    self._motor_override = (left, right)
                time.sleep(0.02)
        finally:
            with self._lock:
                self._motor_override = None
                end_heading = float(self._heading_fb)
                end_left = int(self._enc_left)
                end_right = int(self._enc_right)
                self._target_heading = start_target
                self._speed_scale = 0.0
            self._bridge.send_drive(0.0, 0.0)

        achieved_rad = _wrap_pi(end_heading - start_heading)
        achieved_deg = math.degrees(achieved_rad)
        error_deg = req_deg - achieved_deg
        success = abs(error_deg) <= max(abs(tolerance_deg), 3.0)

        return {
            "requested_degrees": req_deg,
            "achieved_degrees": achieved_deg,
            "error_degrees": error_deg,
            "tolerance_degrees": max(abs(tolerance_deg), 3.0),
            "timed_out": (time.monotonic() - start_time) >= timeout_s and not success,
            "success": success,
            "heading": {
                "start_degrees": math.degrees(start_heading),
                "end_degrees": math.degrees(end_heading),
            },
            "encoders": {
                "start_left": start_left,
                "start_right": start_right,
                "end_left": end_left,
                "end_right": end_right,
                "delta_left": end_left - start_left,
                "delta_right": end_right - start_right,
            },
            "last_error_degrees": math.degrees(last_err) if last_err is not None else None,
        }

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        self._bridge.start()
        self._running = True
        threading.Thread(target=self._control_loop, daemon=True).start()
        threading.Thread(target=self._watchdog_loop, daemon=True).start()
        log.info('Robot started')

    def stop(self):
        self._running = False
        self._speed_scale = 0.0
        self._bridge.send_drive(0.0, 0.0)
        self._bridge.stop()
        log.info('Robot stopped')

    # ── Internal ──────────────────────────────────────────────────────────────

    def _on_heartbeat(self):
        self._last_heartbeat = time.monotonic()
        log.debug('Heartbeat received')

    def _on_encoders(self, left: int, right: int):
        now = time.monotonic()
        with self._lock:
            dt = now - self._last_enc_time
            if self._last_enc_time > 0.0 and dt > 0.0:
                dl = (left  - self._enc_left)  * WHEEL_CIRC_M / TICKS_PER_REV
                dr = (right - self._enc_right) * WHEEL_CIRC_M / TICKS_PER_REV
                omega = (dr - dl) / WHEEL_BASE_M / dt
                if self._use_encoder_heading_odometry:
                    self._heading_fb += omega * dt
                self._linear_speed = (dl + dr) / 2.0 / dt
            self._enc_left      = left
            self._enc_right     = right
            self._last_enc_time = now

    def _control_loop(self):
        interval = 1.0 / RATE_HZ
        while self._running:
            t0 = time.monotonic()

            with self._lock:
                target_heading   = self._target_heading
                speed_scale      = self._speed_scale
                rotation_scale   = self._rotation_scale
                hdg_fb           = self._heading_fb
                override         = self._motor_override
                turn_lag_alpha   = self._turn_lag_alpha
                speed_lag_alpha  = self._speed_lag_alpha

            if override is not None:
                self._bridge.send_drive(*override)
            elif abs(speed_scale) < 1e-6:
                self._heading_pid.reset()
                self._angular_z_lag = 0.0
                self._linear_x_lag  = 0.0
                self._bridge.send_drive(0.0, 0.0)
            else:
                angular_z = self._heading_pid.update(target_heading, hdg_fb, rotation_scale)
                linear_x  = speed_scale

                # Lag filters: y[k] = α·y[k-1] + (1−α)·u[k]
                if turn_lag_alpha > 1e-6:
                    self._angular_z_lag = turn_lag_alpha * self._angular_z_lag + (1.0 - turn_lag_alpha) * angular_z
                    angular_z = self._angular_z_lag
                else:
                    self._angular_z_lag = angular_z

                if speed_lag_alpha > 1e-6:
                    self._linear_x_lag = speed_lag_alpha * self._linear_x_lag + (1.0 - speed_lag_alpha) * linear_x
                    linear_x = self._linear_x_lag
                else:
                    self._linear_x_lag = linear_x

                # Cap total motor output to the commanded speed so the heading
                # PID angular correction can't drive motors beyond MAX_SPEED.
                # When stopped (speed_scale≈0) the branch above handles it.
                cap = min(abs(speed_scale), self.max_speed)
                left  = max(-cap, min(cap, linear_x - angular_z))
                right = max(-cap, min(cap, linear_x + angular_z))
                self._bridge.send_drive(left, right)

            log.debug('target_hdg=%.1f° hdg_fb=%.1f° speed=%.2f', math.degrees(target_heading), math.degrees(hdg_fb), speed_scale)

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))

    def _watchdog_loop(self):
        while self._running:
            time.sleep(1.0)
            if self._last_heartbeat == 0.0:
                continue
            elapsed = time.monotonic() - self._last_heartbeat
            if elapsed > HEARTBEAT_TIMEOUT:
                log.warning('No heartbeat from Arduino for %.1fs', elapsed)
