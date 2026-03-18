import logging
import math
import threading
import time

from .bridge      import SerialBridge
from .heading_pid import HeadingPID, MOTOR_DEADBAND
from .speed_pid   import SpeedPID

# ── Config ────────────────────────────────────────────────────────────────────

RATE_HZ   = 20.0
MAX_SPEED = 1.0

HEARTBEAT_TIMEOUT = 3.0   # seconds before warning


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

    def __init__(self, port: str, baud: int = 115200):
        self._bridge = SerialBridge(port, baud)
        self._bridge.on_heartbeat = self._on_heartbeat
        self._bridge.on_encoders  = self._on_encoders

        self._heading_pid = HeadingPID()
        self._speed_pid   = SpeedPID()

        self._lock = threading.Lock()
        self._target_heading = 0.0  # radians — updated incrementally via set_direction
        self._speed_scale    = 0.0  # [-1, 1] direct motor speed
        self._rotation_scale = 1.0  # [0, 1] multiplier applied to angular_z
        self._motor_override: tuple[float, float] | None = None  # (left, right)
        self._heading_fb = 0.0   # radians — from encoder odometry
        self._linear_speed = 0.0  # m/s — forward speed from encoders

        self._enc_left      = 0
        self._enc_right     = 0
        self._last_enc_time = 0.0

        self._last_heartbeat = 0.0
        self._running = False

    # ── Public API ────────────────────────────────────────────────────────────

    def set_direction(self, x: float, y: float):
        """Update target heading relative to current heading.

        delta = atan2(x, abs(y))
        target_heading = current_heading + delta

        x=0, y=1  → delta=0°, hold heading
        x=1, y=0  → delta=+90°, turn right
        x=0, y=0  → no update, hold heading
        """
        with self._lock:
            self._motor_override = None
            if abs(x) > 1e-6 or abs(y) > 1e-6:
                delta = math.atan2(x, abs(y))
                self._target_heading = self._heading_fb + delta

    def set_speed(self, speed: float):
        """Set forward/backward speed. -1 = full reverse, 0 = stop, 1 = full forward."""
        with self._lock:
            self._speed_scale = max(-1.0, min(1.0, speed))

    def set_rotation_scale(self, scale: float):
        """Set rotation speed as a fraction of max angular output. 0 = no rotation, 1 = full."""
        with self._lock:
            self._rotation_scale = max(0.0, min(1.0, scale))

    def set_gains(self, kp: float, ki: float, kd: float):
        """Set heading PID gains live."""
        self._heading_pid.set_gains(kp, ki, kd)
        log.info('heading PID gains → kp=%.3f ki=%.3f kd=%.3f', kp, ki, kd)

    def set_speed_gains(self, kp: float, ki: float, kd: float):
        """Set speed PID gains live."""
        self._speed_pid.set_gains(kp, ki, kd)
        log.info('speed PID gains → kp=%.3f ki=%.3f kd=%.3f', kp, ki, kd)

    def set_heading_feedback(self, radians: float):
        with self._lock:
            self._heading_fb = radians

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
                omega              = (dr - dl) / WHEEL_BASE_M / dt
                self._heading_fb  += omega * dt
                self._linear_speed = (dl + dr) / 2.0 / dt
            self._enc_left      = left
            self._enc_right     = right
            self._last_enc_time = now

    def _control_loop(self):
        interval = 1.0 / RATE_HZ
        while self._running:
            t0 = time.monotonic()

            with self._lock:
                target_heading = self._target_heading
                speed_scale    = self._speed_scale
                rotation_scale = self._rotation_scale
                hdg_fb         = self._heading_fb
                override       = self._motor_override

            if override is not None:
                self._bridge.send_drive(*override)
            elif abs(speed_scale) < 1e-6:
                self._heading_pid.reset()
                self._bridge.send_drive(0.0, 0.0)
            else:
                angular_z = self._heading_pid.update(target_heading, hdg_fb, rotation_scale)
                linear_x  = speed_scale

                left  = max(-1.0, min(1.0, linear_x - angular_z))
                right = max(-1.0, min(1.0, linear_x + angular_z))
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
