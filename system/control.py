"""
Motor and servo output for the local (on-Pi) runtime.

Two-layer motor control
───────────────────────
The state machine outputs desired speed fractions [-1, 1] where ±1 maps to
±MAX_RPM.  A per-wheel PID (simple-pid) uses encoder feedback to maintain
the commanded RPM by modulating the voltage sent to each motor.

  State machine → left/right fraction → Motor PID → voltage → Arduino
                                             ▲
                                       encoder RPM feedback

simple-pid handles timing internally; output_limits provides anti-windup.
https://simple-pid.readthedocs.io

Hardware constants (update for your robot)
──────────────────────────────────────────
  TICKS_PER_REV : encoder counts per full wheel revolution
  MAX_RPM       : maximum wheel RPM — speed fractions are relative to this
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

from simple_pid import PID

import config as _config_module
from bridge import SerialBridge


# ── Hardware constants ────────────────────────────────────────────────────────

TICKS_PER_REV = 680   # encoder ticks per wheel revolution
MAX_RPM       = 320   # maximum commanded RPM


# ── Discrete PID (matches robot_control_system/pid.py) ───────────────────────

class _DiscretePID:
    """Discrete PID with real dt, derivative-on-error, and integral clamping."""

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float = -1.0, out_max: float = 1.0,
                 integral_limit: float = 1.0) -> None:
        self.kp = kp; self.ki = ki; self.kd = kd
        self.out_min = out_min; self.out_max = out_max
        self.integral_limit = integral_limit
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time: float | None = None

    def reset(self) -> None:
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def update(self, setpoint: float, measurement: float) -> float:
        now   = time.monotonic()
        error = setpoint - measurement
        dt    = (now - self._prev_time) if self._prev_time is not None else 0.0
        if dt > 0:
            self._integral = max(-self.integral_limit,
                                 min(self.integral_limit, self._integral + error * dt))
        deriv = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        self._prev_time  = now
        raw = self.kp * error + self.ki * self._integral + self.kd * deriv
        return max(self.out_min, min(self.out_max, raw))


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _ticks_for_degrees(degrees: float, wheel_diameter_m: float, wheelbase_m: float) -> float:
    """Encoder ticks each wheel travels for an in-place rotation of *degrees*."""
    arc = math.radians(abs(degrees)) * (wheelbase_m / 2.0)
    return (arc / (math.pi * wheel_diameter_m)) * TICKS_PER_REV


# ── Public types ──────────────────────────────────────────────────────────────

@dataclass
class MotorCommand:
    left:  float   # desired speed fraction [-1, 1], where ±1 = ±MAX_RPM
    right: float


# ── Position controller ───────────────────────────────────────────────────────

class PositionController:
    """
    Drives the robot forward or backward a fixed number of encoder ticks.

    PID error = target_ticks - signed_displacement (positive = not there yet,
    negative = overshot).  Output is a signed voltage [-1, 1] so the robot
    can reverse to correct overshoot, enabling proper PID behaviour.

        ctrl = PositionController(control, delta_ticks=500)
        while not ctrl.done:
            ctrl.step()
        control.idle()

    Positive delta_ticks = forward, negative = backward.
    """

    def __init__(
        self,
        control:     "LocalMotorController",
        delta_ticks: int,
        tolerance:   int   | None = None,
        cfg:         _config_module.Config | None = None,
    ) -> None:
        c = cfg if cfg is not None else _config_module.get()
        tol = tolerance if tolerance is not None else c.pos_tolerance
        l, r = control.encoder_ticks
        self._control   = control
        self._start_avg = (l + r) / 2.0
        self._target    = int(delta_ticks)   # signed
        self._tolerance = max(1, tol)
        self.done       = False
        # PID: setpoint = target (signed), input = signed displacement, output = voltage [-1, 1]
        self._pid = PID(c.pos_kp, c.pos_ki, c.pos_kd,
                        setpoint=float(self._target),
                        output_limits=(-1.0, 1.0),
                        sample_time=None)

    def progress(self) -> tuple[float, float]:
        """Returns (abs_ticks_traveled, abs_ticks_target) for display."""
        l, r = self._control.encoder_ticks
        displaced = (l + r) / 2.0 - self._start_avg
        return abs(displaced), float(abs(self._target))

    def step(self) -> None:
        """Send one drive command. Sets self.done=True when within tolerance."""
        if self.done:
            return
        l, r = self._control.encoder_ticks
        displaced = (l + r) / 2.0 - self._start_avg
        if abs(displaced - self._target) <= self._tolerance:
            self.done = True
            return
        voltage = self._pid(displaced)
        self._control.send_voltage(voltage, voltage)


# ── Rotation controller ───────────────────────────────────────────────────────

class RotationController:
    """
    Rotates the robot in place by a fixed number of degrees.

    Positive degrees = clockwise: left wheel forward, right backward.
    Negative degrees = counter-clockwise: opposite.

    Each step integrates encoder deltas into a running heading (radians) using
    differential-drive kinematics — the same approach used in the reference
    robot_control_system.  The PID operates on angle error (radians), so
    rot_kp/ki/kd are radian-space gains and rot_tolerance is in degrees.

        ctrl = RotationController(control, degrees=90)
        while not ctrl.done:
            ctrl.step()
        control.idle()
    """

    @staticmethod
    def _wrap(angle: float) -> float:
        """Wrap angle to (-π, π]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def _apply_deadband(value: float, deadband: float) -> float:
        """Zero tiny signals; boost non-zero signals to at least deadband."""
        if abs(value) < 1e-6:
            return 0.0
        return math.copysign(max(abs(value), deadband), value)

    def __init__(
        self,
        control:   "LocalMotorController",
        degrees:   float,
        tolerance: float | None = None,
        cfg:       _config_module.Config | None = None,
    ) -> None:
        c = cfg if cfg is not None else _config_module.get()
        tol_deg = tolerance if tolerance is not None else c.rot_tolerance
        l, r = control.encoder_ticks
        self._control        = control
        self._prev_left      = l
        self._prev_right     = r
        self._wheel_circ     = math.pi * c.wheel_diameter_m
        self._wheelbase      = c.wheelbase_m
        self._heading        = 0.0                   # integrated, radians; positive = CW
        self._target         = math.radians(degrees) # positive = CW
        self._tol_rad        = math.radians(abs(tol_deg))
        self._motor_deadband = c.rot_motor_deadband
        self.done            = False
        # Discrete PID matching robot_control_system/pid.py.
        # Called with (error, 0.0) — error as setpoint, 0 as measurement.
        self._pid = _DiscretePID(c.rot_kp, c.rot_ki, c.rot_kd)

    def progress(self) -> tuple[float, float]:
        """Returns (abs_degrees_traveled, abs_degrees_target) for display."""
        return abs(math.degrees(self._heading)), abs(math.degrees(self._target))

    def step(self) -> None:
        """Integrate heading, run PID, send voltage. Matches HeadingPID.update() logic."""
        if self.done:
            return

        # Integrate encoder deltas → heading (CW = positive)
        l, r = self._control.encoder_ticks
        dl = (l - self._prev_left)  * self._wheel_circ / TICKS_PER_REV
        dr = (r - self._prev_right) * self._wheel_circ / TICKS_PER_REV
        self._heading   += (dl - dr) / self._wheelbase
        self._prev_left  = l
        self._prev_right = r

        # Angle-wrapped error — same as HeadingPID: error = _wrap(desired - current)
        error = self._wrap(self._target - self._heading)

        # Done when within tolerance — equivalent to HEADING_DEADBAND check
        if abs(error) <= self._tol_rad:
            self._pid.reset()
            self.done = True
            return

        # PID on error directly, matching: pid.update(error, 0.0)
        output = self._pid.update(error, 0.0)

        # Motor deadband — matching: _apply_deadband(output)
        output = self._apply_deadband(output, self._motor_deadband)

        # Positive output = CW (left fwd, right rev); negative = CCW.
        self._control.send_voltage(output, -output)


# ── Controller ────────────────────────────────────────────────────────────────

class LocalMotorController:
    """
    Wraps SerialBridge to send drive and claw commands.

    Each call to send_drive():
      1. Converts the speed fractions to RPM targets.
      2. Sets each wheel PID's setpoint to its target RPM.
      3. Calls the PID with the latest measured RPM → voltage output.
      4. Sends voltages to the Arduino.

    simple-pid handles dt and anti-windup internally.
    """

    def __init__(
        self,
        serial_port: str,
        baud:        int,
        dry_run:     bool = False,
        cfg:         _config_module.Config | None = None,
    ) -> None:
        self.serial_port = serial_port
        self.baud        = baud
        self.dry_run     = dry_run

        c = cfg if cfg is not None else _config_module.get()

        # One PID per wheel; setpoint is updated each call to send_drive()
        self._pid_left  = PID(c.motor_kp, c.motor_ki, c.motor_kd,
                              setpoint=0, output_limits=(-1.0, 1.0), sample_time=None)
        self._pid_right = PID(c.motor_kp, c.motor_ki, c.motor_kd,
                              setpoint=0, output_limits=(-1.0, 1.0), sample_time=None)

        self._lock = threading.Lock()

        # Encoder state — updated by the Arduino callback thread
        self._left_ticks  = 0
        self._right_ticks = 0
        self._prev_left   = 0
        self._prev_right  = 0
        self._enc_time    = 0.0
        self._rpm_left    = 0.0
        self._rpm_right   = 0.0

        self.bridge: SerialBridge | None = None
        # Last loop values for terminal telemetry (updated in send_drive)
        self._last_cmd_l = 0.0
        self._last_cmd_r = 0.0
        self._last_target_rpm_l = 0.0
        self._last_target_rpm_r = 0.0
        self._last_voltage_l = 0.0
        self._last_voltage_r = 0.0

        if not dry_run:
            self.bridge = SerialBridge(serial_port, baud)
            self.bridge.on_encoders = self._on_encoders
            self.bridge.start()

    def set_motor_pid(self, cfg: _config_module.Config) -> None:
        """Update wheel PID gains from a Config object; resets integrators."""
        t = (cfg.motor_kp, cfg.motor_ki, cfg.motor_kd)
        self._pid_left.tunings = t
        self._pid_right.tunings = t
        self._pid_left.reset()
        self._pid_right.reset()

    def reset_wheel_pids(self) -> None:
        """Clear inner-loop integral state (e.g. when entering lost-line search spin)."""
        self._pid_left.reset()
        self._pid_right.reset()

    # ── Commands ──────────────────────────────────────────────────────────────

    def send_drive(self, cmd: MotorCommand) -> None:
        """Convert speed fractions → RPM targets → motor PID → voltage."""
        target_left_rpm  = _clamp(cmd.left,  -1.0, 1.0) * MAX_RPM
        target_right_rpm = _clamp(cmd.right, -1.0, 1.0) * MAX_RPM

        with self._lock:
            measured_left  = self._rpm_left
            measured_right = self._rpm_right

        # Update setpoints then call each PID with the current measurement
        self._pid_left.setpoint  = target_left_rpm
        self._pid_right.setpoint = target_right_rpm

        voltage_left  = self._pid_left(measured_left)
        voltage_right = self._pid_right(measured_right)

        self._last_cmd_l = float(cmd.left)
        self._last_cmd_r = float(cmd.right)
        self._last_target_rpm_l = target_left_rpm
        self._last_target_rpm_r = target_right_rpm
        self._last_voltage_l = voltage_left
        self._last_voltage_r = voltage_right

        if self.dry_run:
            print(
                f"[dry-run]  L  target={target_left_rpm:+6.1f}rpm  "
                f"actual={measured_left:+6.1f}rpm  V={voltage_left:+.3f}"
                f"    R  target={target_right_rpm:+6.1f}rpm  "
                f"actual={measured_right:+6.1f}rpm  V={voltage_right:+.3f}",
                flush=True,
            )
            return

        assert self.bridge is not None
        self.bridge.send_drive(voltage_left, voltage_right)

    def send_voltage(self, left: float, right: float) -> None:
        """Send motor voltages directly, bypassing the RPM PID. Range [-1, 1]."""
        left  = _clamp(left,  -1.0, 1.0)
        right = _clamp(right, -1.0, 1.0)
        if self.dry_run:
            print(f"[dry-run]  send_voltage  L={left:+.3f}  R={right:+.3f}", flush=True)
            return
        assert self.bridge is not None
        self.bridge.send_drive(left, right)

    def idle(self) -> None:
        """Zero motors and reset wheel PIDs; keep the serial port open (e.g. after pause / mission leg)."""
        self._pid_left.reset()
        self._pid_right.reset()
        if self.dry_run:
            return
        if self.bridge is not None:
            try:
                self.bridge.send_drive(0.0, 0.0)
            except Exception:
                pass

    def send_claw(self, angle: float) -> None:
        if self.dry_run:
            print(f"[dry-run]  claw  angle={angle:.1f}°", flush=True)
            return
        assert self.bridge is not None
        self.bridge.send_claw(angle)

    def shutdown(self) -> None:
        """Stop motors and close serial — call once on process exit."""
        self.idle()
        if self.dry_run:
            print("[dry-run]  shutdown", flush=True)
            return
        if self.bridge is not None:
            self.bridge.stop()
            self.bridge = None

    # ── Encoder access ────────────────────────────────────────────────────────

    @property
    def encoder_ticks(self) -> tuple[int, int]:
        """Latest cumulative (left_ticks, right_ticks) from the Arduino."""
        with self._lock:
            return self._left_ticks, self._right_ticks

    @property
    def measured_rpm(self) -> tuple[float, float]:
        """Latest measured (left_rpm, right_rpm). Negative = reverse."""
        with self._lock:
            return self._rpm_left, self._rpm_right

    def motor_telemetry_line(self) -> str:
        """One-line summary: fractions, RPM targets, measured RPM, PID voltage out."""
        with self._lock:
            ml, mr = self._rpm_left, self._rpm_right
        return (
            f"cmd_L={self._last_cmd_l:+.3f} cmd_R={self._last_cmd_r:+.3f}  "
            f"tgt_rpm_L={self._last_target_rpm_l:+7.1f} tgt_rpm_R={self._last_target_rpm_r:+7.1f}  "
            f"rpm_L={ml:+7.1f} rpm_R={mr:+7.1f}  "
            f"V_L={self._last_voltage_l:+.3f} V_R={self._last_voltage_r:+.3f}"
        )

    # ── Internal ──────────────────────────────────────────────────────────────

    def _on_encoders(self, left: int, right: int) -> None:
        """Called by the bridge read thread on every encoder packet from the Arduino."""
        now = time.monotonic()
        with self._lock:
            dt = now - self._enc_time
            if self._enc_time > 0.0 and dt >= 0.005:   # skip packets < 5 ms apart
                dl = left  - self._prev_left
                dr = right - self._prev_right
                # rpm = (ticks / ticks_per_rev) / dt_seconds * 60
                self._rpm_left  = (dl / TICKS_PER_REV) * 60.0 / dt
                self._rpm_right = (dr / TICKS_PER_REV) * 60.0 / dt

            self._left_ticks  = left
            self._right_ticks = right
            self._prev_left   = left
            self._prev_right  = right
            self._enc_time    = now
