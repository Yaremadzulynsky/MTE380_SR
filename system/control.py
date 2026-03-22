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

import threading
import time
from dataclasses import dataclass

from simple_pid import PID

import config as _config_module
from bridge import SerialBridge


# ── Hardware constants ────────────────────────────────────────────────────────

TICKS_PER_REV = 680   # encoder ticks per wheel revolution
MAX_RPM       = 320   # maximum commanded RPM


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ── Public types ──────────────────────────────────────────────────────────────

@dataclass
class MotorCommand:
    left:  float   # desired speed fraction [-1, 1], where ±1 = ±MAX_RPM
    right: float


# ── Position mover ────────────────────────────────────────────────────────────

class PositionMover:
    """
    Moves the robot a fixed number of encoder ticks using a position PID.

    Uses the average of both wheel encoders as the position measurement so the
    robot drives straight.  Positive delta = forward, negative = backward.

    Usage (call each control-loop tick until done):

        mover = PositionMover(control, delta_ticks=500)
        while not mover.done:
            l, r = mover.step()
            control.send_voltage(l, r)   # bypasses motor speed PID
        control.idle()
    """

    def __init__(
        self,
        control: "LocalMotorController",
        delta_ticks: int,
        cfg: _config_module.Config | None = None,
    ) -> None:
        c = cfg if cfg is not None else _config_module.get()
        self._control   = control
        l, r            = control.encoder_ticks
        self._target    = (l + r) / 2.0 + delta_ticks
        self._tolerance = max(1, int(c.pos_tolerance))
        self._pid = PID(
            c.pos_kp, c.pos_ki, c.pos_kd,
            setpoint=self._target,
            output_limits=(-abs(c.pos_max_speed), abs(c.pos_max_speed)),
            sample_time=None,
        )
        self.done = False

    def step(self) -> tuple[float, float]:
        """
        Compute one PID step. Returns (left_voltage, right_voltage) to pass to
        send_voltage() — bypasses the motor speed PID entirely.
        Sets self.done=True and returns (0, 0) when on target.
        """
        l, r = self._control.encoder_ticks
        pos  = (l + r) / 2.0
        if abs(self._target - pos) <= self._tolerance:
            self.done = True
            return 0.0, 0.0
        v = self._pid(pos)
        return v, v


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
