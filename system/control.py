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

from bridge import SerialBridge


# ── Hardware constants ────────────────────────────────────────────────────────

TICKS_PER_REV = 680   # encoder ticks per wheel revolution
MAX_RPM       = 320   # maximum commanded RPM


# ── Motor PID defaults ────────────────────────────────────────────────────────
# Error  : RPM  (target_rpm − measured_rpm)
# Output : motor voltage [-1, 1]
#
# Starting point:
#   kp = 1/MAX_RPM ≈ 0.003  →  full-speed RPM error ≈ 1.0 V output
#   ki, kd = 0 initially; add once kp is stable

MOTOR_KP = 1.0 / MAX_RPM   # ≈ 0.003125
MOTOR_KI = 0.0005
MOTOR_KD = 0.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ── Public types ──────────────────────────────────────────────────────────────

@dataclass
class MotorCommand:
    left:  float   # desired speed fraction [-1, 1], where ±1 = ±MAX_RPM
    right: float


@dataclass
class MotorPIDConfig:
    kp: float = MOTOR_KP
    ki: float = MOTOR_KI
    kd: float = MOTOR_KD


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
        serial_port:   str,
        baud:          int,
        dry_run:       bool            = False,
        motor_pid_cfg: MotorPIDConfig | None = None,
    ) -> None:
        self.serial_port = serial_port
        self.baud        = baud
        self.dry_run     = dry_run

        cfg = motor_pid_cfg or MotorPIDConfig()

        # One PID per wheel; setpoint is updated each call to send_drive()
        self._pid_left  = PID(cfg.kp, cfg.ki, cfg.kd,
                              setpoint=0, output_limits=(-1.0, 1.0), sample_time=None)
        self._pid_right = PID(cfg.kp, cfg.ki, cfg.kd,
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

    def set_motor_pid(self, cfg: MotorPIDConfig) -> None:
        """Update wheel PID gains (e.g. after reloading pid_config.json); resets integrators."""
        t = (cfg.kp, cfg.ki, cfg.kd)
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
