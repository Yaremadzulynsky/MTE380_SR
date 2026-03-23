"""
RobotBrain — hardware abstraction for the on-Pi runtime.

Owns the serial link to the Arduino and maintains encoder / RPM state.
SpeedController and RotationController both take a RobotBrain as their
hardware interface — they read encoder_ticks / measured_rpm from it and
write voltages back through send_voltage().
"""
from __future__ import annotations

import threading
import time

from bridge import SerialBridge


# ── Hardware constants ────────────────────────────────────────────────────────

TICKS_PER_REV = 680   # encoder ticks per wheel revolution
MAX_RPM       = 320   # maximum commanded RPM


# ── Helpers ───────────────────────────────────────────────────────────────────

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ── Brain ─────────────────────────────────────────────────────────────────────

class RobotBrain:
    """
    Hardware abstraction: serial bridge, encoder callbacks, raw voltage and
    claw commands.  Contains no PID logic.
    """

    def __init__(self, serial_port: str, baud: int, dry_run: bool = False) -> None:
        self.serial_port = serial_port
        self.baud        = baud
        self.dry_run     = dry_run

        self._lock = threading.Lock()

        # Encoder state — updated by the bridge read thread
        self._left_ticks  = 0
        self._right_ticks = 0
        self._prev_left   = 0
        self._prev_right  = 0
        self._enc_time    = 0.0
        self._rpm_left    = 0.0
        self._rpm_right   = 0.0

        self.bridge: SerialBridge | None = None
        if not dry_run:
            self.bridge = SerialBridge(serial_port, baud)
            self.bridge.on_encoders = self._on_encoders
            self.bridge.start()

    # ── Commands ──────────────────────────────────────────────────────────────

    def send_voltage(self, left: float, right: float) -> None:
        """Send motor voltages directly. Range [-1, 1]."""
        left  = _clamp(left,  -1.0, 1.0)
        right = _clamp(right, -1.0, 1.0)
        if self.dry_run:
            print(f"[dry-run]  send_voltage  L={left:+.3f}  R={right:+.3f}", flush=True)
            return
        assert self.bridge is not None
        self.bridge.send_drive(left, right)

    def send_claw(self, angle: float) -> None:
        if self.dry_run:
            print(f"[dry-run]  claw  angle={angle:.1f}°", flush=True)
            return
        assert self.bridge is not None
        self.bridge.send_claw(angle)

    def idle(self) -> None:
        """Zero motor voltages; keep serial open."""
        if self.dry_run:
            return
        if self.bridge is not None:
            try:
                self.bridge.send_drive(0.0, 0.0)
            except Exception:
                pass

    def shutdown(self) -> None:
        """Zero motors and close serial — call once on process exit."""
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
        """Latest cumulative (left_ticks, right_ticks)."""
        with self._lock:
            return self._left_ticks, self._right_ticks

    @property
    def measured_rpm(self) -> tuple[float, float]:
        """Latest measured (left_rpm, right_rpm). Negative = reverse."""
        with self._lock:
            return self._rpm_left, self._rpm_right

    # ── Internal ──────────────────────────────────────────────────────────────

    def _on_encoders(self, left: int, right: int) -> None:
        """Called by the bridge read thread on every encoder packet."""
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
