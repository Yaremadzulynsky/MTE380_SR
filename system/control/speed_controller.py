"""
SpeedController — per-wheel RPM PID.

Translates speed fractions [-1, 1] into motor voltages using RPM feedback
from RobotBrain.  This is a continuous controller (done is always False) —
call set_target() to change the setpoint then step() every control tick.

    ctrl = SpeedController(brain, cfg)
    ctrl.set_target(left=0.5, right=0.5)
    while running:
        ctrl.step()
        time.sleep(0.02)
"""
from __future__ import annotations

from simple_pid import PID

import config as _config_module
from control.brain import RobotBrain, MAX_RPM, _clamp


class SpeedController:
    """
    Per-wheel RPM PID.  Shares the step() / done / progress() interface with
    RotationController so either can be driven by the same control loop.
    """

    done = False   # continuous controller — never self-terminates

    def __init__(self, brain: RobotBrain, cfg: _config_module.Config | None = None) -> None:
        self._brain = brain
        c = cfg if cfg is not None else _config_module.get()

        self._pid_left  = PID(c.motor_kp, c.motor_ki, c.motor_kd,
                              setpoint=0, output_limits=(-1.0, 1.0), sample_time=None)
        self._pid_right = PID(c.motor_kp, c.motor_ki, c.motor_kd,
                              setpoint=0, output_limits=(-1.0, 1.0), sample_time=None)

        self._target_left  = 0.0
        self._target_right = 0.0

        # Cached for telemetry
        self._last_rpm_tgt_l = 0.0
        self._last_rpm_tgt_r = 0.0
        self._last_voltage_l = 0.0
        self._last_voltage_r = 0.0

    # ── Control interface ─────────────────────────────────────────────────────

    def set_target(self, left: float, right: float) -> None:
        """Set speed fraction targets [-1, 1]."""
        self._target_left  = _clamp(left,  -1.0, 1.0)
        self._target_right = _clamp(right, -1.0, 1.0)

    def step(self) -> None:
        """Read RPM from brain, run PIDs, send voltage."""
        rpm_tgt_l = self._target_left  * MAX_RPM
        rpm_tgt_r = self._target_right * MAX_RPM

        rpm_l, rpm_r = self._brain.measured_rpm

        self._pid_left.setpoint  = rpm_tgt_l
        self._pid_right.setpoint = rpm_tgt_r

        voltage_l = self._pid_left(rpm_l)
        voltage_r = self._pid_right(rpm_r)

        self._last_rpm_tgt_l = rpm_tgt_l
        self._last_rpm_tgt_r = rpm_tgt_r
        self._last_voltage_l = voltage_l
        self._last_voltage_r = voltage_r

        if self._brain.dry_run:
            print(
                f"[dry-run]  L  target={rpm_tgt_l:+6.1f}rpm  "
                f"actual={rpm_l:+6.1f}rpm  V={voltage_l:+.3f}"
                f"    R  target={rpm_tgt_r:+6.1f}rpm  "
                f"actual={rpm_r:+6.1f}rpm  V={voltage_r:+.3f}",
                flush=True,
            )
            return

        self._brain.send_voltage(voltage_l, voltage_r)

    def progress(self) -> tuple[float, float]:
        """Returns current (left_frac, right_frac) targets."""
        return self._target_left, self._target_right

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def reset(self) -> None:
        """Clear integrators."""
        self._pid_left.reset()
        self._pid_right.reset()

    def set_gains(self, cfg: _config_module.Config) -> None:
        """Update PID gains and reset integrators."""
        t = (cfg.motor_kp, cfg.motor_ki, cfg.motor_kd)
        self._pid_left.tunings  = t
        self._pid_right.tunings = t
        self.reset()

    # ── Telemetry ─────────────────────────────────────────────────────────────

    def telemetry_line(self) -> str:
        rpm_l, rpm_r = self._brain.measured_rpm
        return (
            f"tgt_L={self._target_left:+.3f} tgt_R={self._target_right:+.3f}  "
            f"tgt_rpm_L={self._last_rpm_tgt_l:+7.1f} tgt_rpm_R={self._last_rpm_tgt_r:+7.1f}  "
            f"rpm_L={rpm_l:+7.1f} rpm_R={rpm_r:+7.1f}  "
            f"V_L={self._last_voltage_l:+.3f} V_R={self._last_voltage_r:+.3f}"
        )
