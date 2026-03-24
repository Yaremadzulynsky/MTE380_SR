"""
Mission state machine.

State sequence
──────────────
LINE_FOLLOW   → DRIVE_FORWARD when blue target is seen
DRIVE_FORWARD → PICKUP when encoder distance reached
PICKUP        → TURN_180 when hold timer expires
TURN_180      → LINE_FOLLOW (is_returning=True)
LINE_FOLLOW   → DONE when T-junction detected (is_returning=True)
DONE          : motors off

Each state's logic lives in states/<state_name>.py.
"""
from __future__ import annotations

import time

from simple_pid import PID

import config as _config_module
from config import Config
from perception import FrameDetection
from states import ControlOutput, State

import states.line_follow          as _line_follow
import states.find_line            as _find_line
import states.line_follow_reversed as _line_follow_reversed
import states.find_line_reversed   as _find_line_reversed
import states.pid_turn             as _pid_turn
import states.drive_forward        as _drive_forward
import states.pickup               as _pickup
import states.turn_180             as _turn_180
import states.drop_off             as _drop_off


class MissionStateMachine:
    """One step() call per control-loop tick."""

    def __init__(self, cfg: Config | None = None, brain=None) -> None:
        self.cfg    = cfg if cfg is not None else _config_module.get()
        self.state  = State.LINE_FOLLOW
        self._brain = brain  # RobotBrain reference, used by controllers in states

        self._t0               = time.time()
        self._enc0_left        = 0
        self._enc0_right       = 0
        self._last_red_error    = 0.0
        self._last_curvature    = 0.0
        self._find_line_turn_cw    = True   # updated by line_follow_find when red is seen
        self.is_returning          = False  # set True by turn_180 after 180° turn
        self._green_seen_t         = None   # wall-clock time green was first seen (returning leg)
        self._heading_lateral_turn = 0.0   # last lateral PID output (mode 4)
        self._heading_heading_turn = 0.0   # last heading PID output (mode 4)
        self._consecutive_lost  = 0

        self._steer_pid = PID(
            Kp=self.cfg.steer_kp,
            Ki=self.cfg.steer_ki,
            Kd=self.cfg.steer_kd,
            setpoint=0,
            output_limits=(-self.cfg.steer_out_limit, self.cfg.steer_out_limit),
            sample_time=None,
        )

        self._heading_pid = PID(
            Kp=self.cfg.heading_kp,
            Ki=self.cfg.heading_ki,
            Kd=self.cfg.heading_kd,
            setpoint=0,
            output_limits=(-self.cfg.steer_out_limit, self.cfg.steer_out_limit),
            sample_time=None,
        )

    def step(self, det: FrameDetection, left_ticks: int, right_ticks: int) -> ControlOutput:
        if self.state == State.LINE_FOLLOW:
            return _line_follow.step(self, det, left_ticks, right_ticks)
        if self.state == State.FIND_LINE:
            return _find_line.step(self, det, left_ticks, right_ticks)
        if self.state == State.LINE_FOLLOW_REVERSED:
            return _line_follow_reversed.step(self, det, left_ticks, right_ticks)
        if self.state == State.FIND_LINE_REVERSED:
            return _find_line_reversed.step(self, det, left_ticks, right_ticks)
        if self.state == State.PID_TURN:
            return _pid_turn.step(self, det, left_ticks, right_ticks)
        if self.state == State.DRIVE_FORWARD:
            return _drive_forward.step(self, left_ticks, right_ticks)
        if self.state == State.PICKUP:
            return _pickup.step(self)
        if self.state == State.TURN_180:
            return _turn_180.step(self)
        if self.state == State.DROP_OFF:
            return _drop_off.step(self)
        return ControlOutput(left=0.0, right=0.0, claw=None, state=self.state)

    def _enter(self, new_state: State, left_ticks: int = 0, right_ticks: int = 0) -> None:
        print(f"[STATE] {self.state.value} → {new_state.value}", flush=True)
        self.state             = new_state
        self._t0               = time.time()
        self._enc0_left        = left_ticks
        self._enc0_right       = right_ticks
        self._consecutive_lost = 0
        self._steer_pid.reset()
        self._heading_pid.reset()
        if self._brain is not None:
            self._brain._speed_ctrl.reset()
