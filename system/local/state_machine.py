"""
Mission state machine.

State sequence
──────────────
LINE_FOLLOW   : follow the red line (steering PID + lost-line search)
                → transitions to DRIVE_FORWARD when blue target is seen

DRIVE_FORWARD : drive straight for a fixed encoder distance to reach pickup
                → transitions to PICKUP when distance is reached

PICKUP        : stop, close claw, hold briefly
                → transitions to TURN_180 when hold timer expires

TURN_180      : spin right in place for a timed 180°
                → transitions to RETURN when spin timer expires

RETURN        : follow the red line back to the start
                → transitions to DONE when the T-junction (end bar) is detected

DONE          : motors off, mission complete

Steering PID
────────────
Uses simple-pid (https://simple-pid.readthedocs.io).
  setpoint  = 0           (want line_error centred)
  measurement = line_error  [-1, 1]
  output    = turn correction, clamped to ±steer_out_limit

simple-pid handles timing internally — no manual dt tracking needed.
output_limits also acts as anti-windup: integral is not accumulated
while the output is saturated.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum

from simple_pid import PID

from local.perception import FrameDetection


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ── States ────────────────────────────────────────────────────────────────────

class State(Enum):
    LINE_FOLLOW   = "LINE_FOLLOW"
    DRIVE_FORWARD = "DRIVE_FORWARD"
    PICKUP        = "PICKUP"
    TURN_180      = "TURN_180"
    RETURN        = "RETURN"
    DONE          = "DONE"


# ── Configuration ─────────────────────────────────────────────────────────────

@dataclass
class Config:
    # ── Steering PID ─────────────────────────────────────────────────────────
    steer_kp:        float = 0.65
    steer_ki:        float = 0.04
    steer_kd:        float = 0.10
    steer_out_limit: float = 0.80   # output clamped to ±this value

    # ── Forward speed (fraction of MAX_RPM) ──────────────────────────────────
    base_speed:      float = 0.28   # speed at zero line error
    min_speed:       float = 0.16   # speed at maximum line error
    max_speed:       float = 0.45   # hard ceiling (line follow + steer mix)
    search_turn:     float = 0.18   # desired differential magnitude when line is lost
    # Caps search spin only — independent of max_speed so a low line-follow cap does not
    # force a tiny in-place search (~0.08 → wheels barely move).
    search_turn_max: float = 0.32
    # Require this many consecutive frames with no red before starting search spin.
    # Stops brief contour flicker from looking like “rotating on the line”.
    lost_frames_before_search: int = 5

    # ── Drive-forward (encoder-based) ────────────────────────────────────────
    forward_ticks:   int   = 800    # average encoder ticks — tune on hardware
    forward_speed:   float = 0.25

    # ── 180° turn (timed) ────────────────────────────────────────────────────
    turn_speed:      float = 0.30
    turn_duration_s: float = 2.2    # tune so the robot rotates ~180°

    # ── Servo / claw ─────────────────────────────────────────────────────────
    claw_open:       float = 0.0    # degrees, servo resting position
    claw_closed:     float = 90.0   # degrees, gripping position
    pickup_hold_s:   float = 0.8    # seconds to hold claw before turning


# ── Output ────────────────────────────────────────────────────────────────────

@dataclass
class ControlOutput:
    left:  float          # desired speed fraction for left  wheel [-1, 1]
    right: float          # desired speed fraction for right wheel [-1, 1]
    claw:  float | None   # servo angle in degrees, or None = leave unchanged
    state: State


# ── State machine ─────────────────────────────────────────────────────────────

class MissionStateMachine:
    """One step() call per control-loop tick."""

    def __init__(self, config: Config | None = None) -> None:
        self.cfg   = config or Config()
        self.state = State.LINE_FOLLOW

        self._t0         = time.time()
        self._enc0_left  = 0
        self._enc0_right = 0
        # Last red_error while line was visible — used to spin *toward* the line when lost
        # (same sign convention as perception: + = line right of image centre).
        self._last_red_error = 0.0
        self._consecutive_lost = 0  # frames without red (reset when line reacquired)

        # Steering PID: drives line_error → 0
        #   Measurement is red_error ∈ [-1,1], already (cx − frame_w/2) / (frame_w/2) in
        #   perception — so "centered" is 0, not frame_w/2 pixels. Setpoint must match units.
        #   output_limits provides anti-windup (integral freezes when output saturates)
        self._steer_pid = PID(
            Kp=self.cfg.steer_kp,
            Ki=self.cfg.steer_ki,
            Kd=self.cfg.steer_kd,
            setpoint=0,
            output_limits=(-self.cfg.steer_out_limit, self.cfg.steer_out_limit),
            sample_time=None,   # always compute when called; loop rate is controlled externally
        )

    # ── Main entry point ──────────────────────────────────────────────────────

    def step(
        self,
        det:         FrameDetection,
        left_ticks:  int,
        right_ticks: int,
    ) -> ControlOutput:
        if self.state == State.LINE_FOLLOW:
            return self._line_follow(det, left_ticks, right_ticks)
        if self.state == State.DRIVE_FORWARD:
            return self._drive_forward(left_ticks, right_ticks)
        if self.state == State.PICKUP:
            return self._pickup()
        if self.state == State.TURN_180:
            return self._turn_180()
        if self.state == State.RETURN:
            return self._return_follow(det)
        # DONE
        return ControlOutput(left=0.0, right=0.0, claw=None, state=self.state)

    # ── Per-state logic ───────────────────────────────────────────────────────

    def _line_follow(
        self, det: FrameDetection, left_ticks: int, right_ticks: int
    ) -> ControlOutput:
        if det.blue_found:
            self._enter(State.DRIVE_FORWARD, left_ticks, right_ticks)
            return ControlOutput(left=0.0, right=0.0, claw=None, state=self.state)

        if not det.red_found:
            self._consecutive_lost += 1
            n = max(1, self.cfg.lost_frames_before_search)
            if self._consecutive_lost < n:
                # Soft loss: contour flicker — coast straight, keep steering PID state
                lo, hi = sorted((self.cfg.min_speed, self.cfg.max_speed))
                coast = _clamp(self.cfg.min_speed, lo, hi)
                return ControlOutput(
                    left=coast, right=coast, claw=None, state=self.state
                )
            # Hard loss: search spin toward last known line side (see _search_spin_pair)
            self._steer_pid.reset()
            st = self._clamp_search_turn()
            left, right = self._search_spin_pair(st)
            return ControlOutput(left=left, right=right, claw=None, state=self.state)

        self._consecutive_lost = 0
        self._last_red_error = det.red_error
        left, right = self._steer(det.red_error)
        return ControlOutput(left=left, right=right, claw=None, state=self.state)

    def _drive_forward(self, left_ticks: int, right_ticks: int) -> ControlOutput:
        avg_delta = ((left_ticks - self._enc0_left) + (right_ticks - self._enc0_right)) // 2
        if abs(avg_delta) >= self.cfg.forward_ticks:
            self._enter(State.PICKUP)
            return ControlOutput(left=0.0, right=0.0, claw=None, state=self.state)
        spd = min(self.cfg.forward_speed, self.cfg.max_speed)
        return ControlOutput(left=spd, right=spd, claw=None, state=self.state)

    def _pickup(self) -> ControlOutput:
        if time.time() - self._t0 >= self.cfg.pickup_hold_s:
            self._enter(State.TURN_180)
        return ControlOutput(left=0.0, right=0.0, claw=self.cfg.claw_closed, state=self.state)

    def _turn_180(self) -> ControlOutput:
        if time.time() - self._t0 >= self.cfg.turn_duration_s:
            self._enter(State.RETURN)
            return ControlOutput(left=0.0, right=0.0, claw=None, state=self.state)
        spd = min(self.cfg.turn_speed, self.cfg.max_speed)
        return ControlOutput(left=spd, right=-spd, claw=None, state=self.state)

    def _return_follow(self, det: FrameDetection) -> ControlOutput:
        if det.t_junction:
            self._enter(State.DONE)
            return ControlOutput(left=0.0, right=0.0, claw=None, state=self.state)

        if not det.red_found:
            self._consecutive_lost += 1
            n = max(1, self.cfg.lost_frames_before_search)
            if self._consecutive_lost < n:
                lo, hi = sorted((self.cfg.min_speed, self.cfg.max_speed))
                coast = _clamp(self.cfg.min_speed, lo, hi)
                return ControlOutput(
                    left=coast, right=coast, claw=None, state=self.state
                )
            self._steer_pid.reset()
            st = self._clamp_search_turn()
            left, right = self._search_spin_pair(st)
            return ControlOutput(left=left, right=right, claw=None, state=self.state)

        self._consecutive_lost = 0
        self._last_red_error = det.red_error
        left, right = self._steer(det.red_error)
        return ControlOutput(left=left, right=right, claw=None, state=self.state)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _clamp_search_turn(self) -> float:
        m = self.cfg.search_turn_max
        return _clamp(self.cfg.search_turn, -m, m)

    def _search_spin_pair(self, st: float) -> tuple[float, float]:
        """
        In-place spin while the line is lost.

        Perception: red_error > 0 means the line was to the *right* of frame centre.
        To sweep back toward that side, rotate clockwise: left forward, right reverse.
        If the line was last seen on the *left* (negative error), rotate the other way.

        A fixed CW spin (always left=+st, right=-st) often looks like it turns *away*
        from the line when the line left the FOV on the left.
        """
        if self._last_red_error >= 0.0:
            return st, -st
        return -st, st

    def _scale_pair_to_max_speed(self, left: float, right: float) -> tuple[float, float]:
        """
        Limit both wheels to ±max_speed without changing the turn / forward *mix*.

        Clamping each wheel independently (old behaviour) made ``fwd ± turn`` collapse
        toward full ±max_speed whenever steering was active, so base/min/max speed
        looked “ignored” as soon as Kp > 0.
        """
        ms = self.cfg.max_speed
        m = max(abs(left), abs(right))
        if m <= 1e-12:
            return left, right
        if m <= ms:
            return left, right
        s = ms / m
        return left * s, right * s

    def _steer(self, error: float) -> tuple[float, float]:
        """
        Steering PID output:
          error > 0 → line is right of centre → steer right (left faster, right slower)
          error < 0 → line is left  of centre → steer left

        Forward speed scales down with |error| so the robot slows into sharp turns.

        ``fwd ± turn`` is scaled down uniformly if either wheel would exceed ``max_speed``,
        so base/min/max still matter when steering is strong.
        """
        error = _clamp(error, -1.0, 1.0)
        turn  = self._steer_pid(error)   # simple-pid: call with measurement, returns output

        speed_scale = 1.0 - abs(error)
        fwd = self.cfg.min_speed + (self.cfg.base_speed - self.cfg.min_speed) * speed_scale
        lo, hi = sorted((self.cfg.min_speed, self.cfg.max_speed))
        fwd = _clamp(fwd, lo, hi)

        left = _clamp(fwd + turn, -1.0, 1.0)
        right = _clamp(fwd - turn, -1.0, 1.0)
        return self._scale_pair_to_max_speed(left, right)

    def _enter(self, new_state: State, left_ticks: int = 0, right_ticks: int = 0) -> None:
        print(f"[STATE] {self.state.value} → {new_state.value}", flush=True)
        self.state       = new_state
        self._t0         = time.time()
        self._enc0_left  = left_ticks
        self._enc0_right = right_ticks
        self._consecutive_lost = 0
        self._steer_pid.reset()
