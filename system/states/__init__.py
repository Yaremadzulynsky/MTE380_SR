"""Shared types and helpers used by all state modules."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class State(Enum):
    LINE_FOLLOW   = "LINE_FOLLOW"
    FIND_LINE     = "FIND_LINE"
    PID_TURN      = "PID_TURN"
    DRIVE_FORWARD = "DRIVE_FORWARD"
    PICKUP        = "PICKUP"
    TURN_180      = "TURN_180"
    RETURN        = "RETURN"
    DONE          = "DONE"



@dataclass
class ControlOutput:
    left:           float
    right:          float
    claw:           float | None  # servo angle in degrees, or None = leave unchanged
    state:          State
    direct_voltage: bool = False  # bypass motor speed PID, send voltages directly
    skip:           bool = False  # controller already sent voltages — brain skips motor apply


# ── Shared helpers (take sm = MissionStateMachine as context) ─────────────────

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def scale_pair_to_max_speed(sm, left: float, right: float) -> tuple[float, float]:
    ms = sm.cfg.max_speed
    m = max(abs(left), abs(right))
    if m <= 1e-12 or m <= ms:
        return left, right
    s = ms / m
    return left * s, right * s


def clamp_search_turn(sm) -> float:
    m = sm.cfg.search_turn_max
    return _clamp(sm.cfg.search_turn, -m, m)


def search_spin_pair(sm, st: float) -> tuple[float, float]:
    if sm._last_red_error >= 0.0:
        return st, -st
    return -st, st


def steer(sm, error: float, base_speed: float | None = None) -> tuple[float, float]:
    error = _clamp(error, -1.0, 1.0)
    turn  = sm._steer_pid(-error)

    _base = base_speed if base_speed is not None else sm.cfg.base_speed
    speed_scale = 1.0 - abs(error)
    fwd = sm.cfg.min_speed + (_base - sm.cfg.min_speed) * speed_scale
    lo, hi = sorted((sm.cfg.min_speed, sm.cfg.max_speed))
    fwd = _clamp(fwd, lo, hi)

    left  = _clamp(fwd + turn, -1.0, 1.0)
    right = _clamp(fwd - turn, -1.0, 1.0)
    return scale_pair_to_max_speed(sm, left, right)
