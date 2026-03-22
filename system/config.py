"""Single source of truth for all tunable robot parameters.

Usage
─────
    import config

    cfg = config.get()           # cached singleton (loads config.yaml on first call)
    cfg.steer_kp = 0.8           # mutate in-place
    config.save(cfg)             # persist to config.yaml atomically

    cfg = config.reload()        # re-read config.yaml and refresh the singleton
    config.set_path("/tmp/x.yaml")  # switch config file (invalidates cache)
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

import yaml

_CONFIG_PATH = Path(__file__).parent / "config.yaml"


# ── Config dataclass ──────────────────────────────────────────────────────────

@dataclass
class Config:
    # ── Steering PID ─────────────────────────────────────────────────────────
    steer_kp:        float = 0.65
    steer_ki:        float = 0.04
    steer_kd:        float = 0.10
    steer_out_limit: float = 0.80

    # ── Motor PID ─────────────────────────────────────────────────────────────
    motor_kp: float = 0.003125
    motor_ki: float = 0.0005
    motor_kd: float = 0.0

    # ── Speed ─────────────────────────────────────────────────────────────────
    base_speed:                float = 0.28
    min_speed:                 float = 0.16
    max_speed:                 float = 0.45
    search_turn:               float = 0.18
    search_turn_max:           float = 0.32
    lost_frames_before_search: int   = 5

    # ── Drive-forward ─────────────────────────────────────────────────────────
    forward_ticks: int   = 800
    forward_speed: float = 0.25

    # ── 180-degree turn ───────────────────────────────────────────────────────
    turn_speed:      float = 0.30
    turn_duration_s: float = 2.2

    # ── Claw / servo ──────────────────────────────────────────────────────────
    claw_open:     float = 0.0
    claw_closed:   float = 90.0
    pickup_hold_s: float = 0.8

    # ── Vision / perception ───────────────────────────────────────────────────
    geom_enable:              bool  = True
    geom_lateral_norm_m:      float = 0.10
    red_loss_debounce_frames: int   = 4
    red_error_ema_alpha:      float = 0.35
    curve_threshold:          float = 0.15
    curve_coast_s:            float = 0.1
    curve_debounce_s:         float = 1.0

    # ── Line detection HSV ────────────────────────────────────────────────────
    red_h_lo1:  int = 0
    red_h_hi1:  int = 12
    red_h_lo2:  int = 168
    red_h_hi2:  int = 179
    red_s_min:  int = 100
    red_v_min:  int = 70
    blue_h_lo:  int = 95
    blue_h_hi:  int = 135
    blue_s_min: int = 120
    blue_v_min: int = 70

    # ── Camera / runtime ──────────────────────────────────────────────────────
    camera_width:  int   = 640
    camera_height: int   = 480
    fps:           float = 30.0
    roi_top_ratio: float = 0.0


# ── YAML serialisation ────────────────────────────────────────────────────────

_SECTIONS: list[tuple[str, list[str]]] = [
    ("Steering PID", [
        "steer_kp", "steer_ki", "steer_kd", "steer_out_limit",
    ]),
    ("Motor PID", [
        "motor_kp", "motor_ki", "motor_kd",
    ]),
    ("Speed", [
        "base_speed", "min_speed", "max_speed",
        "search_turn", "search_turn_max", "lost_frames_before_search",
    ]),
    ("Drive-forward", [
        "forward_ticks", "forward_speed",
    ]),
    ("180-degree turn", [
        "turn_speed", "turn_duration_s",
    ]),
    ("Claw / servo", [
        "claw_open", "claw_closed", "pickup_hold_s",
    ]),
    ("Vision / perception", [
        "geom_enable", "geom_lateral_norm_m",
        "red_loss_debounce_frames", "red_error_ema_alpha",
        "curve_threshold", "curve_coast_s", "curve_debounce_s",
    ]),
    ("Line detection HSV", [
        "red_h_lo1", "red_h_hi1", "red_h_lo2", "red_h_hi2",
        "red_s_min", "red_v_min",
        "blue_h_lo", "blue_h_hi", "blue_s_min", "blue_v_min",
    ]),
    ("Camera / runtime", [
        "camera_width", "camera_height", "fps", "roi_top_ratio",
    ]),
]


def _to_yaml_str(cfg: Config) -> str:
    """Produce readable YAML with section-header comments."""
    d = asdict(cfg)
    lines: list[str] = []
    for section, keys in _SECTIONS:
        lines.append(f"# {section}")
        for k in keys:
            v = d[k]
            if isinstance(v, bool):
                lines.append(f"{k}: {'true' if v else 'false'}")
            elif isinstance(v, float):
                lines.append(f"{k}: {v!r}")
            else:
                lines.append(f"{k}: {v}")
        lines.append("")
    return "\n".join(lines)


# ── Load / save ───────────────────────────────────────────────────────────────

def load(path: Path = _CONFIG_PATH) -> Config:
    """Load YAML from *path*; unknown or missing keys fall back to dataclass defaults."""
    defaults = Config()

    if not path.exists():
        return Config()

    try:
        raw: dict[str, Any] = yaml.safe_load(path.read_text()) or {}
    except Exception as e:
        print(f"[config] Cannot parse {path}: {e} — using built-in defaults.", flush=True)
        return Config()

    kwargs: dict[str, Any] = {}
    for fld in dataclasses.fields(Config):
        if fld.name not in raw:
            continue
        val = raw[fld.name]
        expected = type(getattr(defaults, fld.name))
        try:
            kwargs[fld.name] = bool(val) if expected is bool else expected(val)
        except (TypeError, ValueError):
            pass  # keep dataclass default

    result = Config(**kwargs)

    # Safety: reject all-zero steering PID (e.g. bad save from web UI)
    if result.steer_kp == result.steer_ki == result.steer_kd == 0:
        print("[config] steer PID all zero — restoring built-in defaults.", flush=True)
        result.steer_kp        = defaults.steer_kp
        result.steer_ki        = defaults.steer_ki
        result.steer_kd        = defaults.steer_kd
        result.steer_out_limit = defaults.steer_out_limit

    if result.motor_kp == 0:
        print("[config] motor_kp is zero — restoring built-in defaults.", flush=True)
        result.motor_kp = defaults.motor_kp
        result.motor_ki = defaults.motor_ki
        result.motor_kd = defaults.motor_kd

    return result


def save(cfg: Config, path: Path = _CONFIG_PATH) -> None:
    """Atomically write *cfg* to YAML."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(_to_yaml_str(cfg))
    tmp.replace(path)


# ── Singleton ─────────────────────────────────────────────────────────────────

_instance: Config | None = None


def get() -> Config:
    """Return the cached Config singleton, loading from disk on first call."""
    global _instance
    if _instance is None:
        _instance = load(_CONFIG_PATH)
    return _instance


def reload() -> Config:
    """Force a fresh load from disk and update the singleton."""
    global _instance
    _instance = load(_CONFIG_PATH)
    return _instance


def set_path(path: Path | str) -> None:
    """Override the config file path and invalidate the cache."""
    global _CONFIG_PATH, _instance
    _CONFIG_PATH = Path(path)
    _instance = None
