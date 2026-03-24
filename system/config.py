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
    # ── Line follow mode ──────────────────────────────────────────────────────
    line_follow_mode: int = 0        # 0=simple  1=find_line  2=pid_turn  3=adaptive
    simple_curv_thresh: float = 0.5  # curvature at which simple mode reaches min_speed
    adaptive_curv_a: float = 0.3        # adaptive mode: single gain for speed reduction and horizontal bias
    curv_speed_bias: float = 1.0    # power applied to normalised curvature before speed scaling (1=linear, >1=faster drop)

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
    base_speed: float = 0.28
    min_speed:  float = 0.16
    max_speed:  float = 0.45

    # ── Drive-forward ─────────────────────────────────────────────────────────
    forward_ticks: int   = 800
    forward_speed: float = 0.25

    # ── 180-degree turn ───────────────────────────────────────────────────────
    turn_speed:      float = 0.30
    turn_duration_s: float = 2.2

    # ── Controllers (rotation) ────────────────────────────────────────────────
    wheel_diameter_m: float = 0.065  # wheel outer diameter (metres)
    wheelbase_m:      float = 0.155  # centre-to-centre track width (metres)
    rot_kp:           float = 0.5    # rotation PID — proportional gain (radian-space)
    rot_ki:           float = 0.0    # rotation PID — integral gain
    rot_kd:           float = 0.0    # rotation PID — derivative gain
    rot_tolerance:    float = 3.0    # done when within this many degrees of target
    rot_motor_deadband: float = 0.05 # minimum motor voltage (overcomes stiction)

    # ── Controllers (position) ────────────────────────────────────────────────
    pos_kp:           float = 2.0    # position PID — proportional gain
    pos_ki:           float = 0.0    # position PID — integral gain
    pos_kd:           float = 0.1    # position PID — derivative gain
    pos_tolerance_m:  float = 0.01   # done when within this many metres of target
    pos_motor_deadband: float = 0.05 # minimum motor voltage (overcomes stiction)

    # ── Claw / servo ──────────────────────────────────────────────────────────
    claw_open:     float = 0.0
    claw_closed:   float = 90.0
    pickup_hold_s: float = 0.8

    # ── Find line ─────────────────────────────────────────────────────────────
    find_line_turn_speed: float = 0.20  # in-place spin voltage while searching

    # ── Corner turn ───────────────────────────────────────────────────────────
    corner_curvature_thresh: float = 0.3   # |curvature| above this triggers CORNER_TURN
    corner_curve_conf_min:   float = 0.6   # minimum curve_conf required (fraction of strips)
    corner_forward_m:        float = 0.2   # metres to drive forward before rotating
    corner_forward_speed:    float = 0.25  # speed during forward drive phase

    # ── Vision / perception ───────────────────────────────────────────────────
    red_loss_debounce_frames: int   = 4
    red_error_ema_alpha:      float = 0.35
    line_axle_extrap:         float = 0.0
    curve_n_strips:           int   = 5   # horizontal slices for curvature fit
    error_heading_weight:     float = 0.3  # how much local tangent contributes to red_error
    error_curvature_weight:   float = 0.2  # how much curvature (bend ahead) contributes to red_error

    # ── Line detection HSV ────────────────────────────────────────────────────
    red_h_lo1:  int = 0
    red_h_hi1:  int = 10
    red_h_lo2:  int = 161
    red_h_hi2:  int = 179
    red_s_min:  int = 155
    red_v_min:  int = 84
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
    ("Line follow mode", [
        "line_follow_mode", "simple_curv_thresh", "adaptive_curv_a", "curv_speed_bias",
    ]),
    ("Steering PID", [
        "steer_kp", "steer_ki", "steer_kd", "steer_out_limit",
    ]),
    ("Motor PID", [
        "motor_kp", "motor_ki", "motor_kd",
    ]),
    ("Speed", [
        "base_speed", "min_speed", "max_speed",
    ]),
    ("Drive-forward", [
        "forward_ticks", "forward_speed",
    ]),
    ("180-degree turn", [
        "turn_speed", "turn_duration_s",
    ]),
    ("Controllers", [
        "wheel_diameter_m", "wheelbase_m",
        "rot_kp", "rot_ki", "rot_kd", "rot_tolerance", "rot_motor_deadband",
        "pos_kp", "pos_ki", "pos_kd", "pos_tolerance_m", "pos_motor_deadband",
    ]),
    ("Claw / servo", [
        "claw_open", "claw_closed", "pickup_hold_s",
    ]),
    ("Find line", [
        "find_line_turn_speed",
    ]),
    ("Corner turn", [
        "corner_curvature_thresh", "corner_curve_conf_min",
        "corner_forward_m", "corner_forward_speed",
    ]),
    ("Vision / perception", [
        "red_loss_debounce_frames", "red_error_ema_alpha",
        "line_axle_extrap", "curve_n_strips", "error_heading_weight", "error_curvature_weight",
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
