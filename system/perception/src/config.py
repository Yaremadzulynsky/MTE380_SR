from __future__ import annotations

import os
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import yaml


VALID_PATH_MASK_KEYS = {"red", "blue", "black"}


def _ratio(value: Any, name: str) -> float:
    parsed = float(value)
    if parsed < 0.0 or parsed > 1.0:
        raise ValueError(f"{name} must be within [0, 1]")
    return parsed


def _nonnegative(value: Any, name: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise ValueError(f"{name} must be >= 0")
    return parsed


def _tuple3(value: Any, name: str) -> tuple[int, int, int]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain exactly 3 integers")
    return int(value[0]), int(value[1]), int(value[2])


def _mask_key(value: Any) -> str:
    key = str(value).strip().lower()
    if key not in VALID_PATH_MASK_KEYS:
        raise ValueError(f"path_mask_key must be one of {sorted(VALID_PATH_MASK_KEYS)}")
    return key


@dataclass
class HSVRange:
    lo: tuple[int, int, int]
    hi: tuple[int, int, int]

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, name: str) -> "HSVRange":
        return cls(
            lo=_tuple3(data.get("lo", (0, 0, 0)), f"{name}.lo"),
            hi=_tuple3(data.get("hi", (179, 255, 255)), f"{name}.hi"),
        )


@dataclass
class HeadingConfig:
    min_area: float = 150.0
    probe_x_frac: float = 0.5
    probe_y_frac: float = 0.9
    lookahead_y_frac: float = 0.72
    forward_bias: float = 1.0
    lateral_gain: float = 0.5
    max_lateral_abs: float = 0.6
    pid_kp: float = 0.5
    pid_ki: float = 0.0
    pid_kd: float = 0.1
    pid_i_max: float = 0.6
    pid_deadband: float = 0.01
    base_speed: float = 0.35
    min_speed: float = 0.1
    max_speed: float = 0.45
    error_slowdown: float = 0.25


@dataclass
class MorphConfig:
    kernel_size: int = 5
    open_iters: int = 1
    close_iters: int = 1


@dataclass
class OutputConfig:
    method: str = "http"
    url: str = ""
    timeout_s: float = 0.5
    send_hz: float = 10.0


@dataclass
class DebugStreamConfig:
    enabled: bool = False
    host: str = "0.0.0.0"
    port: int = 8081
    jpeg_quality: int = 80
    max_fps: float = 15.0


@dataclass
class AppConfig:
    fps: float = 30.0
    source: str = "webcam"
    width: int = 640
    height: int = 480
    roi_y_start_ratio: float = 0.5
    alpha: float = 0.9
    path_mask_key: str = "red"
    expected_area: float = 6000.0
    heading: HeadingConfig = field(default_factory=HeadingConfig)
    morph: MorphConfig = field(default_factory=MorphConfig)
    output: OutputConfig = field(default_factory=OutputConfig)
    debug_stream: DebugStreamConfig = field(default_factory=DebugStreamConfig)
    red1: HSVRange = field(default_factory=lambda: HSVRange((0, 120, 80), (10, 255, 255)))
    red2: HSVRange = field(default_factory=lambda: HSVRange((170, 120, 80), (179, 255, 255)))
    blue: HSVRange = field(default_factory=lambda: HSVRange((95, 80, 80), (130, 255, 255)))
    black: HSVRange = field(default_factory=lambda: HSVRange((0, 0, 0), (179, 255, 80)))

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "AppConfig":
        cfg = cls()
        if "fps" in data:
            cfg.fps = float(data["fps"])
        if "source" in data:
            cfg.source = str(data["source"])
        if "width" in data:
            cfg.width = int(data["width"])
        if "height" in data:
            cfg.height = int(data["height"])
        if "roi_y_start_ratio" in data:
            cfg.roi_y_start_ratio = _ratio(data["roi_y_start_ratio"], "roi_y_start_ratio")
        if "alpha" in data:
            cfg.alpha = _ratio(data["alpha"], "alpha")
        if "path_mask_key" in data:
            cfg.path_mask_key = _mask_key(data["path_mask_key"])
        if "expected_area" in data:
            cfg.expected_area = _nonnegative(data["expected_area"], "expected_area")

        if "heading" in data:
            heading_data = dict(data["heading"])
            if "forward_y" in heading_data and "forward_bias" not in heading_data:
                heading_data["forward_bias"] = heading_data.pop("forward_y")
            cfg.heading = HeadingConfig(**heading_data)
        if "morph" in data:
            cfg.morph = MorphConfig(**data["morph"])
        if "output" in data:
            cfg.output = OutputConfig(**data["output"])
        if "debug_stream" in data:
            cfg.debug_stream = DebugStreamConfig(**data["debug_stream"])

        hsv = data.get("hsv", {})
        if "red1" in hsv:
            cfg.red1 = HSVRange.from_dict(hsv["red1"], name="hsv.red1")
        if "red2" in hsv:
            cfg.red2 = HSVRange.from_dict(hsv["red2"], name="hsv.red2")
        if "blue" in hsv:
            cfg.blue = HSVRange.from_dict(hsv["blue"], name="hsv.blue")
        if "black" in hsv:
            cfg.black = HSVRange.from_dict(hsv["black"], name="hsv.black")

        cfg.heading.probe_x_frac = _ratio(cfg.heading.probe_x_frac, "heading.probe_x_frac")
        cfg.heading.probe_y_frac = _ratio(cfg.heading.probe_y_frac, "heading.probe_y_frac")
        cfg.heading.lookahead_y_frac = _ratio(cfg.heading.lookahead_y_frac, "heading.lookahead_y_frac")
        cfg.heading.forward_bias = _ratio(cfg.heading.forward_bias, "heading.forward_bias")
        cfg.heading.lateral_gain = _nonnegative(cfg.heading.lateral_gain, "heading.lateral_gain")
        cfg.heading.max_lateral_abs = _ratio(cfg.heading.max_lateral_abs, "heading.max_lateral_abs")
        cfg.heading.pid_i_max = _ratio(cfg.heading.pid_i_max, "heading.pid_i_max")
        cfg.heading.pid_deadband = _ratio(cfg.heading.pid_deadband, "heading.pid_deadband")
        cfg.heading.base_speed = _ratio(cfg.heading.base_speed, "heading.base_speed")
        cfg.heading.min_speed = _ratio(cfg.heading.min_speed, "heading.min_speed")
        cfg.heading.max_speed = _ratio(cfg.heading.max_speed, "heading.max_speed")
        cfg.heading.error_slowdown = _nonnegative(cfg.heading.error_slowdown, "heading.error_slowdown")
        if cfg.heading.min_speed > cfg.heading.max_speed:
            raise ValueError("heading.min_speed must be <= heading.max_speed")
        cfg.path_mask_key = _mask_key(cfg.path_mask_key)
        return cfg

    @property
    def output_url(self) -> str:
        if self.output.url:
            return self.output.url
        base = os.getenv("STATE_MACHINE_BASE_URL", "http://state-machine:8000").rstrip("/")
        path = os.getenv("STATE_MACHINE_INPUT_PATH", "/inputs")
        return f"{base}/{path.lstrip('/')}"


PERCEPTION_TUNING_FIELDS = (
    "pid_kp",
    "pid_ki",
    "pid_kd",
    "pid_i_max",
    "pid_deadband",
    "lookahead_y_frac",
    "lateral_gain",
    "forward_bias",
    "max_lateral_abs",
    "base_speed",
    "min_speed",
    "max_speed",
    "error_slowdown",
)

PERCEPTION_TUNING_RANGES: dict[str, tuple[float, float]] = {
    "pid_kp": (0.0, 20.0),
    "pid_ki": (0.0, 20.0),
    "pid_kd": (0.0, 20.0),
    "pid_i_max": (0.0, 1.0),
    "pid_deadband": (0.0, 1.0),
    "lookahead_y_frac": (0.0, 1.0),
    "lateral_gain": (0.0, 5.0),
    "forward_bias": (0.0, 1.0),
    "max_lateral_abs": (0.0, 1.0),
    "base_speed": (0.0, 1.0),
    "min_speed": (0.0, 1.0),
    "max_speed": (0.0, 1.0),
    "error_slowdown": (0.0, 5.0),
}


def perception_tuning_values(cfg: AppConfig) -> dict[str, float]:
    heading_dict = asdict(cfg.heading)
    return {key: float(heading_dict[key]) for key in PERCEPTION_TUNING_FIELDS}


def perception_tuning_ranges() -> dict[str, dict[str, float]]:
    return {
        key: {"min": bounds[0], "max": bounds[1]}
        for key, bounds in PERCEPTION_TUNING_RANGES.items()
    }


def load_config(path: str | Path) -> AppConfig:
    file_path = Path(path)
    if not file_path.exists():
        return AppConfig()
    with file_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("Config root must be a mapping")
    return AppConfig.from_dict(data)
