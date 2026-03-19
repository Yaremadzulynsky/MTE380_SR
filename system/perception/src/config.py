"""Configuration dataclasses and YAML loading."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


VALID_PATH_MASK_KEYS = ("red", "blue", "black")


def _as_tuple3(value: list[int] | tuple[int, int, int]) -> tuple[int, int, int]:
    if len(value) != 3:
        raise ValueError("HSV bounds must have exactly 3 values")
    return int(value[0]), int(value[1]), int(value[2])


def _as_ratio(value: Any, name: str) -> float:
    parsed = float(value)
    if parsed < 0.0 or parsed > 1.0:
        raise ValueError(f"{name} must be within [0, 1]")
    return parsed


def _as_nonnegative(value: Any, name: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return parsed


def _as_path_mask_key(value: Any) -> str:
    key = str(value).strip().lower()
    if key not in VALID_PATH_MASK_KEYS:
        raise ValueError(f"path_mask_key must be one of {', '.join(VALID_PATH_MASK_KEYS)}")
    return key


@dataclass
class HSVRange:
    lo: tuple[int, int, int]
    hi: tuple[int, int, int]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "HSVRange":
        return cls(lo=_as_tuple3(data["lo"]), hi=_as_tuple3(data["hi"]))


@dataclass
class MorphConfig:
    kernel_size: int = 5
    open_iters: int = 1
    close_iters: int = 1


@dataclass
class HeadingConfig:
    min_area: float = 150.0
    use_centerline: bool = True
    probe_x_frac: float = 0.5
    probe_y_frac: float = 0.9
    lookahead_y_frac: float = 0.72
    forward_bias: float = 1.0
    lateral_gain: float = 0.5
    max_lateral_abs: float = 0.6


@dataclass
class ZoneConfig:
    target_min_area: float = 300.0
    target_min_circularity: float = 0.6
    danger_mode: str = "either"  # ratio | area | either
    danger_ratio_thresh: float = 0.08
    danger_area_thresh: float = 4000.0
    path_ratio_thresh: float = 0.03
    path_area_thresh: float = 1500.0
    safe_green_required: bool = False
    green_ratio_thresh: float = 0.02


@dataclass
class ConfidenceConfig:
    expected_area: float = 6000.0


@dataclass
class CommsConfig:
    method: str = "udp"  # udp | serial | stdout | http
    zone_encoding: str = "string"  # string | int
    udp_ip: str = "127.0.0.1"
    udp_port: int = 5005
    serial_port: str = "/dev/ttyUSB0"
    serial_baud: int = 115200
    http_url: str = ""  # e.g. http://100.72.60.28:8000/inputs


@dataclass
class CameraConfig:
    source: str = "webcam"
    webcam_index: int = 0
    width: int = 640
    height: int = 480
    backend: str = "auto"  # auto | gstreamer | ffmpeg
    gstreamer_device: str | None = None  # e.g. /dev/video0; overrides webcam_index when set


@dataclass
class DebugStreamConfig:
    enabled: bool = False
    host: str = "0.0.0.0"
    port: int = 8081
    jpeg_quality: int = 80


@dataclass
class AppConfig:
    fps: float = 30.0
    roi_y_start: int = 240
    roi_y_start_ratio: float | None = None
    alpha: float = 0.9
    path_mask_key: str = "red"
    show_masks: bool = True
    red1: HSVRange = field(
        default_factory=lambda: HSVRange(lo=(0, 120, 80), hi=(10, 255, 255))
    )
    red2: HSVRange = field(
        default_factory=lambda: HSVRange(lo=(170, 120, 80), hi=(179, 255, 255))
    )
    green: HSVRange = field(
        default_factory=lambda: HSVRange(lo=(35, 60, 60), hi=(85, 255, 255))
    )
    blue: HSVRange = field(
        default_factory=lambda: HSVRange(lo=(95, 80, 80), hi=(130, 255, 255))
    )
    black: HSVRange = field(
        default_factory=lambda: HSVRange(lo=(0, 0, 0), hi=(179, 255, 80))
    )
    danger: HSVRange = field(
        default_factory=lambda: HSVRange(lo=(0, 0, 0), hi=(179, 60, 90))
    )
    morph: MorphConfig = field(default_factory=MorphConfig)
    heading: HeadingConfig = field(default_factory=HeadingConfig)
    zones: ZoneConfig = field(default_factory=ZoneConfig)
    confidence: ConfidenceConfig = field(default_factory=ConfidenceConfig)
    comms: CommsConfig = field(default_factory=CommsConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    debug_stream: DebugStreamConfig = field(default_factory=DebugStreamConfig)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "AppConfig":
        cfg = cls()
        if "fps" in data:
            cfg.fps = float(data["fps"])
        if "roi_y_start" in data:
            cfg.roi_y_start = int(data["roi_y_start"])
        if "roi_y_start_ratio" in data:
            cfg.roi_y_start_ratio = _as_ratio(data["roi_y_start_ratio"], "roi_y_start_ratio")
        if "alpha" in data:
            cfg.alpha = float(data["alpha"])
        if "path_mask_key" in data:
            cfg.path_mask_key = _as_path_mask_key(data["path_mask_key"])
        if "show_masks" in data:
            cfg.show_masks = bool(data["show_masks"])
        if "red1" in data:
            cfg.red1 = HSVRange.from_dict(data["red1"])
        if "red2" in data:
            cfg.red2 = HSVRange.from_dict(data["red2"])
        if "green" in data:
            cfg.green = HSVRange.from_dict(data["green"])
        if "blue" in data:
            cfg.blue = HSVRange.from_dict(data["blue"])
        if "black" in data:
            cfg.black = HSVRange.from_dict(data["black"])
        if "danger" in data:
            cfg.danger = HSVRange.from_dict(data["danger"])
        if "morph" in data:
            cfg.morph = MorphConfig(**data["morph"])
        if "heading" in data:
            heading_data = dict(data["heading"])
            if "forward_y" in heading_data and "forward_bias" not in heading_data:
                heading_data["forward_bias"] = heading_data.pop("forward_y")
            cfg.heading = HeadingConfig(**heading_data)
        if "zones" in data:
            cfg.zones = ZoneConfig(**data["zones"])
        if "confidence" in data:
            cfg.confidence = ConfidenceConfig(**data["confidence"])
        if "comms" in data:
            cfg.comms = CommsConfig(**data["comms"])
        if "camera" in data:
            cfg.camera = CameraConfig(**data["camera"])
        if "debug_stream" in data:
            cfg.debug_stream = DebugStreamConfig(**data["debug_stream"])
        cfg.heading.probe_x_frac = _as_ratio(cfg.heading.probe_x_frac, "heading.probe_x_frac")
        cfg.heading.probe_y_frac = _as_ratio(cfg.heading.probe_y_frac, "heading.probe_y_frac")
        cfg.heading.lookahead_y_frac = _as_ratio(cfg.heading.lookahead_y_frac, "heading.lookahead_y_frac")
        cfg.heading.forward_bias = _as_ratio(cfg.heading.forward_bias, "heading.forward_bias")
        cfg.heading.lateral_gain = _as_nonnegative(cfg.heading.lateral_gain, "heading.lateral_gain")
        cfg.heading.max_lateral_abs = _as_ratio(cfg.heading.max_lateral_abs, "heading.max_lateral_abs")
        cfg.path_mask_key = _as_path_mask_key(cfg.path_mask_key)
        return cfg


def load_config(path: str | Path) -> AppConfig:
    """Load YAML config. If missing, return defaults."""
    p = Path(path)
    if not p.exists():
        return AppConfig()
    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError("Top-level YAML config must be a mapping")
    return AppConfig.from_dict(data)
