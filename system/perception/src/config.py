"""Configuration dataclasses and YAML loading."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


def _as_tuple3(value: list[int] | tuple[int, int, int]) -> tuple[int, int, int]:
    if len(value) != 3:
        raise ValueError("HSV bounds must have exactly 3 values")
    return int(value[0]), int(value[1]), int(value[2])


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


@dataclass
class HoughConfig:
    canny_threshold1: int = 50
    canny_threshold2: int = 150
    rho: float = 1.0
    theta_deg: float = 1.0
    threshold: int = 25
    min_line_length: int = 40
    max_line_gap: int = 20
    min_abs_slope: float = 0.3
    center_weight: float = 0.2


@dataclass
class SmoothingConfig:
    window_size: int = 7
    ema_alpha_path: float = 0.8
    ema_alpha_error: float = 0.85
    max_error_step: float = 0.2
    freeze_on_miss_frames: int = 3
    segment_lock_frames: int = 3
    segment_switch_score_margin: float = 1.2
    segment_max_missed_frames: int = 2
    centerline_lookahead_ratio: float = 0.35
    centerline_ema_alpha: float = 0.8

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
class AppConfig:
    fps: float = 30.0
    roi_y_start: int = 240
    alpha: float = 0.9
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
    hough: HoughConfig = field(default_factory=HoughConfig)
    smoothing: SmoothingConfig = field(default_factory=SmoothingConfig)
    zones: ZoneConfig = field(default_factory=ZoneConfig)
    confidence: ConfidenceConfig = field(default_factory=ConfidenceConfig)
    comms: CommsConfig = field(default_factory=CommsConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "AppConfig":
        cfg = cls()
        if "fps" in data:
            cfg.fps = float(data["fps"])
        if "roi_y_start" in data:
            cfg.roi_y_start = int(data["roi_y_start"])
        if "alpha" in data:
            cfg.alpha = float(data["alpha"])
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
            cfg.heading = HeadingConfig(**data["heading"])
        if "hough" in data:
            cfg.hough = HoughConfig(**data["hough"])
        if "smoothing" in data:
            cfg.smoothing = SmoothingConfig(**data["smoothing"])
        if "zones" in data:
            cfg.zones = ZoneConfig(**data["zones"])
        if "confidence" in data:
            cfg.confidence = ConfidenceConfig(**data["confidence"])
        if "comms" in data:
            cfg.comms = CommsConfig(**data["comms"])
        if "camera" in data:
            cfg.camera = CameraConfig(**data["camera"])
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
