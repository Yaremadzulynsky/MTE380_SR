"""Packet schema and JSON encoding."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json


ZONE_TO_INT = {
    "SAFE": 0,
    "PATH": 1,
    "DANGER": 2,
    "TARGET": 3,
}


@dataclass
class PerceptionPacket:
    px: float
    py: float
    zone: str
    gamma: float
    t: float
    line_error_x: float = 0.0
    line_error_y: float = 1.0
    path_detected: bool = False
    path_mask_key: str = "red"
    target_detected: bool = False
    target_px: float = 0.0
    target_py: float = 0.0

    def to_dict(self, zone_encoding: str = "string") -> dict[str, float | int | str | bool]:
        data = asdict(self)
        if zone_encoding == "int":
            data["zone"] = ZONE_TO_INT.get(self.zone, -1)
        return data

    def to_json(self, zone_encoding: str = "string") -> str:
        return json.dumps(self.to_dict(zone_encoding=zone_encoding), separators=(",", ":"))
