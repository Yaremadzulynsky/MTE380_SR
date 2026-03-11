from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Any

from sm_parse import parse_finite_float


class State(Enum):
    SEARCHING_DEMO = "testing"
    PICKUP = "pickup"
    REMOTE_CONTROL = "remote_control"
    SEARCHING = "searching"
    FIND_TARGET = "find_target"
    ALIGN_FOR_RETRIEVE = "align_for_retrieve"
    RETRIEVING = "retrieving"
    RETRIEVED = "retrieved"
    TRANSPORTING = "transporting"
    ALIGN_FOR_PLACE = "align_for_place"
    PLACING = "placing"
    PLACE_SUCCESS = "place_success"
    RETURN_HOME = "return_home"
    ERROR_RETRIEVE = "error_retrieve"
    ERROR_PLACE = "error_place"
    END = "end"


class Vector:
    detected: bool
    x: float
    y: float

    @property
    def magnitude(self) -> float:
        return (self.x * self.x + self.y * self.y) ** 0.5

    @property
    def heading(self) -> float:
        if self.magnitude <= 1e-9:
            return 0.0
        # Robot-frame heading where 0 is forward (+y) and +angle is right (+x).
        return math.atan2(self.x, self.y)

    def __init__(self, detected: bool, x: float, y: float) -> None:
        self.detected = detected
        self.x = x
        self.y = y
        
    def to_dict(self) -> dict[str, Any]:
        return {
            "detected": self.detected,
            "x": self.x,
            "y": self.y,
            "heading": self.heading,
            "magnitude": self.magnitude,
        }
    
    @classmethod
    def from_dict(cls, data: dict[str, Any] | None, name: str = "vector") -> "Vector":
        data = data or {}
        if not isinstance(data, dict):
            raise ValueError(f"{name} must be an object.")
        detected_value = data.get("detected")
        has_xy = ("x" in data) or ("y" in data)
        if detected_value is None:
            detected_value = has_xy
            for key, value in data.items():
                if "detected" in str(key).lower():
                    detected_value = value
                    break
        detected = bool(detected_value)
        vector = data.get("vector")
        if vector is None:
            vector = {}
        elif not isinstance(vector, dict):
            raise ValueError(f"{name}.vector must be an object.")
        if not has_xy:
            has_xy = ("x" in vector) or ("y" in vector)
        if data.get("detected") is None and has_xy:
            detected = True
        heading_value = vector.get("heading", data.get("heading"))
        magnitude_value = vector.get("magnitude", data.get("magnitude"))
        has_polar = heading_value is not None and magnitude_value is not None
        if has_polar and not has_xy:
            heading = parse_finite_float(heading_value, f"{name}.heading")
            magnitude = parse_finite_float(magnitude_value, f"{name}.magnitude")
            if magnitude < 0:
                raise ValueError(f"{name}.magnitude must be >= 0.")
            x = math.sin(heading) * magnitude
            y = math.cos(heading) * magnitude
        else:
            x = parse_finite_float(vector.get("x", data.get("x", 0.0)), f"{name}.x")
            y = parse_finite_float(vector.get("y", data.get("y", 0.0)), f"{name}.y")
        return cls(detected=detected, x=x, y=y)


@dataclass
class Inputs:
    lego_detected: bool = False
    target_detected: bool = False
    safe_zone_detected: bool = False
    danger_zone_detected: bool = False
    aligned_for_retrieve: bool = False
    aligned_for_place: bool = False
    # retrieving: bool = False
    placing: bool = False
    pick_up_success: bool = False
    place_success: bool = False
    failed_pickup: bool = False
    at_home: bool = False
    heading_rad: float = 0.0
    speed: float = 0.0
    red_line: Vector = field(default_factory=lambda: Vector(detected=False, x=0.0, y=0.0))
    line_error: Vector = field(default_factory=lambda: Vector(detected=False, x=0.0, y=0.0))
    home: Vector = field(default_factory=lambda: Vector(detected=False, x=0.0, y=0.0))
    danger_zone: Vector = field(default_factory=lambda: Vector(detected=False, x=0.0, y=0.0))
    target: Vector = field(default_factory=lambda: Vector(detected=False, x=0.0, y=0.0))
    safe_zone: Vector = field(default_factory=lambda: Vector(detected=False, x=0.0, y=0.0))




@dataclass
class TransitionResult:
    source: State
    target: State
    label: str


@dataclass
class StateContext:
    failed_pickups: int = 0
    transition_count: int = 0
    last_transition_at: float = 0.0
