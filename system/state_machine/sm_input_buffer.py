import time
from threading import Lock
from typing import Any

from sm_models import Inputs, Vector
from sm_parse import parse_finite_float

class InputBuffer:
    def __init__(self, target_align_threshold: float, place_align_threshold: float):
        self._lock = Lock()
        self._payload: dict[str, Any] = {}
        self._inputs = Inputs()
        self._updated_at: float | None = None
        self._target_align_threshold = target_align_threshold
        self._place_align_threshold = place_align_threshold

    def update(self, payload: dict[str, Any]) -> Inputs:
        if not isinstance(payload, dict):
            raise ValueError("Payload must be a JSON object.")
        inputs = self._inputs_from_payload(payload)

        with self._lock:
            self._payload = dict(payload)
            self._inputs = inputs
            self._updated_at = time.time()

        return inputs

    def snapshot(self) -> tuple[Inputs, dict[str, Any], float | None]:
        with self._lock:
            return (
                self._inputs,
                dict(self._payload),
                self._updated_at,
            )

    def latest_inputs(self) -> Inputs:
        with self._lock:
            return self._inputs

    def clear(self) -> None:
        with self._lock:
            self._payload = {}
            self._inputs = Inputs()
            self._updated_at = None

    def _inputs_from_payload(self, payload: dict[str, Any]) -> Inputs:
        safe_zone = Vector.from_dict(payload.get("safe_zone"), "safe_zone")
        danger_zone = Vector.from_dict(payload.get("danger_zone"), "danger_zone")
        target = Vector.from_dict(payload.get("target"), "target")
        has_line_error = "line_error" in payload
        line_error = Vector.from_dict(payload.get("line_error"), "line_error")
        home = Vector.from_dict(payload.get("home"), "home")
        heading_rad = parse_finite_float(
            payload.get("heading_rad", payload.get("heading", 0.0)),
            "heading_rad",
        )
        speed = parse_finite_float(
            payload.get("speed", payload.get("current_speed", 0.0)),
            "speed",
        )
        # Backward compatibility: robot_mock now sends `home` instead of `line`.
        line_payload = payload.get("line")
        if line_payload is None:
            line_payload = payload.get("home")
        _line = Vector.from_dict(line_payload, "line")
        # If `line_error` is explicitly provided by simulator/CV, it is the
        # source of truth for red-line detection (including detected=false).
        # Legacy `line/home` fallback is only for payloads that don't include
        # `line_error` at all.
        if has_line_error:
            _line = line_error

        return Inputs(
            lego_detected=target.detected,
            target_detected=target.detected,
            safe_zone_detected=safe_zone.detected,
            danger_zone_detected=danger_zone.detected,
            aligned_for_retrieve=target.detected
            and target.magnitude <= self._target_align_threshold,
            aligned_for_place=safe_zone.detected
            and safe_zone.magnitude <= self._place_align_threshold,
            placing=bool(payload.get("placing", False)),
            pick_up_success=bool(payload.get("pick_up_success", False)),
            place_success=bool(payload.get("place_success", False)),
            failed_pickup=bool(payload.get("failed_pickup", False)),
            at_home=bool(payload.get("at_home", False)),
            heading_rad=heading_rad,
            speed=speed,
            home=home,
            danger_zone=danger_zone,
            target=target,
            safe_zone=safe_zone,
            red_line=_line,
            line_error=line_error,
        )
