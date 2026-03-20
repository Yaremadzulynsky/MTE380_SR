#!/usr/bin/env python3
"""Standalone Prometheus exporter for robot control/state data.

This service accepts control/state updates over HTTP and exposes only
Prometheus metrics for scraping. It does not write to the IPC metrics
aggregator, logs pipeline, or any external database.

Example usage:
  python3 prometheus_exporter.py

Example update calls:
  curl -X POST http://localhost:9101/command \
    -H 'Content-Type: application/json' \
    -d '{"x": 0.2, "y": 0.8, "speed": 0.45, "servo_deg": 30}'

  curl -X POST http://localhost:9101/feedback \
    -H 'Content-Type: application/json' \
    -d '{"heading_rad": 0.14, "linear_speed_mps": 0.21, "enc_left_ticks": 100, "enc_right_ticks": 104}'

  curl -X POST http://localhost:9101/state \
    -H 'Content-Type: application/json' \
    -d '{"state": "searching", "source_state": "remote_control", "transition_label": "manual_start"}'
"""

from __future__ import annotations

import math
import os
import time
from typing import Any

from flask import Flask, Response, jsonify, request
from prometheus_client import CONTENT_TYPE_LATEST, Counter, Gauge, generate_latest


HOST = (os.getenv("HOST") or "0.0.0.0").strip()
PORT = int((os.getenv("PORT") or "9101").strip())

app = Flask(__name__)


def _now_unix() -> float:
    return time.time()


def _parse_finite_number(payload: dict[str, Any], key: str) -> tuple[float | None, str | None]:
    value = payload.get(key)
    if value is None:
        return None, None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None, f"'{key}' must be numeric."
    value = float(value)
    if not math.isfinite(value):
        return None, f"'{key}' must be finite."
    return value, None


def _parse_string(payload: dict[str, Any], key: str) -> tuple[str | None, str | None]:
    value = payload.get(key)
    if value is None:
        return None, None
    if not isinstance(value, str):
        return None, f"'{key}' must be a string."
    value = value.strip()
    if not value:
        return None, f"'{key}' must not be empty."
    return value, None


def _validate_json_object() -> tuple[dict[str, Any] | None, Response | None, int | None]:
    if not request.is_json:
        return None, jsonify({"ok": False, "message": "Expected JSON payload."}), 400
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return None, jsonify({"ok": False, "message": "JSON payload must be an object."}), 400
    return payload, None, None


EXPORTER_INFO = Gauge(
    "robot_exporter_info",
    "Static info for the robot Prometheus exporter.",
    ["service"],
)

COMMAND_X = Gauge("robot_command_x", "Latest commanded x axis.")
COMMAND_Y = Gauge("robot_command_y", "Latest commanded y axis.")
COMMAND_SPEED = Gauge("robot_command_speed", "Latest commanded speed.")
COMMAND_SERVO_DEG = Gauge("robot_command_servo_deg", "Latest commanded servo angle in degrees.")
COMMAND_TARGET_HEADING_RAD = Gauge(
    "robot_command_target_heading_rad",
    "Latest commanded target heading in radians.",
)
COMMAND_LEFT = Gauge("robot_command_left", "Latest commanded left motor value.")
COMMAND_RIGHT = Gauge("robot_command_right", "Latest commanded right motor value.")
COMMAND_UPDATED_AT = Gauge("robot_command_updated_at_unixtime", "Unix time of the latest command update.")
COMMANDS_TOTAL = Counter("robot_commands_total", "Total number of accepted command updates.")

FEEDBACK_HEADING_RAD = Gauge("robot_feedback_heading_rad", "Latest measured heading in radians.")
FEEDBACK_LINEAR_SPEED_MPS = Gauge(
    "robot_feedback_linear_speed_mps",
    "Latest measured forward speed in metres per second.",
)
FEEDBACK_LEFT_ENCODER_TICKS = Gauge(
    "robot_feedback_left_encoder_ticks",
    "Latest left encoder tick count.",
)
FEEDBACK_RIGHT_ENCODER_TICKS = Gauge(
    "robot_feedback_right_encoder_ticks",
    "Latest right encoder tick count.",
)
FEEDBACK_UPDATED_AT = Gauge(
    "robot_feedback_updated_at_unixtime",
    "Unix time of the latest feedback update.",
)
FEEDBACK_UPDATES_TOTAL = Counter(
    "robot_feedback_updates_total",
    "Total number of accepted feedback updates.",
)

CURRENT_STATE = Gauge(
    "robot_current_state",
    "Current state machine state. Exactly one state label should be 1.",
    ["state"],
)
STATE_UPDATED_AT = Gauge("robot_state_updated_at_unixtime", "Unix time of the latest state update.")
STATE_UPDATES_TOTAL = Counter("robot_state_updates_total", "Total number of accepted state updates.")
STATE_TRANSITIONS_TOTAL = Counter(
    "robot_state_transitions_total",
    "Total number of reported state transitions.",
    ["source_state", "target_state", "transition_label"],
)


KNOWN_STATES = {
    "searching_demo",
    "pickup",
    "remote_control",
    "searching",
    "find_target",
    "align_for_retrieve",
    "retrieving",
    "retrieved",
    "transporting",
    "align_for_place",
    "placing",
    "place_success",
    "return_home",
    "error_retrieve",
    "error_place",
    "end",
}


def _set_current_state(state: str) -> None:
    for known_state in KNOWN_STATES | {state}:
        CURRENT_STATE.labels(state=known_state).set(1 if known_state == state else 0)


@app.get("/health")
def health() -> Any:
    return jsonify({"ok": True, "status": "ok"})


@app.get("/metrics")
def metrics() -> Response:
    return Response(generate_latest(), mimetype=CONTENT_TYPE_LATEST)


@app.post("/command")
def post_command() -> Any:
    payload, response, status_code = _validate_json_object()
    if response is not None:
        return response, status_code

    field_errors: dict[str, str] = {}
    values: dict[str, float] = {}
    for key in (
        "x",
        "y",
        "speed",
        "servo_deg",
        "target_heading_rad",
        "left",
        "right",
    ):
        value, error = _parse_finite_number(payload, key)
        if error:
            field_errors[key] = error
        elif value is not None:
            values[key] = value

    if field_errors:
        return jsonify({"ok": False, "message": "Invalid command payload.", "errors": field_errors}), 400

    if "x" in values:
        COMMAND_X.set(values["x"])
    if "y" in values:
        COMMAND_Y.set(values["y"])
    if "speed" in values:
        COMMAND_SPEED.set(values["speed"])
    if "servo_deg" in values:
        COMMAND_SERVO_DEG.set(values["servo_deg"])
    if "target_heading_rad" in values:
        COMMAND_TARGET_HEADING_RAD.set(values["target_heading_rad"])
    if "left" in values:
        COMMAND_LEFT.set(values["left"])
    if "right" in values:
        COMMAND_RIGHT.set(values["right"])

    COMMAND_UPDATED_AT.set(_now_unix())
    COMMANDS_TOTAL.inc()
    return jsonify({"ok": True, "updated": sorted(values.keys())})


@app.post("/feedback")
def post_feedback() -> Any:
    payload, response, status_code = _validate_json_object()
    if response is not None:
        return response, status_code

    field_errors: dict[str, str] = {}
    values: dict[str, float] = {}
    for key in (
        "heading_rad",
        "linear_speed_mps",
        "enc_left_ticks",
        "enc_right_ticks",
    ):
        value, error = _parse_finite_number(payload, key)
        if error:
            field_errors[key] = error
        elif value is not None:
            values[key] = value

    if field_errors:
        return jsonify({"ok": False, "message": "Invalid feedback payload.", "errors": field_errors}), 400

    if "heading_rad" in values:
        FEEDBACK_HEADING_RAD.set(values["heading_rad"])
    if "linear_speed_mps" in values:
        FEEDBACK_LINEAR_SPEED_MPS.set(values["linear_speed_mps"])
    if "enc_left_ticks" in values:
        FEEDBACK_LEFT_ENCODER_TICKS.set(values["enc_left_ticks"])
    if "enc_right_ticks" in values:
        FEEDBACK_RIGHT_ENCODER_TICKS.set(values["enc_right_ticks"])

    FEEDBACK_UPDATED_AT.set(_now_unix())
    FEEDBACK_UPDATES_TOTAL.inc()
    return jsonify({"ok": True, "updated": sorted(values.keys())})


@app.post("/state")
def post_state() -> Any:
    payload, response, status_code = _validate_json_object()
    if response is not None:
        return response, status_code

    state, state_error = _parse_string(payload, "state")
    source_state, source_error = _parse_string(payload, "source_state")
    transition_label, transition_error = _parse_string(payload, "transition_label")

    errors: dict[str, str] = {}
    if state_error:
        errors["state"] = state_error
    if source_error:
        errors["source_state"] = source_error
    if transition_error:
        errors["transition_label"] = transition_error
    if errors:
        return jsonify({"ok": False, "message": "Invalid state payload.", "errors": errors}), 400

    if state is not None:
        _set_current_state(state)
        STATE_UPDATED_AT.set(_now_unix())
        STATE_UPDATES_TOTAL.inc()

    if state is not None and source_state is not None:
        STATE_TRANSITIONS_TOTAL.labels(
            source_state=source_state,
            target_state=state,
            transition_label=transition_label or "unspecified",
        ).inc()

    return jsonify(
        {
            "ok": True,
            "state": state,
            "transition_recorded": state is not None and source_state is not None,
        }
    )


if __name__ == "__main__":
    EXPORTER_INFO.labels(service="robot_prometheus_exporter").set(1)
    app.run(host=HOST, port=PORT)
