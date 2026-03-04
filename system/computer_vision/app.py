import copy
import json
import os
import time
from threading import Lock
from urllib.parse import urljoin

import requests
from flask import Flask, jsonify, request, send_from_directory

DEFAULT_VECTOR = {"detected": False, "vector": {"x": 0.0, "y": 0.0}}
DEFAULT_FLAGS = {
    "retrieving": False,
    "placing": False,
    "pick_up_success": False,
    "place_success": False,
    "failed_pickup": False,
    "at_home": False,
}

STATE_MACHINE_BASE_URL = os.getenv("STATE_MACHINE_BASE_URL", "http://state-machine:8000")
STATE_MACHINE_INPUT_PATH = os.getenv("STATE_MACHINE_INPUT_PATH", "/inputs")
LOG_PATH = os.getenv("LOG_PATH", "/var/log/computer-vision/computer-vision.log")


app = Flask(__name__, static_folder="public", static_url_path="")

_state_lock = Lock()
_state_payload = {
    "safe_zone": copy.deepcopy(DEFAULT_VECTOR),
    "danger_zone": copy.deepcopy(DEFAULT_VECTOR),
    "target": copy.deepcopy(DEFAULT_VECTOR),
    "line": copy.deepcopy(DEFAULT_VECTOR),
    **DEFAULT_FLAGS,
}


class Logger:
    def __init__(self, path: str):
        self._handle = None
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            self._handle = open(path, "a", encoding="utf-8")
        except OSError:
            self._handle = None

    def log(self, event: str, payload: dict):
        entry = {
            "ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "event": event,
            "payload": payload,
        }
        line = json.dumps(entry)
        print(line, flush=True)
        if self._handle:
            self._handle.write(line + "\n")
            self._handle.flush()


LOGGER = Logger(LOG_PATH)


def parse_finite_float(value, field_name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{field_name} must be a finite number.") from None
    if parsed != parsed or parsed in (float("inf"), float("-inf")):
        raise ValueError(f"{field_name} must be a finite number.")
    return parsed


def normalize_vector(data: dict, name: str) -> dict:
    data = data or {}
    if not isinstance(data, dict):
        raise ValueError(f"{name} must be an object.")
    detected_value = data.get("detected")
    if detected_value is None:
        detected_value = False
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
    x = parse_finite_float(vector.get("x", data.get("x", 0.0)), f"{name}.x")
    y = parse_finite_float(vector.get("y", data.get("y", 0.0)), f"{name}.y")
    return {"detected": detected, "vector": {"x": x, "y": y}}


def normalize_payload(payload: dict) -> dict:
    payload = payload or {}
    if not isinstance(payload, dict):
        raise ValueError("Payload must be a JSON object.")
    normalized = {
        "safe_zone": normalize_vector(payload.get("safe_zone"), "safe_zone"),
        "danger_zone": normalize_vector(payload.get("danger_zone"), "danger_zone"),
        "target": normalize_vector(payload.get("target"), "target"),
        "line": normalize_vector(payload.get("line"), "line"),
    }
    for key in DEFAULT_FLAGS:
        normalized[key] = bool(payload.get(key, False))
    return normalized


def parse_timestamp_ms(value) -> int | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if parsed != parsed or parsed in (float("inf"), float("-inf")):
        return None
    return int(parsed)


def extract_trace(payload: dict) -> dict | None:
    if not payload:
        return None
    trace = payload.get("trace")
    trace_id = None
    sent_at = None
    if isinstance(trace, dict):
        trace_id = trace.get("id") or trace.get("trace_id")
        sent_at = trace.get("sent_at_ms") or trace.get("sent_at")
    else:
        trace_id = payload.get("trace_id")
        sent_at = payload.get("trace_sent_at_ms")

    if not trace_id:
        return None

    trace_payload = {"id": str(trace_id)}
    sent_at_ms = parse_timestamp_ms(sent_at)
    if sent_at_ms is not None:
        trace_payload["sent_at_ms"] = sent_at_ms
    return trace_payload


def update_state(payload: dict) -> dict:
    normalized = normalize_payload(payload)
    with _state_lock:
        _state_payload.update(normalized)
        return copy.deepcopy(_state_payload)


def get_state() -> dict:
    with _state_lock:
        return copy.deepcopy(_state_payload)


def send_to_state_machine(payload: dict) -> dict:
    url = urljoin(STATE_MACHINE_BASE_URL.rstrip("/") + "/", STATE_MACHINE_INPUT_PATH.lstrip("/"))
    response = requests.post(url, json=payload, timeout=1.0)
    response_payload = None
    if response.content:
        try:
            response_payload = response.json()
        except ValueError:
            response_payload = {"text": response.text[:2000]}
    return {
        "status_code": response.status_code,
        "ok": response.ok,
        "response": response_payload,
    }


@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.get("/health")
def health():
    return jsonify({"status": "ok"})


@app.get("/api/config")
def config():
    return jsonify(
        {
            "state_machine_url": STATE_MACHINE_BASE_URL,
            "state_machine_path": STATE_MACHINE_INPUT_PATH,
        }
    )


@app.get("/api/state")
def api_state():
    return jsonify({"state": get_state()})


@app.post("/api/state")
def api_update_state():
    if not request.is_json:
        return jsonify({"message": "Expected JSON payload."}), 400
    payload = request.get_json(silent=True) or {}
    try:
        state = update_state(payload)
    except ValueError as exc:
        return jsonify({"message": str(exc)}), 400
    LOGGER.log("mock_state_updated", {"state": state})
    return jsonify({"state": state})


@app.post("/api/send")
def api_send():
    if request.is_json:
        payload = request.get_json(silent=True) or {}
        try:
            state = update_state(payload)
        except ValueError as exc:
            return jsonify({"message": str(exc)}), 400
        trace = extract_trace(payload)
    else:
        state = get_state()
        trace = None

    try:
        outbound = dict(state)
        if trace:
            outbound["trace"] = trace
        start = time.perf_counter()
        result = send_to_state_machine(outbound)
        elapsed_ms = round((time.perf_counter() - start) * 1000, 2)
        log_payload = {"result": result, "request_ms": elapsed_ms}
        if trace:
            log_payload["trace_id"] = trace.get("id")
            if trace.get("sent_at_ms") is not None:
                log_payload["trace_sent_at_ms"] = trace.get("sent_at_ms")
        LOGGER.log("mock_state_sent", log_payload)
        if trace:
            LOGGER.log(
                "diagnostic_trace_sent",
                {
                    "trace_id": trace.get("id"),
                    "trace_sent_at_ms": trace.get("sent_at_ms"),
                    "request_ms": elapsed_ms,
                    "state_machine_ok": result.get("ok"),
                    "status_code": result.get("status_code"),
                },
            )
        return jsonify({"state": state, "result": result})
    except requests.RequestException as exc:
        LOGGER.log("mock_state_error", {"error": str(exc)})
        return jsonify({"message": "Failed to send to state machine.", "error": str(exc)}), 502


if __name__ == "__main__":
    port = int(os.getenv("PORT", "8100"))
    LOGGER.log("mock_cv_started", {"port": port})
    app.run(host="0.0.0.0", port=port)
