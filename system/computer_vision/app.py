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


def parse_float(value, fallback: float = 0.0) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return fallback
    if parsed != parsed or parsed in (float("inf"), float("-inf")):
        return fallback
    return parsed


def normalize_vector(data: dict) -> dict:
    data = data or {}
    detected_value = data.get("detected")
    if detected_value is None:
        detected_value = False
        for key, value in data.items():
            if "detected" in str(key).lower():
                detected_value = value
                break
    detected = bool(detected_value)
    vector = data.get("vector") if isinstance(data.get("vector"), dict) else {}
    x = parse_float(vector.get("x", data.get("x", 0.0)), 0.0)
    y = parse_float(vector.get("y", data.get("y", 0.0)), 0.0)
    return {"detected": detected, "vector": {"x": x, "y": y}}


def normalize_payload(payload: dict) -> dict:
    payload = payload or {}
    normalized = {
        "safe_zone": normalize_vector(payload.get("safe_zone")),
        "danger_zone": normalize_vector(payload.get("danger_zone")),
        "target": normalize_vector(payload.get("target")),
        "line": normalize_vector(payload.get("line")),
    }
    for key in DEFAULT_FLAGS:
        normalized[key] = bool(payload.get(key, False))
    return normalized


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
    return {
        "status_code": response.status_code,
        "ok": response.ok,
        "response": response.json() if response.content else None,
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
    state = update_state(payload)
    LOGGER.log("mock_state_updated", {"state": state})
    return jsonify({"state": state})


@app.post("/api/send")
def api_send():
    if request.is_json:
        payload = request.get_json(silent=True) or {}
        state = update_state(payload)
    else:
        state = get_state()

    try:
        result = send_to_state_machine(state)
        LOGGER.log("mock_state_sent", {"result": result})
        return jsonify({"state": state, "result": result})
    except requests.RequestException as exc:
        LOGGER.log("mock_state_error", {"error": str(exc)})
        return jsonify({"message": "Failed to send to state machine.", "error": str(exc)}), 502


if __name__ == "__main__":
    port = int(os.getenv("PORT", "8100"))
    LOGGER.log("mock_cv_started", {"port": port})
    app.run(host="0.0.0.0", port=port)
