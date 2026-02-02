import json
import os
import sys
from datetime import datetime
from typing import Optional

from flask import Flask, Response, jsonify, request
from prometheus_client import Counter, Gauge, CONTENT_TYPE_LATEST, generate_latest


class PidStore:
    def get(self, key: str) -> float:
        raise NotImplementedError

    def set(self, key: str, value: float) -> float:
        raise NotImplementedError


class MockPidStore(PidStore):
    def __init__(self, initial_values: dict[str, float]):
        self._values = dict(initial_values)

    def get(self, key: str) -> float:
        return self._values[key]

    def set(self, key: str, value: float) -> float:
        self._values[key] = value
        return value


class SpiPidStore(PidStore):
    def __init__(self, spi_client, initial_values: dict[str, float]):
        self._spi = spi_client
        self._values = dict(initial_values)

    def get(self, key: str) -> float:
        return self._values[key]

    def set(self, key: str, value: float) -> float:
        self._values[key] = value
        payload = f"pid:{key}:{value}\n".encode("utf-8")
        self._spi.send(payload)
        return value


class ControlStore:
    def send(self, x: float, y: float, speed: float) -> dict:
        raise NotImplementedError


class MockControlStore(ControlStore):
    def __init__(self):
        self._last = {"x": 0.0, "y": 0.0, "speed": 0.0}

    def send(self, x: float, y: float, speed: float) -> dict:
        self._last = {"x": x, "y": y, "speed": speed}
        return dict(self._last)


class SpiControlStore(ControlStore):
    def __init__(self, spi_client):
        self._spi = spi_client
        self._last = {"x": 0.0, "y": 0.0, "speed": 0.0}

    def send(self, x: float, y: float, speed: float) -> dict:
        self._last = {"x": x, "y": y, "speed": speed}
        payload = f"control:{x}:{y}:{speed}\n".encode("utf-8")
        self._spi.send(payload)
        return dict(self._last)


class SpiClient:
    def __init__(self, bus: int, device: int, speed_hz: int):
        import spidev  # type: ignore

        self._spi = spidev.SpiDev()
        self._spi.open(bus, device)
        self._spi.max_speed_hz = speed_hz

    def send(self, payload: bytes) -> None:
        self._spi.xfer2(list(payload))


def parse_float(value: str) -> Optional[float]:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not (parsed == parsed and abs(parsed) != float("inf")):
        return None
    return parsed


def load_initial_values() -> dict[str, float]:
    return {
        "p": parse_float(os.getenv("PID_P_DEFAULT", "0")) or 0.0,
        "i": parse_float(os.getenv("PID_I_DEFAULT", "0")) or 0.0,
        "d": parse_float(os.getenv("PID_D_DEFAULT", "0")) or 0.0,
    }


def create_spi_client():
    try:
        bus = int(os.getenv("SPI_BUS", "0"))
        device = int(os.getenv("SPI_DEVICE", "0"))
        speed_hz = int(os.getenv("SPI_SPEED_HZ", "500000"))
        return SpiClient(bus, device, speed_hz)
    except Exception as exc:  # pylint: disable=broad-except
        log_event("hardware_init_failed", {"error": str(exc)})
        return None


def create_stores() -> tuple[PidStore, ControlStore, str]:
    mode = os.getenv("HARDWARE_MODE", "mock").lower()
    if mode in {"mock", "sim", "simulation"}:
        return MockPidStore(load_initial_values()), MockControlStore(), "mock"

    spi_client = create_spi_client()
    if spi_client:
        return (
            SpiPidStore(spi_client, load_initial_values()),
            SpiControlStore(spi_client),
            "spi",
        )

    return MockPidStore(load_initial_values()), MockControlStore(), "mock"


def create_insights_logger():
    path = os.getenv("LOG_PATH") or os.getenv("INSIGHTS_IPC_PATH")
    if not path:
        return None
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        return open(path, "a", encoding="utf-8")
    except OSError as exc:
        print(f"Unable to open insights IPC pipe: {exc}", file=sys.stderr)
        return None


def log_event(event: str, payload: dict):
    entry = {
        "ts": datetime.utcnow().isoformat(timespec="seconds") + "Z",
        "event": event,
        "payload": payload,
    }
    line = json.dumps(entry)
    print(line, flush=True)
    if INSIGHTS_LOGGER:
        INSIGHTS_LOGGER.write(line + "\n")
        INSIGHTS_LOGGER.flush()


def log_service_line(message: str):
    log_event("service_log", {"message": message})


INSIGHTS_LOGGER = create_insights_logger()
PID_STORE, CONTROL_STORE, PID_MODE = create_stores()

app = Flask(__name__)

PID_GAIN = Gauge("pid_gain", "Current PID gain value", ["axis"])
PID_SET_TOTAL = Counter("pid_set_total", "PID set requests", ["axis"])
PID_GET_TOTAL = Counter("pid_get_total", "PID get requests", ["axis"])
PID_ERRORS_TOTAL = Counter("pid_errors_total", "PID errors", ["axis", "type"])
PID_MODE_GAUGE = Gauge("pid_bridge_mode", "Bridge mode status", ["mode"])
CONTROL_COMMAND_TOTAL = Counter(
    "control_command_total", "Control commands sent", []
)
CONTROL_ERRORS_TOTAL = Counter(
    "control_command_errors_total", "Control command errors", ["type"]
)
CONTROL_VECTOR = Gauge("control_vector", "Control vector", ["axis"])
CONTROL_SPEED = Gauge("control_speed", "Control speed", [])
SYSTEM_STATE_GAUGE = Gauge("system_state", "State machine state", ["state"])

SYSTEM_STATE = {
    "state": "unknown",
    "updated_at": None,
}


def initialize_metrics():
    for mode in ("mock", "spi"):
        PID_MODE_GAUGE.labels(mode=mode).set(1 if PID_MODE == mode else 0)
    for axis in ("p", "i", "d"):
        PID_GAIN.labels(axis=axis).set(PID_STORE.get(axis))
    CONTROL_VECTOR.labels(axis="x").set(0)
    CONTROL_VECTOR.labels(axis="y").set(0)
    CONTROL_SPEED.set(0)
    SYSTEM_STATE_GAUGE.labels(state="unknown").set(1)


@app.get("/health")
def health():
    return jsonify(
        {
            "status": "ok",
            "mode": PID_MODE,
            "spi_active": PID_MODE == "spi",
        }
    )


@app.get("/metrics")
def metrics():
    return Response(generate_latest(), mimetype=CONTENT_TYPE_LATEST)


def handle_get_pid(key: str):
    value = PID_STORE.get(key)
    PID_GET_TOTAL.labels(axis=key).inc()
    PID_GAIN.labels(axis=key).set(value)
    log_event("pid_get", {"key": key, "value": value})
    return jsonify({"value": value})


def extract_value_from_request():
    if request.is_json:
        payload = request.get_json(silent=True) or {}
        value = payload.get("value")
        if value is None:
            return None, "Missing 'value' in JSON payload."
        return value, None

    raw = request.get_data(as_text=True).strip()
    if not raw:
        return None, "Empty request body."
    return raw, None


def handle_set_pid(key: str):
    raw_value, error = extract_value_from_request()
    if error:
        PID_ERRORS_TOTAL.labels(axis=key, type="payload").inc()
        return jsonify({"message": error}), 400

    value = parse_float(raw_value)
    if value is None:
        PID_ERRORS_TOTAL.labels(axis=key, type="parse").inc()
        return jsonify({"message": "Value must be a finite number."}), 400

    updated = PID_STORE.set(key, value)
    PID_SET_TOTAL.labels(axis=key).inc()
    PID_GAIN.labels(axis=key).set(updated)
    log_event("pid_set", {"key": key, "value": updated})
    return jsonify({"value": updated})


def parse_control_payload():
    if not request.is_json:
        return None, {"payload": "Expected JSON payload."}

    payload = request.get_json(silent=True) or {}
    errors = {}

    x = parse_float(payload.get("x"))
    y = parse_float(payload.get("y"))
    speed = parse_float(payload.get("speed"))

    if x is None:
        errors["x"] = "X must be a finite number."
    if y is None:
        errors["y"] = "Y must be a finite number."
    if speed is None:
        errors["speed"] = "Speed must be a finite number."

    if x is not None and not (-1.0 <= x <= 1.0):
        errors["x"] = "X must be between -1 and 1."
    if y is not None and not (-1.0 <= y <= 1.0):
        errors["y"] = "Y must be between -1 and 1."
    if speed is not None and not (0.0 <= speed <= 1.0):
        errors["speed"] = "Speed must be between 0 and 1."

    if errors:
        return None, errors

    return {"x": x, "y": y, "speed": speed}, None


@app.post("/control")
def send_control():
    payload, errors = parse_control_payload()
    if errors:
        CONTROL_ERRORS_TOTAL.labels(type="validation").inc()
        return jsonify({"message": "Invalid control payload.", "errors": errors}), 400

    command = CONTROL_STORE.send(payload["x"], payload["y"], payload["speed"])
    CONTROL_COMMAND_TOTAL.inc()
    CONTROL_VECTOR.labels(axis="x").set(command["x"])
    CONTROL_VECTOR.labels(axis="y").set(command["y"])
    CONTROL_SPEED.set(command["speed"])
    log_event("control_command", command)
    return jsonify({"command": command})


@app.get("/state")
def get_state():
    return jsonify(SYSTEM_STATE)


@app.post("/state")
def set_state():
    if not request.is_json:
        return jsonify({"message": "Expected JSON payload."}), 400
    payload = request.get_json(silent=True) or {}
    state = payload.get("state")
    if not isinstance(state, str) or not state:
        return jsonify({"message": "Missing 'state' in payload."}), 400

    SYSTEM_STATE["state"] = state
    SYSTEM_STATE["updated_at"] = datetime.utcnow().isoformat(timespec="seconds") + "Z"
    SYSTEM_STATE_GAUGE.clear()
    SYSTEM_STATE_GAUGE.labels(state=state).set(1)
    log_event("state_update", SYSTEM_STATE)
    return jsonify(SYSTEM_STATE)


@app.get("/pid/proportional")
def get_proportional():
    return handle_get_pid("p")


@app.post("/pid/proportional")
def set_proportional():
    return handle_set_pid("p")


@app.get("/pid/integral")
def get_integral():
    return handle_get_pid("i")


@app.post("/pid/integral")
def set_integral():
    return handle_set_pid("i")


@app.get("/pid/derivative")
def get_derivative():
    return handle_get_pid("d")


@app.post("/pid/derivative")
def set_derivative():
    return handle_set_pid("d")


if __name__ == "__main__":
    port = int(os.getenv("PORT", "5000"))
    initialize_metrics()
    log_service_line(f"Control communication running on 0.0.0.0:{port}")
    app.run(host="0.0.0.0", port=port)
