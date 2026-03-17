#!/usr/bin/env python3
import atexit
import json
import math
import os
import signal
import threading
import time
from typing import Any, Optional
from urllib import request as urlrequest

from flask import Flask, jsonify, request
from smbus2 import SMBus, i2c_msg

try:
    import serial
except Exception:
    serial = None


def parse_env_int(name: str, default: str) -> int:
    raw = os.getenv(name, default).strip()
    return int(raw, 0)


def parse_env_bool(name: str, default: str = "0") -> bool:
    raw = (os.getenv(name, default) or "").strip().lower()
    return raw in {"1", "true", "yes", "on"}


def parse_hardware_mode() -> str:
    raw = (os.getenv("HARDWARE_MODE") or "mock").strip().lower()
    if raw in {"mock", "sim", "simulation"}:
        return "mock"
    if raw in {"serial", "uart"}:
        return "serial"
    return "i2c"


HOST = (os.getenv("HOST") or "0.0.0.0").strip()
PORT = parse_env_int("PORT", "5001")
I2C_BUS_NUM = parse_env_int("I2C_BUS", "1")
I2C_ADDR = parse_env_int("I2C_ADDR", "0x08")
SERIAL_PORT = (os.getenv("SERIAL_PORT") or "/dev/ttyUSB0").strip()
SERIAL_BAUDRATE = parse_env_int("SERIAL_BAUDRATE", "115200")
SERIAL_TIMEOUT_S = float((os.getenv("SERIAL_TIMEOUT_S") or "1.0").strip())
SERIAL_WRITE_TIMEOUT_S = float((os.getenv("SERIAL_WRITE_TIMEOUT_S") or "1.0").strip())
DEBUG = parse_env_bool("DEBUG", "0")
MODE = parse_hardware_mode()
SERVO_MIN_DEG = parse_env_int("SERVO_MIN_DEG", "-90")
SERVO_MAX_DEG = parse_env_int("SERVO_MAX_DEG", "0")
SERVO_DEFAULT_DEG = parse_env_int("SERVO_DEFAULT_DEG", "0")
ROBOT_MOCK_CONTROL_URL = (
    os.getenv("ROBOT_MOCK_CONTROL_URL") or "http://localhost:8200/api/sim-control"
).strip()

app = Flask(__name__)

bus_lock = threading.Lock()
bus: Optional[SMBus] = None
serial_link = None
last_sent: Optional[dict[str, Any]] = None


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def log_line(event: str, fields: dict[str, Any]) -> None:
    parts = [f"ts={now_iso()}", f"event={event}"]
    for key, value in fields.items():
        parts.append(f"{key}={value}")
    print(" ".join(parts), flush=True)


def close_bus() -> None:
    global bus
    if bus is not None:
        try:
            bus.close()
        except Exception:
            pass
        bus = None


def close_serial_link() -> None:
    global serial_link
    if serial_link is not None:
        try:
            serial_link.close()
        except Exception:
            pass
        serial_link = None


def handle_signal(signum, _frame) -> None:
    log_line("shutdown", {"signal": signum})
    close_bus()
    close_serial_link()
    raise SystemExit(0)


def parse_numeric(payload: dict[str, Any], key: str) -> tuple[Optional[float], Optional[str]]:
    value = payload.get(key, None)
    if value is None:
        return None, f"Missing '{key}'."
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None, f"'{key}' must be numeric."
    if not math.isfinite(float(value)):
        return None, f"'{key}' must be finite."
    return float(value), None


def parse_optional_speed(payload: dict[str, Any]) -> tuple[Optional[float], Optional[str]]:
    if "speed" not in payload or payload.get("speed") is None:
        return None, None
    raw_value = payload.get("speed")
    if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
        return None, "'speed' must be numeric."
    speed = float(raw_value)
    if not math.isfinite(speed):
        return None, "'speed' must be finite."
    if not (0.0 <= speed <= 5.0):
        return None, "'speed' must be between 0 and 5."
    return speed, None


def parse_optional_servo(payload: dict[str, Any]) -> tuple[Optional[int], Optional[str]]:
    if "servo" not in payload or payload.get("servo") is None:
        return None, None
    raw_value = payload.get("servo")
    if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
        return None, "'servo' must be numeric."
    servo_float = float(raw_value)
    if not math.isfinite(servo_float):
        return None, "'servo' must be finite."
    if not servo_float.is_integer():
        return None, "'servo' must be an integer angle in degrees."
    servo = int(servo_float)
    if not (SERVO_MIN_DEG <= servo <= SERVO_MAX_DEG):
        return None, f"'servo' must be between {SERVO_MIN_DEG} and {SERVO_MAX_DEG}."
    return servo, None


def normalized_to_int8(value: float, name: str) -> tuple[Optional[int], Optional[str]]:
    if not (-1.0 <= value <= 1.0):
        return None, f"'{name}' must be between -1.0 and 1.0 for format='normalized'."
    quantized = int(round(value * 127.0))
    quantized = max(-128, min(127, quantized))
    return quantized, None


def int8_to_int8(value: float, name: str) -> tuple[Optional[int], Optional[str]]:
    if not value.is_integer():
        return None, f"'{name}' must be an integer for format='int8'."
    as_int = int(value)
    if not (-128 <= as_int <= 127):
        return None, f"'{name}' must be between -128 and 127 for format='int8'."
    return as_int, None


def int8_to_twos_complement_u8(value: int) -> int:
    # Keep signed int8 semantics on-wire; neutral (0) stays 0.
    return value & 0xFF


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def encode_payload(payload: dict[str, Any]) -> tuple[Optional[dict[str, Any]], Optional[dict[str, Any]]]:
    fmt_raw = payload.get("format", "normalized")
    fmt = str(fmt_raw).strip().lower()
    if fmt not in {"normalized", "int8"}:
        return None, {"message": "Invalid format. Use 'normalized' or 'int8'."}

    x_val, x_err = parse_numeric(payload, "x")
    y_val, y_err = parse_numeric(payload, "y")
    speed_val, speed_err = parse_optional_speed(payload)
    servo_val, servo_err = parse_optional_servo(payload)

    errors: dict[str, str] = {}
    if x_err:
        errors["x"] = x_err
    if y_err:
        errors["y"] = y_err
    if speed_err:
        errors["speed"] = speed_err
    if servo_err:
        errors["servo"] = servo_err
    if errors:
        return None, {"message": "Invalid vector payload.", "errors": errors}

    if fmt == "normalized":
        x_i8, x_i8_err = normalized_to_int8(x_val, "x")
        y_i8, y_i8_err = normalized_to_int8(y_val, "y")
    else:
        x_i8, x_i8_err = int8_to_int8(x_val, "x")
        y_i8, y_i8_err = int8_to_int8(y_val, "y")

    errors = {}
    if x_i8_err:
        errors["x"] = x_i8_err
    if y_i8_err:
        errors["y"] = y_i8_err
    if errors:
        return None, {"message": "Invalid vector payload.", "errors": errors}

    x_norm = clamp(x_i8 / 127.0, -1.0, 1.0)
    y_norm = clamp(y_i8 / 127.0, -1.0, 1.0)
    left_norm = clamp(y_norm + x_norm, -1.0, 1.0)
    right_norm = clamp(y_norm - x_norm, -1.0, 1.0)
    left_i8 = int(round(left_norm * 127.0))
    right_i8 = int(round(right_norm * 127.0))
    if servo_val is None:
        if last_sent is not None and isinstance(last_sent.get("servo_deg"), int):
            servo_deg = int(last_sent["servo_deg"])
        else:
            servo_deg = SERVO_DEFAULT_DEG
    else:
        servo_deg = servo_val
    servo_deg = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, servo_deg))

    encoded = {
        "format": fmt,
        "x_input": x_val,
        "y_input": y_val,
        "speed": speed_val,
        "servo_deg": servo_deg,
        "x_int8": x_i8,
        "y_int8": y_i8,
        "left_int8": left_i8,
        "right_int8": right_i8,
        "bytes": [int8_to_twos_complement_u8(x_i8), int8_to_twos_complement_u8(y_i8), servo_deg],
    }
    return encoded, None


def process_vector_payload(payload: dict[str, Any]) -> tuple[dict[str, Any], int]:
    global last_sent

    encoded, error = encode_payload(payload)
    if error:
        return {"ok": False, **error, "payload": payload}, 400
    speed_value = encoded["speed"] if encoded["speed"] is not None else 0.0
    sim_payload = {
        "x": encoded["x_input"],
        "y": encoded["y_input"],
        "speed": speed_value,
        "servo": encoded["servo_deg"],
    }
    sim_result = post_to_simulator(sim_payload)

    if MODE == "i2c":
        try:
            with bus_lock:
                if bus is None:
                    raise OSError("I2C bus is not available.")
                msg = i2c_msg.write(I2C_ADDR, encoded["bytes"])
                bus.i2c_rdwr(msg)
        except Exception as exc:
            log_line(
                "vector_send_fail",
                {
                    "client": request.remote_addr,
                    "mode": MODE,
                    "x": encoded["x_input"],
                    "y": encoded["y_input"],
                    "speed": speed_value,
                    "x_int8": encoded["x_int8"],
                    "y_int8": encoded["y_int8"],
                    "servo_deg": encoded["servo_deg"],
                    "left_int8": encoded["left_int8"],
                    "right_int8": encoded["right_int8"],
                    "bytes": encoded["bytes"],
                    "error": repr(exc),
                },
            )
            return (
                {
                    "ok": False,
                    "message": "I2C write failed.",
                    "error": str(exc),
                    "i2c": {
                        "bus": I2C_BUS_NUM,
                        "addr": f"0x{I2C_ADDR:02X}",
                        "active": False,
                    },
                },
                503,
            )

    if MODE == "serial":
        try:
            with bus_lock:
                if serial_link is None:
                    raise OSError("Serial port is not available.")
                # Keep byte-for-byte parity with I2C payload format.
                serial_link.write(bytes(encoded["bytes"]))
                serial_link.flush()
        except Exception as exc:
            log_line(
                "vector_send_fail",
                {
                    "client": request.remote_addr,
                    "mode": MODE,
                    "x": encoded["x_input"],
                    "y": encoded["y_input"],
                    "speed": speed_value,
                    "x_int8": encoded["x_int8"],
                    "y_int8": encoded["y_int8"],
                    "servo_deg": encoded["servo_deg"],
                    "left_int8": encoded["left_int8"],
                    "right_int8": encoded["right_int8"],
                    "bytes": encoded["bytes"],
                    "error": repr(exc),
                },
            )
            return (
                {
                    "ok": False,
                    "message": "Serial write failed.",
                    "error": str(exc),
                    "serial": {
                        "port": SERIAL_PORT,
                        "baudrate": SERIAL_BAUDRATE,
                        "active": False,
                    },
                },
                503,
            )

    ts = now_iso()
    last_sent = {
        "updated_at": ts,
        "ts": ts,
        "format": encoded["format"],
        "x_input": encoded["x_input"],
        "y_input": encoded["y_input"],
        "speed": speed_value,
        "servo_deg": encoded["servo_deg"],
        "x_int8": encoded["x_int8"],
        "y_int8": encoded["y_int8"],
        "left_int8": encoded["left_int8"],
        "right_int8": encoded["right_int8"],
        "bytes": encoded["bytes"],
    }

    log_line(
        "vector_send_ok",
        {
            "client": request.remote_addr,
            "mode": MODE,
            "x": encoded["x_input"],
            "y": encoded["y_input"],
            "speed": speed_value,
            "x_int8": encoded["x_int8"],
            "y_int8": encoded["y_int8"],
            "servo_deg": encoded["servo_deg"],
            "left_int8": encoded["left_int8"],
            "right_int8": encoded["right_int8"],
            "bytes": encoded["bytes"],
            "sim_forwarded": sim_result.get("ok", False),
        },
    )

    return (
        {
            "ok": True,
            "sent": {
                "x_int8": encoded["x_int8"],
                "y_int8": encoded["y_int8"],
                "left_int8": encoded["left_int8"],
                "right_int8": encoded["right_int8"],
                "bytes": encoded["bytes"],
            },
            "command": {
                "x": encoded["x_input"],
                "y": encoded["y_input"],
                "speed": speed_value,
                "servo": encoded["servo_deg"],
                "left": encoded["left_int8"] / 127.0,
                "right": encoded["right_int8"] / 127.0,
                "updated_at": ts,
                "format": encoded["format"],
                "bytes": encoded["bytes"],
            },
            "i2c": {
                "bus": I2C_BUS_NUM,
                "addr": f"0x{I2C_ADDR:02X}",
                "active": MODE == "i2c",
            },
            "serial": {
                "port": SERIAL_PORT,
                "baudrate": SERIAL_BAUDRATE,
                "active": MODE == "serial",
            },
            "mode": MODE,
            "simulator": sim_result,
            "ts": ts,
        },
        200,
    )


def post_to_simulator(payload: dict[str, Any]) -> dict[str, Any]:
    body = json.dumps(payload).encode("utf-8")
    req = urlrequest.Request(
        ROBOT_MOCK_CONTROL_URL,
        data=body,
        headers={"Content-Type": "application/json", "Accept": "application/json"},
        method="POST",
    )
    try:
        with urlrequest.urlopen(req, timeout=0.25) as response:
            return {"ok": 200 <= response.status < 300, "status_code": int(response.status)}
    except Exception as exc:
        log_line(
            "simulator_forward_fail",
            {
                "url": ROBOT_MOCK_CONTROL_URL,
                "error": repr(exc),
            },
        )
        return {"ok": False, "error": str(exc)}


@app.get("/health")
def health():
    return jsonify(
        {
            "ok": True,
            "status": "ok",
            "mode": MODE,
            "i2c_active": MODE == "i2c",
            "serial_active": MODE == "serial",
            "spi_active": False,
            "i2c_bus": I2C_BUS_NUM,
            "i2c_addr": f"0x{I2C_ADDR:02X}",
            "serial_port": SERIAL_PORT,
            "serial_baudrate": SERIAL_BAUDRATE,
        }
    )


@app.get("/last")
def get_last():
    return jsonify({"ok": True, "last": last_sent})


@app.get("/control")
def get_control():
    if last_sent is None:
        command = {
            "x": 0.0,
            "y": 0.0,
            "speed": 0.0,
            "servo": SERVO_DEFAULT_DEG,
            "left": 0.0,
            "right": 0.0,
            "updated_at": None,
            "format": "normalized",
            "bytes": [0, 0, SERVO_DEFAULT_DEG],
        }
    else:
        command = {
            "x": last_sent["x_input"],
            "y": last_sent["y_input"],
            "speed": last_sent.get("speed", 0.0),
            "servo": int(last_sent.get("servo_deg", SERVO_DEFAULT_DEG)),
            "left": last_sent.get("left_int8", 0) / 127.0,
            "right": last_sent.get("right_int8", 0) / 127.0,
            "updated_at": last_sent.get("updated_at"),
            "format": last_sent.get("format", "normalized"),
            "bytes": list(last_sent.get("bytes", [0, 0, SERVO_DEFAULT_DEG])),
        }
    return jsonify({"ok": True, "command": command})


@app.post("/vector")
def post_vector():
    if not request.is_json:
        return jsonify({"ok": False, "message": "Expected JSON payload."}), 400

    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"ok": False, "message": "JSON payload must be an object."}), 400

    response_payload, status_code = process_vector_payload(payload)
    return jsonify(response_payload), status_code


@app.post("/control")
def post_control():
    return post_vector()


def main() -> None:
    global bus, serial_link

    if MODE == "i2c":
        bus = SMBus(I2C_BUS_NUM)
    elif MODE == "serial":
        if serial is None:
            raise RuntimeError("pyserial is required for HARDWARE_MODE=serial but is not installed.")
        serial_link = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUDRATE,
            timeout=SERIAL_TIMEOUT_S,
            write_timeout=SERIAL_WRITE_TIMEOUT_S,
        )

    log_line(
        "startup",
        {
            "host": HOST,
            "port": PORT,
            "mode": MODE,
            "i2c_active": MODE == "i2c",
            "serial_active": MODE == "serial",
            "i2c_bus": I2C_BUS_NUM,
            "i2c_addr": f"0x{I2C_ADDR:02X}",
            "serial_port": SERIAL_PORT,
            "serial_baudrate": SERIAL_BAUDRATE,
            "servo_min_deg": SERVO_MIN_DEG,
            "servo_max_deg": SERVO_MAX_DEG,
            "servo_default_deg": SERVO_DEFAULT_DEG,
            "debug": DEBUG,
        },
    )
    app.run(host=HOST, port=PORT, debug=DEBUG)


atexit.register(close_bus)
atexit.register(close_serial_link)
signal.signal(signal.SIGINT, handle_signal)
signal.signal(signal.SIGTERM, handle_signal)


if __name__ == "__main__":
    main()
