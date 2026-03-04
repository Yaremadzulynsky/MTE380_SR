#!/usr/bin/env python3
import atexit
import math
import os
import signal
import threading
import time
from typing import Any, Optional

from flask import Flask, jsonify, request
from smbus2 import SMBus, i2c_msg


def parse_env_int(name: str, default: str) -> int:
    raw = os.getenv(name, default).strip()
    return int(raw, 0)


def parse_env_bool(name: str, default: str = "0") -> bool:
    raw = (os.getenv(name, default) or "").strip().lower()
    return raw in {"1", "true", "yes", "on"}


HOST = (os.getenv("HOST") or "0.0.0.0").strip()
PORT = parse_env_int("PORT", "5001")
I2C_BUS_NUM = parse_env_int("I2C_BUS", "1")
I2C_ADDR = parse_env_int("I2C_ADDR", "0x08")
DEBUG = parse_env_bool("DEBUG", "0")

app = Flask(__name__)

bus_lock = threading.Lock()
bus: Optional[SMBus] = None
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


def handle_signal(signum, _frame) -> None:
    log_line("shutdown", {"signal": signum})
    close_bus()
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


def encode_payload(payload: dict[str, Any]) -> tuple[Optional[dict[str, Any]], Optional[dict[str, Any]]]:
    fmt_raw = payload.get("format", "normalized")
    fmt = str(fmt_raw).strip().lower()
    if fmt not in {"normalized", "int8"}:
        return None, {"message": "Invalid format. Use 'normalized' or 'int8'."}

    x_val, x_err = parse_numeric(payload, "x")
    y_val, y_err = parse_numeric(payload, "y")
    errors = {}
    if x_err:
        errors["x"] = x_err
    if y_err:
        errors["y"] = y_err
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

    encoded = {
        "format": fmt,
        "x_input": x_val,
        "y_input": y_val,
        "x_int8": x_i8,
        "y_int8": y_i8,
        "bytes": [x_i8 & 0xFF, y_i8 & 0xFF],
    }
    return encoded, None


@app.get("/health")
def health():
    return jsonify(
        {
            "ok": True,
            "i2c_bus": I2C_BUS_NUM,
            "i2c_addr": f"0x{I2C_ADDR:02X}",
        }
    )


@app.get("/last")
def get_last():
    return jsonify({"ok": True, "last": last_sent})


@app.post("/vector")
def post_vector():
    global last_sent

    if not request.is_json:
        return jsonify({"ok": False, "message": "Expected JSON payload."}), 400

    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"ok": False, "message": "JSON payload must be an object."}), 400

    encoded, error = encode_payload(payload)
    if error:
        return jsonify({"ok": False, **error, "payload": payload}), 400

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
                "x": encoded["x_input"],
                "y": encoded["y_input"],
                "x_int8": encoded["x_int8"],
                "y_int8": encoded["y_int8"],
                "bytes": encoded["bytes"],
                "error": repr(exc),
            },
        )
        return (
            jsonify(
                {
                    "ok": False,
                    "message": "I2C write failed.",
                    "error": str(exc),
                    "i2c": {"bus": I2C_BUS_NUM, "addr": f"0x{I2C_ADDR:02X}"},
                }
            ),
            503,
        )

    ts = now_iso()
    last_sent = {
        "ts": ts,
        "format": encoded["format"],
        "x_input": encoded["x_input"],
        "y_input": encoded["y_input"],
        "x_int8": encoded["x_int8"],
        "y_int8": encoded["y_int8"],
        "bytes": encoded["bytes"],
    }

    log_line(
        "vector_send_ok",
        {
            "client": request.remote_addr,
            "x": encoded["x_input"],
            "y": encoded["y_input"],
            "x_int8": encoded["x_int8"],
            "y_int8": encoded["y_int8"],
            "bytes": encoded["bytes"],
        },
    )

    return jsonify(
        {
            "ok": True,
            "sent": {
                "x_int8": encoded["x_int8"],
                "y_int8": encoded["y_int8"],
                "bytes": encoded["bytes"],
            },
            "i2c": {"bus": I2C_BUS_NUM, "addr": f"0x{I2C_ADDR:02X}"},
            "ts": ts,
        }
    )


def main() -> None:
    global bus
    bus = SMBus(I2C_BUS_NUM)
    log_line(
        "startup",
        {
            "host": HOST,
            "port": PORT,
            "i2c_bus": I2C_BUS_NUM,
            "i2c_addr": f"0x{I2C_ADDR:02X}",
            "debug": DEBUG,
        },
    )
    app.run(host=HOST, port=PORT, debug=DEBUG)


atexit.register(close_bus)
signal.signal(signal.SIGINT, handle_signal)
signal.signal(signal.SIGTERM, handle_signal)


if __name__ == "__main__":
    main()
