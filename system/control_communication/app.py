#!/usr/bin/env python3
import atexit
from datetime import datetime
import json
import math
import os
import signal
import sys
import threading
import time
from typing import Any, Optional
from urllib import request as urlrequest

from flask import Flask, jsonify, request
from prometheus_client import CONTENT_TYPE_LATEST, Counter, Gauge, generate_latest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from robot_control_system.heading_pid import (
        HEADING_KP,
        HEADING_KI,
        HEADING_KD,
    )
except Exception:  # pragma: no cover
    HEADING_KP, HEADING_KI, HEADING_KD = 2.0, 0.1, 0.0

try:
    from robot_control_system.robot import Robot as _Robot
except Exception as _robot_import_err:
    print(f"ts={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())} event=robot_import_fail error={_robot_import_err!r}", flush=True)
    _Robot = None


def parse_env_int(name: str, default: str) -> int:
    raw = os.getenv(name, default).strip()
    return int(raw, 0)


def parse_env_bool(name: str, default: str = "0") -> bool:
    raw = (os.getenv(name, default) or "").strip().lower()
    return raw in {"1", "true", "yes", "on"}


HOST = (os.getenv("HOST") or "0.0.0.0").strip()
PORT = parse_env_int("PORT", "5001")
SERIAL_PORT = (os.getenv("SERIAL_PORT") or "/dev/ttyACM0").strip()
SERIAL_BAUDRATE = parse_env_int("SERIAL_BAUDRATE", "115200")
DEBUG = parse_env_bool("DEBUG", "0")
SERVO_MIN_DEG = parse_env_int("SERVO_MIN_DEG", "-90")
SERVO_MAX_DEG = parse_env_int("SERVO_MAX_DEG", "0")
SERVO_DEFAULT_DEG = parse_env_int("SERVO_DEFAULT_DEG", "0")
SIM_CONTROL_URL = (
    os.getenv("SIM_CONTROL_URL")
    or ""
).strip()
PID_AUDIT_LOG_PATH = (
    os.getenv("PID_AUDIT_LOG_PATH")
    or "/var/log/control-communication/pid-changes.txt"
).strip()
LINE_FOLLOW_TUNING_PATH = (
    os.getenv("LINE_FOLLOW_TUNING_PATH")
    or "/var/log/control-communication/line-follow-tuning.txt"
).strip()

app = Flask(__name__)

last_sent: Optional[dict[str, Any]] = None

COMMAND_X = Gauge("robot_command_x", "Latest commanded x axis.")
COMMAND_Y = Gauge("robot_command_y", "Latest commanded y axis.")
COMMAND_LATERAL_ERROR = Gauge("robot_command_lateral_error", "Latest commanded lateral error component.")
COMMAND_Y_ERROR = Gauge("robot_command_y_error", "Latest commanded y error component.")
COMMAND_MAGNITUDE = Gauge("robot_command_magnitude", "Latest commanded vector magnitude from x and y.")
COMMAND_SPEED = Gauge("robot_command_speed", "Latest commanded speed.")
COMMAND_SERVO_DEG = Gauge("robot_command_servo_deg", "Latest commanded servo angle in degrees.")
COMMAND_TARGET_HEADING_RAD = Gauge("robot_command_target_heading_rad", "Latest commanded target heading in radians.")
COMMAND_LEFT = Gauge("robot_command_left", "Latest commanded left motor value.")
COMMAND_RIGHT = Gauge("robot_command_right", "Latest commanded right motor value.")
COMMAND_UPDATED_AT = Gauge("robot_command_updated_at_unixtime", "Unix time of the latest command update.")
COMMANDS_TOTAL = Counter("robot_commands_total", "Total number of accepted command updates.")

FEEDBACK_HEADING_RAD = Gauge("robot_feedback_heading_rad", "Latest measured heading in radians.")
FEEDBACK_LINEAR_SPEED_MPS = Gauge("robot_feedback_linear_speed_mps", "Latest measured forward speed in metres per second.")
FEEDBACK_LEFT_ENCODER_TICKS = Gauge("robot_feedback_left_encoder_ticks", "Latest left encoder tick count.")
FEEDBACK_RIGHT_ENCODER_TICKS = Gauge("robot_feedback_right_encoder_ticks", "Latest right encoder tick count.")
FEEDBACK_UPDATED_AT = Gauge("robot_feedback_updated_at_unixtime", "Unix time of the latest feedback update.")
FEEDBACK_UPDATES_TOTAL = Counter("robot_feedback_updates_total", "Total number of accepted feedback updates.")

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


def _clampf(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


class LineFollowPID:
    def __init__(self, kp: float, ki: float, kd: float, i_max: float, out_max: float, deadband: float):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.i_max = abs(float(i_max))
        self.out_max = abs(float(out_max))
        self.deadband = abs(float(deadband))
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_t: float | None = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_t = None

    def configure(self, *, kp: float, ki: float, kd: float, i_max: float, out_max: float, deadband: float) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.i_max = abs(float(i_max))
        self.out_max = abs(float(out_max))
        self.deadband = abs(float(deadband))
        self.reset()

    def update(self, error: float) -> float:
        e = float(error)
        if abs(e) < self.deadband:
            e = 0.0
        now = time.monotonic()
        dt = (now - self._prev_t) if self._prev_t is not None else 0.0
        if dt > 0.0:
            self._integral = _clampf(self._integral + e * dt, -self.i_max, self.i_max)
            derivative = (e - self._prev_error) / dt
        else:
            derivative = 0.0
        self._prev_error = e
        self._prev_t = now
        raw = self.kp * e + self.ki * self._integral + self.kd * derivative
        return _clampf(raw, -self.out_max, self.out_max)


class FirstOrderLag:
    def __init__(self, tau_s: float):
        self.tau_s = max(0.0, float(tau_s))
        self._value = 0.0
        self._prev_t: float | None = None

    def configure(self, tau_s: float) -> None:
        self.tau_s = max(0.0, float(tau_s))
        self.reset()

    def reset(self) -> None:
        self._value = 0.0
        self._prev_t = None

    def filter(self, raw: float) -> float:
        raw_value = float(raw)
        if self.tau_s <= 1e-9:
            self._value = raw_value
            self._prev_t = time.monotonic()
            return raw_value
        now = time.monotonic()
        dt = (now - self._prev_t) if self._prev_t is not None else 0.0
        self._prev_t = now
        if dt <= 0.0:
            self._value = raw_value
            return self._value
        alpha = dt / (self.tau_s + dt)
        self._value = self._value + alpha * (raw_value - self._value)
        return self._value


line_follow_settings: dict[str, float] = {
    "kp": float(os.getenv("LINE_FOLLOW_KP", "0.2")),
    "ki": float(os.getenv("LINE_FOLLOW_KI", "0.04")),
    "kd": float(os.getenv("LINE_FOLLOW_KD", "1.92")),
    "i_max": abs(float(os.getenv("LINE_FOLLOW_I_MAX", "1.0"))),
    "out_max": abs(float(os.getenv("LINE_FOLLOW_OUT_MAX", "1.0"))),
    "deadband": abs(float(os.getenv("LINE_FOLLOW_DEADBAND", "0.01"))),
    "base_speed": _clampf(float(os.getenv("LINE_FOLLOW_BASE_SPEED", "0.2")), 0.0, 1.0),
    "min_speed": _clampf(float(os.getenv("LINE_FOLLOW_MIN_SPEED", "0.05")), 0.0, 1.0),
    "max_speed": _clampf(float(os.getenv("LINE_FOLLOW_MAX_SPEED", "0.3")), 0.0, 1.0),
    "rotation_scale": _clampf(float(os.getenv("LINE_FOLLOW_ROTATION_SCALE", "1.0")), 0.0, 1.0),
    "line_lag_tau": max(0.0, float(os.getenv("LINE_FOLLOW_LAG_TAU", "0.1"))),
    "line_lag_enabled": 1.0 if str(os.getenv("LINE_FOLLOW_LAG_ENABLED", "1")).strip().lower() in {"1", "true", "yes", "on"} else 0.0,
    # Compatibility keys used by dashboard/state-machine UI (not used in control-comm bypass loop).
    "follow_max_speed": _clampf(float(os.getenv("LINE_FOLLOW_FOLLOW_MAX_SPEED", "0.3")), 0.0, 1.0),
    "turn_slowdown": abs(float(os.getenv("LINE_FOLLOW_TURN_SLOWDOWN", "0.0"))),
    "error_slowdown": abs(float(os.getenv("LINE_FOLLOW_ERROR_SLOWDOWN", "0.0"))),
}
if line_follow_settings["min_speed"] > line_follow_settings["max_speed"]:
    line_follow_settings["min_speed"], line_follow_settings["max_speed"] = (
        line_follow_settings["max_speed"],
        line_follow_settings["min_speed"],
    )
line_follow_pid = LineFollowPID(
    kp=line_follow_settings["kp"],
    ki=line_follow_settings["ki"],
    kd=line_follow_settings["kd"],
    i_max=line_follow_settings["i_max"],
    out_max=line_follow_settings["out_max"],
    deadband=line_follow_settings["deadband"],
)
line_follow_lag = FirstOrderLag(line_follow_settings["line_lag_tau"])

if _Robot is not None:
    try:
        print(f"ts={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())} event=robot_init port={SERIAL_PORT} baudrate={SERIAL_BAUDRATE}", flush=True)
        robot = _Robot(SERIAL_PORT, SERIAL_BAUDRATE)
        robot.start()
        print(f"ts={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())} event=robot_start port={SERIAL_PORT} baudrate={SERIAL_BAUDRATE}", flush=True)
    except Exception as _robot_start_err:
        print(f"ts={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())} event=robot_start_fail error={_robot_start_err!r}", flush=True)
        robot = None
else:
    robot = None


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def log_line(event: str, fields: dict[str, Any]) -> None:
    parts = [f"ts={now_iso()}", f"event={event}"]
    for key, value in fields.items():
        parts.append(f"{key}={value}")
    print(" ".join(parts), flush=True)


def publish_command_metrics(
    *,
    x: float,
    y: float,
    speed: float,
    servo_deg: int,
    left: float,
    right: float,
    target_heading_rad: float | None = None,
) -> None:
    COMMAND_X.set(x)
    COMMAND_Y.set(y)
    COMMAND_LATERAL_ERROR.set(x)
    COMMAND_Y_ERROR.set(y)
    COMMAND_MAGNITUDE.set(math.hypot(x, y))
    COMMAND_SPEED.set(speed)
    COMMAND_SERVO_DEG.set(servo_deg)
    COMMAND_LEFT.set(left)
    COMMAND_RIGHT.set(right)
    if target_heading_rad is not None:
        COMMAND_TARGET_HEADING_RAD.set(target_heading_rad)
    COMMAND_UPDATED_AT.set(time.time())
    COMMANDS_TOTAL.inc()


def publish_feedback_metrics() -> None:
    if robot is None:
        return
    try:
        snapshot = robot.get_feedback_snapshot()
    except Exception as exc:
        log_line("metrics_feedback_snapshot_fail", {"error": repr(exc)})
        return
    FEEDBACK_HEADING_RAD.set(float(snapshot.get("heading_rad", 0.0)))
    FEEDBACK_LINEAR_SPEED_MPS.set(float(snapshot.get("linear_speed_mps", 0.0)))
    FEEDBACK_LEFT_ENCODER_TICKS.set(float(snapshot.get("enc_left_ticks", 0)))
    FEEDBACK_RIGHT_ENCODER_TICKS.set(float(snapshot.get("enc_right_ticks", 0)))
    FEEDBACK_UPDATED_AT.set(time.time())
    FEEDBACK_UPDATES_TOTAL.inc()


def _feedback_metrics_loop() -> None:
    while True:
        publish_feedback_metrics()
        time.sleep(0.05)


def _set_current_state_metric(state: str) -> None:
    for known_state in KNOWN_STATES | {state}:
        CURRENT_STATE.labels(state=known_state).set(1 if known_state == state else 0)


def _append_pid_audit(kind: str, value: float, *, source_ip: str | None) -> None:
    path = (PID_AUDIT_LOG_PATH or "").strip()
    if not path:
        return
    line = (
        f"{datetime.utcnow().isoformat(timespec='seconds')}Z "
        f"kind={kind} value={float(value):.6f} source={source_ip or 'unknown'}\n"
    )
    try:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "a", encoding="utf-8") as handle:
            handle.write(line)
    except OSError as exc:
        log_line("pid_audit_write_failed", {"path": path, "error": repr(exc)})


def _persist_tuning_settings() -> None:
    path = (LINE_FOLLOW_TUNING_PATH or "").strip()
    if not path:
        return
    if robot is None:
        heading_kp, heading_ki, heading_kd = float(HEADING_KP), float(HEADING_KI), float(HEADING_KD)
    else:
        heading_kp, heading_ki, heading_kd = robot.get_heading_gains()
    rows = {
        "heading_kp": float(heading_kp),
        "heading_ki": float(heading_ki),
        "heading_kd": float(heading_kd),
        "line_kp": float(line_follow_settings["kp"]),
        "line_ki": float(line_follow_settings["ki"]),
        "line_kd": float(line_follow_settings["kd"]),
        "line_i_max": float(line_follow_settings["i_max"]),
        "line_out_max": float(line_follow_settings["out_max"]),
        "line_deadband": float(line_follow_settings["deadband"]),
        "line_base_speed": float(line_follow_settings["base_speed"]),
        "line_min_speed": float(line_follow_settings["min_speed"]),
        "line_max_speed": float(line_follow_settings["max_speed"]),
        "line_rotation_scale": float(line_follow_settings["rotation_scale"]),
        "line_lag_tau": float(line_follow_settings["line_lag_tau"]),
        "line_lag_enabled": float(line_follow_settings["line_lag_enabled"]),
        "line_follow_max_speed": float(line_follow_settings["follow_max_speed"]),
        "line_turn_slowdown": float(line_follow_settings["turn_slowdown"]),
        "line_error_slowdown": float(line_follow_settings["error_slowdown"]),
        "updated_at": now_iso(),
    }
    try:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            for key, value in rows.items():
                handle.write(f"{key}={value}\n")
    except OSError as exc:
        log_line("line_follow_tuning_write_failed", {"path": path, "error": repr(exc)})


def _load_tuning_settings() -> None:
    path = (LINE_FOLLOW_TUNING_PATH or "").strip()
    if not path:
        return
    try:
        with open(path, "r", encoding="utf-8") as handle:
            lines = handle.readlines()
    except FileNotFoundError:
        return
    except OSError as exc:
        log_line("line_follow_tuning_read_failed", {"path": path, "error": repr(exc)})
        return

    parsed: dict[str, float] = {}
    for raw in lines:
        row = raw.strip()
        if not row or "=" not in row:
            continue
        key, value = row.split("=", 1)
        try:
            parsed[key.strip()] = float(value.strip())
        except ValueError:
            continue
    if not parsed:
        return

    line_follow_settings["kp"] = float(parsed.get("line_kp", line_follow_settings["kp"]))
    line_follow_settings["ki"] = float(parsed.get("line_ki", line_follow_settings["ki"]))
    line_follow_settings["kd"] = float(parsed.get("line_kd", line_follow_settings["kd"]))
    line_follow_settings["i_max"] = abs(float(parsed.get("line_i_max", line_follow_settings["i_max"])))
    line_follow_settings["out_max"] = abs(float(parsed.get("line_out_max", line_follow_settings["out_max"])))
    line_follow_settings["deadband"] = abs(float(parsed.get("line_deadband", line_follow_settings["deadband"])))
    line_follow_settings["base_speed"] = _clampf(parsed.get("line_base_speed", line_follow_settings["base_speed"]), 0.0, 1.0)
    line_follow_settings["min_speed"] = _clampf(parsed.get("line_min_speed", line_follow_settings["min_speed"]), 0.0, 1.0)
    line_follow_settings["max_speed"] = _clampf(parsed.get("line_max_speed", line_follow_settings["max_speed"]), 0.0, 1.0)
    line_follow_settings["rotation_scale"] = _clampf(parsed.get("line_rotation_scale", line_follow_settings["rotation_scale"]), 0.0, 1.0)
    line_follow_settings["line_lag_tau"] = max(0.0, float(parsed.get("line_lag_tau", line_follow_settings["line_lag_tau"])))
    line_follow_settings["line_lag_enabled"] = 1.0 if float(parsed.get("line_lag_enabled", line_follow_settings["line_lag_enabled"])) >= 0.5 else 0.0
    line_follow_settings["follow_max_speed"] = _clampf(parsed.get("line_follow_max_speed", line_follow_settings["follow_max_speed"]), 0.0, 1.0)
    line_follow_settings["turn_slowdown"] = abs(float(parsed.get("line_turn_slowdown", line_follow_settings["turn_slowdown"])))
    line_follow_settings["error_slowdown"] = abs(float(parsed.get("line_error_slowdown", line_follow_settings["error_slowdown"])))
    if line_follow_settings["min_speed"] > line_follow_settings["max_speed"]:
        line_follow_settings["min_speed"], line_follow_settings["max_speed"] = (
            line_follow_settings["max_speed"],
            line_follow_settings["min_speed"],
        )

    line_follow_pid.configure(
        kp=line_follow_settings["kp"],
        ki=line_follow_settings["ki"],
        kd=line_follow_settings["kd"],
        i_max=line_follow_settings["i_max"],
        out_max=line_follow_settings["out_max"],
        deadband=line_follow_settings["deadband"],
    )
    line_follow_lag.configure(line_follow_settings["line_lag_tau"])

    if robot is not None:
        hk = parsed.get("heading_kp")
        hi = parsed.get("heading_ki")
        hd = parsed.get("heading_kd")
        if hk is not None and hi is not None and hd is not None:
            robot.set_gains(float(hk), float(hi), float(hd))


def close_robot() -> None:
    global robot
    if robot is not None:
        try:
            robot.stop()
        except Exception:
            pass
        robot = None


def handle_signal(signum, _frame) -> None:
    log_line("shutdown", {"signal": signum})
    close_robot()
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
    if not (0.0 <= speed <= 1.0):
        return None, "'speed' must be between 0 and 1."
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
    control_mode = str(payload.get("control_mode", "manual")).strip().lower()
    speed_value = encoded["speed"] if encoded["speed"] is not None else line_follow_settings["base_speed"]
    speed_value = _clampf(speed_value, line_follow_settings["min_speed"], line_follow_settings["max_speed"])
    line_error_x = float(payload.get("line_error_x", encoded["x_input"]))
    line_pid_turn = 0.0
    cmd_x = float(encoded["x_input"])
    cmd_y = float(encoded["y_input"])
    rotation_scale = 0.5
    if control_mode == "line_follow":
        line_pid_turn = line_follow_pid.update(line_error_x)
        if line_follow_settings["line_lag_enabled"] >= 0.5:
            line_pid_turn = line_follow_lag.filter(line_pid_turn)
        else:
            line_follow_lag.reset()
        cmd_x = _clampf(line_pid_turn, -1.0, 1.0)
        cmd_y = 1.0
        rotation_scale = float(line_follow_settings["rotation_scale"])
    sim_payload = {
        "x": cmd_x,
        "y": cmd_y,
        "speed": speed_value,
        "servo": encoded["servo_deg"],
    }
    sim_result = post_to_simulator(sim_payload)

    if robot is not None:
        robot.set_rotation_scale(rotation_scale)
        robot.set_direction(cmd_x, cmd_y)
        robot.set_speed(clamp(speed_value, -1.0, 1.0))
        log_line(
            "robot_command",
            {
                "mode": control_mode,
                "x": cmd_x,
                "y": cmd_y,
                "speed": speed_value,
                "line_error_x": line_error_x,
                "line_pid_turn": line_pid_turn,
                "rotation_scale": rotation_scale,
            },
        )
        target_heading_rad = robot.get_feedback_snapshot().get("target_heading_rad")
    else:
        target_heading_rad = None

    applied_left = clamp(cmd_y + cmd_x, -1.0, 1.0)
    applied_right = clamp(cmd_y - cmd_x, -1.0, 1.0)

    publish_command_metrics(
        x=cmd_x,
        y=cmd_y,
        speed=speed_value,
        servo_deg=encoded["servo_deg"],
        left=applied_left,
        right=applied_right,
        target_heading_rad=target_heading_rad if isinstance(target_heading_rad, (int, float)) else None,
    )

    ts = now_iso()
    last_sent = {
        "updated_at": ts,
        "ts": ts,
        "format": encoded["format"],
        "x_input": encoded["x_input"],
        "y_input": encoded["y_input"],
        "x_applied": cmd_x,
        "y_applied": cmd_y,
        "speed": speed_value,
        "control_mode": control_mode,
        "line_error_x": line_error_x,
        "line_pid_turn": line_pid_turn,
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
            "x": encoded["x_input"],
            "y": encoded["y_input"],
            "x_applied": cmd_x,
            "y_applied": cmd_y,
            "speed": speed_value,
            "mode": control_mode,
            "line_error_x": line_error_x,
            "line_pid_turn": line_pid_turn,
            "x_int8": encoded["x_int8"],
            "y_int8": encoded["y_int8"],
            "servo_deg": encoded["servo_deg"],
            "left_int8": encoded["left_int8"],
            "right_int8": encoded["right_int8"],
            "sim_forwarded": sim_result.get("ok", False),
        },
    )

    return (
        {
            "ok": True,
            "command": {
                "x": encoded["x_input"],
                "y": encoded["y_input"],
                "x_applied": cmd_x,
                "y_applied": cmd_y,
                "speed": speed_value,
                "mode": control_mode,
                "line_error_x": line_error_x,
                "line_pid_turn": line_pid_turn,
                "servo": encoded["servo_deg"],
                "left": encoded["left_int8"] / 127.0,
                "right": encoded["right_int8"] / 127.0,
                "updated_at": ts,
                "format": encoded["format"],
            },
            "serial": {
                "port": SERIAL_PORT,
                "baudrate": SERIAL_BAUDRATE,
                "active": robot is not None,
            },
            "simulator": sim_result,
            "ts": ts,
        },
        200,
    )


def post_to_simulator(payload: dict[str, Any]) -> dict[str, Any]:
    if not SIM_CONTROL_URL:
        return {"ok": False, "disabled": True, "error": "simulator forwarding disabled"}
    body = json.dumps(payload).encode("utf-8")
    req = urlrequest.Request(
        SIM_CONTROL_URL,
        data=body,
        headers={"Content-Type": "application/json", "Accept": "application/json"},
        method="POST",
    )
    try:
        with urlrequest.urlopen(req, timeout=0.25) as response:
            return {"ok": 200 <= response.status < 300, "status_code": int(response.status)}
    except Exception as exc:
        log_line("simulator_forward_fail", {"url": SIM_CONTROL_URL, "error": repr(exc)})
        return {"ok": False, "error": str(exc)}


def _heading_pid_triplet() -> tuple[float, float, float]:
    if robot is None:
        return (
            float(HEADING_KP),
            float(HEADING_KI),
            float(HEADING_KD),
        )
    return robot.get_heading_gains()


@app.get("/pid/proportional")
def get_pid_p():
    kp, _, _ = _heading_pid_triplet()
    return jsonify(kp)


@app.post("/pid/proportional")
def post_pid_p():
    if robot is None:
        return jsonify({"ok": False, "message": "Robot is not active."}), 503
    payload = request.get_json(silent=True) or {}
    val, err = parse_numeric(payload, "value")
    if err:
        return jsonify({"ok": False, "message": err}), 400
    _, ki, kd = robot.get_heading_gains()
    robot.set_gains(float(val), ki, kd)
    _append_pid_audit("heading_p", float(val), source_ip=request.remote_addr)
    _persist_tuning_settings()
    return jsonify(float(val))


@app.get("/pid/integral")
def get_pid_i():
    _, ki, _ = _heading_pid_triplet()
    return jsonify(ki)


@app.post("/pid/integral")
def post_pid_i():
    if robot is None:
        return jsonify({"ok": False, "message": "Robot is not active."}), 503
    payload = request.get_json(silent=True) or {}
    val, err = parse_numeric(payload, "value")
    if err:
        return jsonify({"ok": False, "message": err}), 400
    kp, _, kd = robot.get_heading_gains()
    robot.set_gains(kp, float(val), kd)
    _append_pid_audit("heading_i", float(val), source_ip=request.remote_addr)
    _persist_tuning_settings()
    return jsonify(float(val))


@app.get("/pid/derivative")
def get_pid_d():
    _, _, kd = _heading_pid_triplet()
    return jsonify(kd)


@app.post("/pid/derivative")
def post_pid_d():
    if robot is None:
        return jsonify({"ok": False, "message": "Robot is not active."}), 503
    payload = request.get_json(silent=True) or {}
    val, err = parse_numeric(payload, "value")
    if err:
        return jsonify({"ok": False, "message": err}), 400
    kp, ki, _ = robot.get_heading_gains()
    robot.set_gains(kp, ki, float(val))
    _append_pid_audit("heading_d", float(val), source_ip=request.remote_addr)
    _persist_tuning_settings()
    return jsonify(float(val))


@app.get("/line-follow-pid")
def get_line_follow_pid():
    return jsonify(
        {
            "ok": True,
            "values": dict(line_follow_settings),
            "path": LINE_FOLLOW_TUNING_PATH,
        }
    )


@app.post("/line-follow-pid")
def post_line_follow_pid():
    payload = request.get_json(silent=True) or {}
    if not isinstance(payload, dict):
        return jsonify({"ok": False, "message": "JSON payload must be an object."}), 400

    allowed = set(line_follow_settings.keys())
    updates: dict[str, float] = {}
    errors: dict[str, str] = {}
    for key, value in payload.items():
        if key not in allowed:
            errors[key] = "Unknown setting."
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            errors[key] = "Value must be numeric."
            continue
        numeric = float(value)
        if not math.isfinite(numeric):
            errors[key] = "Value must be finite."
            continue
        updates[key] = numeric

    if errors:
        return jsonify({"ok": False, "message": "Invalid payload.", "errors": errors}), 400
    if not updates:
        return jsonify({"ok": False, "message": "No settings provided."}), 400

    for key, val in updates.items():
        if key in {"i_max", "out_max", "deadband"}:
            line_follow_settings[key] = abs(float(val))
        elif key in {"base_speed", "min_speed", "max_speed"}:
            line_follow_settings[key] = _clampf(val, 0.0, 1.0)
        elif key == "rotation_scale":
            line_follow_settings[key] = _clampf(val, 0.0, 1.0)
        elif key == "line_lag_tau":
            line_follow_settings[key] = max(0.0, float(val))
        elif key == "line_lag_enabled":
            line_follow_settings[key] = 1.0 if float(val) >= 0.5 else 0.0
        elif key == "follow_max_speed":
            line_follow_settings[key] = _clampf(val, 0.0, 1.0)
        elif key in {"turn_slowdown", "error_slowdown"}:
            line_follow_settings[key] = abs(float(val))
        else:
            line_follow_settings[key] = float(val)

    if line_follow_settings["min_speed"] > line_follow_settings["max_speed"]:
        return jsonify(
            {
                "ok": False,
                "message": "Invalid speed range.",
                "errors": {
                    "min_speed": "min_speed must be <= max_speed",
                    "max_speed": "max_speed must be >= min_speed",
                },
            }
        ), 400

    line_follow_pid.configure(
        kp=line_follow_settings["kp"],
        ki=line_follow_settings["ki"],
        kd=line_follow_settings["kd"],
        i_max=line_follow_settings["i_max"],
        out_max=line_follow_settings["out_max"],
        deadband=line_follow_settings["deadband"],
    )
    line_follow_lag.configure(line_follow_settings["line_lag_tau"])
    _persist_tuning_settings()
    return jsonify({"ok": True, "values": dict(line_follow_settings), "updated": updates})


@app.get("/health")
def health():
    return jsonify(
        {
            "ok": True,
            "status": "ok",
            "serial_port": SERIAL_PORT,
            "serial_baudrate": SERIAL_BAUDRATE,
            "robot_active": robot is not None,
        }
    )


@app.get("/telemetry")
def telemetry():
    heading_current = 0.0
    heading_target = 0.0
    enc_left = 0
    enc_right = 0
    if robot is not None:
        try:
            heading_current, heading_target = robot.get_heading()
            enc_left, enc_right = robot.get_encoders()
        except Exception as exc:
            return jsonify({"ok": False, "message": f"Telemetry unavailable: {exc}"}), 500
    return jsonify(
        {
            "ok": True,
            "robot_active": robot is not None,
            "heading": {
                "current_degrees": heading_current,
                "target_degrees": heading_target,
            },
            "encoders": {
                "left_ticks": enc_left,
                "right_ticks": enc_right,
            },
        }
    )


@app.get("/metrics")
def metrics():
    return app.response_class(generate_latest(), mimetype=CONTENT_TYPE_LATEST)


@app.post("/metrics/state")
def post_metrics_state():
    if not request.is_json:
        return jsonify({"ok": False, "message": "Expected JSON payload."}), 400
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"ok": False, "message": "JSON payload must be an object."}), 400

    state = str(payload.get("state", "")).strip()
    source_state = str(payload.get("source_state", "")).strip()
    transition_label = str(payload.get("transition_label", "")).strip() or "unspecified"
    if not state:
        return jsonify({"ok": False, "message": "Missing 'state'."}), 400

    _set_current_state_metric(state)
    STATE_UPDATED_AT.set(time.time())
    STATE_UPDATES_TOTAL.inc()
    if source_state:
        STATE_TRANSITIONS_TOTAL.labels(
            source_state=source_state,
            target_state=state,
            transition_label=transition_label,
        ).inc()
    return jsonify({"ok": True, "state": state})


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


@app.post("/turn-test")
def post_turn_test():
    if robot is None:
        return jsonify({"ok": False, "message": "Robot is not active."}), 503
    if not request.is_json:
        return jsonify({"ok": False, "message": "Expected JSON payload."}), 400
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"ok": False, "message": "JSON payload must be an object."}), 400

    raw_degrees = payload.get("degrees")
    if isinstance(raw_degrees, bool) or not isinstance(raw_degrees, (int, float)):
        return jsonify({"ok": False, "message": "'degrees' must be numeric."}), 400
    degrees = float(raw_degrees)
    if not math.isfinite(degrees) or abs(degrees) > 720.0:
        return jsonify({"ok": False, "message": "'degrees' must be finite and within [-720, 720]."}), 400

    speed = float(payload.get("speed", 0.35))
    tolerance_deg = float(payload.get("tolerance_deg", 5.0))
    timeout_s = float(payload.get("timeout_s", 10.0))
    settle_s = float(payload.get("settle_s", 0.2))

    try:
        result = robot.turn_by_degrees(
            degrees=degrees,
            speed=speed,
            tolerance_deg=tolerance_deg,
            timeout=timeout_s,
            settle_time_s=settle_s,
        )
        log_line(
            "turn_test",
            {
                "requested_deg": degrees,
                "achieved_deg": round(result.get("achieved_degrees", 0.0), 3),
                "error_deg": round(result.get("error_degrees", 0.0), 3),
                "success": result.get("success", False),
            },
        )
        return jsonify({"ok": True, "result": result})
    except Exception as exc:
        log_line("turn_test_fail", {"error": repr(exc), "requested_deg": degrees})
        return jsonify({"ok": False, "message": f"Turn test failed: {exc}"}), 500


def main() -> None:
    _load_tuning_settings()
    _persist_tuning_settings()
    _set_current_state_metric("remote_control")
    log_line(
        "startup",
        {
            "host": HOST,
            "port": PORT,
            "serial_port": SERIAL_PORT,
            "serial_baudrate": SERIAL_BAUDRATE,
            "robot_active": robot is not None,
            "servo_min_deg": SERVO_MIN_DEG,
            "servo_max_deg": SERVO_MAX_DEG,
            "servo_default_deg": SERVO_DEFAULT_DEG,
            "debug": DEBUG,
        },
    )
    threading.Thread(target=_feedback_metrics_loop, daemon=True).start()
    app.run(host=HOST, port=PORT, debug=DEBUG)


atexit.register(close_robot)
signal.signal(signal.SIGINT, handle_signal)
signal.signal(signal.SIGTERM, handle_signal)


if __name__ == "__main__":
    main()
