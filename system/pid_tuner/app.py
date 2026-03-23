#!/usr/bin/env python3
"""
PID tuner web UI — runs on the Pi.

Access from any device on the same network:
    http://<pi-ip>:5050

GET  /         → serves the tuner page
GET  /api/pid  → returns the current pid_config.json as JSON
POST /api/pid  → validates and saves the updated config, returns saved values

The config file is at system/pid_config.json (one level up from this file).
The robot runner (run_local_line_follow.py) reads that file on startup.

Mission telemetry JSONL (lateral_err, avg_rpm) is written by the runner next to
pid_config.json by default; GET /api/mission serves it for charts. Override with
MISSION_TELEMETRY_PATH.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

from flask import Flask, jsonify, request, send_from_directory

# ── Paths ─────────────────────────────────────────────────────────────────────

HERE        = Path(__file__).parent
CONFIG_PATH = Path(os.getenv("PID_CONFIG_PATH", str(HERE.parent / "pid_config.json")))
MISSION_TELEMETRY_PATH = Path(
    os.getenv("MISSION_TELEMETRY_PATH", str(HERE.parent / "mission_telemetry.jsonl"))
)
PORT        = int(os.getenv("PORT", "5050"))

# ── Defaults (used when the config file is missing or a key is absent) ────────

DEFAULTS: dict[str, float | int] = {
    "steer_kp":            0.65,
    "steer_ki":            0.04,
    "steer_kd":            0.10,
    "steer_out_limit":     0.80,

    "motor_kp":            0.003125,
    "motor_ki":            0.0005,
    "motor_kd":            0.0,

    "base_speed":          0.28,
    "min_speed":           0.16,
    "max_speed":           0.45,
    "search_turn":         0.18,
    "search_turn_max":     0.32,

    "forward_ticks":       800,
    "forward_speed":       0.25,

    "turn_speed":          0.30,
    "turn_duration_s":     2.2,

    "claw_open":           0.0,
    "claw_closed":         90.0,
    "pickup_hold_s":       0.8,

    "geom_enable":         True,
    "geom_lateral_norm_m": 0.10,

    "red_loss_debounce_frames": 4,
    "red_error_ema_alpha":      0.35,
}

# Keys that must be integers (not floats)
INT_KEYS = {"forward_ticks", "red_loss_debounce_frames"}

# Keys stored as bool in JSON
BOOL_KEYS = {"geom_enable"}

# ── Flask app ─────────────────────────────────────────────────────────────────

app = Flask(__name__)


def load_config() -> dict:
    try:
        raw = json.loads(CONFIG_PATH.read_text())
        # Fill in any keys that are missing (e.g. after a schema addition)
        return {**DEFAULTS, **{k: v for k, v in raw.items() if k in DEFAULTS}}
    except (FileNotFoundError, json.JSONDecodeError):
        return dict(DEFAULTS)


def save_config(cfg: dict) -> None:
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = CONFIG_PATH.with_suffix(".tmp")
    tmp.write_text(json.dumps(cfg, indent=2) + "\n")
    tmp.replace(CONFIG_PATH)


def validate(data: dict) -> tuple[dict, dict]:
    """Return (clean_values, errors). Only known keys are accepted."""
    out    = {}
    errors = {}
    for key, default in DEFAULTS.items():
        raw = data.get(key)
        if raw is None:
            out[key] = default
            continue
        try:
            if key in BOOL_KEYS:
                if isinstance(raw, bool):
                    val = raw
                else:
                    s = str(raw).lower()
                    if s in ("false", "no", "0", "off"):
                        val = False
                    elif s in ("true", "yes", "on", "1"):
                        val = True
                    else:
                        errors[key] = "Must be a boolean."
                        continue
            elif key in INT_KEYS:
                val = int(raw)
            else:
                val = float(raw)
        except (TypeError, ValueError):
            errors[key] = "Must be a number."
            continue
        if type(val) is bool:
            out[key] = val
            continue
        if not isinstance(val, (int, float)) or val != val:   # NaN check
            errors[key] = "Must be a finite number."
            continue
        out[key] = val
    return out, errors


# ── Routes ────────────────────────────────────────────────────────────────────

@app.get("/")
def index():
    return send_from_directory(str(HERE), "index.html")


@app.get("/api/pid")
def get_pid():
    return jsonify(load_config())


@app.post("/api/pid")
def set_pid():
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({"error": "Expected a JSON object."}), 400

    clean, errors = validate(data)
    if errors:
        return jsonify({"error": "Validation failed.", "fields": errors}), 400

    save_config(clean)
    return jsonify({"ok": True, "saved": clean})


@app.get("/api/mission")
def get_mission():
    """JSONL written by run_local_line_follow.py while mission is running."""
    path = MISSION_TELEMETRY_PATH
    if not path.is_file():
        return jsonify({"points": [], "path": str(path.resolve())})
    points: list[dict] = []
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return jsonify({"points": [], "path": str(path.resolve())})
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            points.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return jsonify({"points": points, "path": str(path.resolve())})


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if not CONFIG_PATH.exists():
        save_config(dict(DEFAULTS))
        print(f"Created default config at {CONFIG_PATH}", flush=True)
    print(f"PID tuner → http://0.0.0.0:{PORT}", flush=True)
    app.run(host="0.0.0.0", port=PORT, debug=False)
