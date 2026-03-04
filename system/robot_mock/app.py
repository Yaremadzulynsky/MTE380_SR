import json
import os
import time
from collections import defaultdict, deque
from threading import Lock
from urllib.parse import urljoin

import requests
from flask import Flask, jsonify, request, send_from_directory


def env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


CONTROL_COMM_BASE_URL = os.getenv("CONTROL_COMM_BASE_URL", "http://control-communication:5000")
CONTROL_COMM_CONTROL_PATH = os.getenv("CONTROL_COMM_CONTROL_PATH", "/control")

STATE_MACHINE_BASE_URL = os.getenv("STATE_MACHINE_BASE_URL", "http://state-machine:8000")
STATE_MACHINE_INPUT_PATH = os.getenv("STATE_MACHINE_INPUT_PATH", "/inputs")
STATE_MACHINE_RESET_PATH = os.getenv("STATE_MACHINE_RESET_PATH", "/reset")
STATE_MACHINE_STATES_PATH = os.getenv("STATE_MACHINE_STATES_PATH", "/states")

DISABLE_EXTERNAL_REQUESTS = env_bool("DISABLE_EXTERNAL_REQUESTS", False)
QUIET_ACCESS_LOG = env_bool("QUIET_ACCESS_LOG", False)
API_TOKEN = os.getenv("ROBOT_MOCK_API_TOKEN") or None

REQUEST_TIMEOUT_S = float(os.getenv("REQUEST_TIMEOUT_S", "0.5"))
SYSTEM_POLL_INTERVAL_S = float(os.getenv("SYSTEM_POLL_INTERVAL_S", "0.1"))


app = Flask(__name__, static_folder="public", static_url_path="")

_cache_lock = Lock()
_system_cache: dict | None = None
_system_cache_updated_at_ms: int | None = None

_rate_lock = Lock()
_rate_buckets: dict[tuple[str, str], deque[float]] = defaultdict(deque)


def _absolute_url(base_url: str, path: str) -> str:
    return urljoin(base_url.rstrip("/") + "/", path.lstrip("/"))


def _fetch_json(url: str) -> dict:
    if DISABLE_EXTERNAL_REQUESTS:
        return {"ok": True, "status_code": 200, "data": None, "disabled": True}
    try:
        response = requests.get(url, timeout=REQUEST_TIMEOUT_S)
        content_type = (response.headers.get("content-type") or "").lower()
        if response.content and "application/json" in content_type:
            data = response.json()
        elif response.content:
            data = {"text": response.text[:2000], "content_type": response.headers.get("content-type")}
        else:
            data = None
        return {"ok": response.ok, "status_code": response.status_code, "data": data, "content_type": response.headers.get("content-type")}
    except requests.RequestException as exc:
        return {"ok": False, "status_code": None, "error": str(exc)}
    except json.JSONDecodeError as exc:
        return {"ok": False, "status_code": None, "error": f"Invalid JSON: {exc}"}


def _post_json(url: str, payload: dict) -> dict:
    if DISABLE_EXTERNAL_REQUESTS:
        return {"ok": True, "status_code": 200, "data": {"disabled": True}}
    try:
        response = requests.post(url, json=payload, timeout=REQUEST_TIMEOUT_S)
        content_type = (response.headers.get("content-type") or "").lower()
        if response.content and "application/json" in content_type:
            data = response.json()
        elif response.content:
            data = {"text": response.text[:2000], "content_type": response.headers.get("content-type")}
        else:
            data = None
        return {"ok": response.ok, "status_code": response.status_code, "data": data, "content_type": response.headers.get("content-type")}
    except requests.RequestException as exc:
        return {"ok": False, "status_code": None, "error": str(exc)}
    except json.JSONDecodeError as exc:
        return {"ok": False, "status_code": None, "error": f"Invalid JSON: {exc}"}


def _client_ip() -> str:
    forwarded = request.headers.get("X-Forwarded-For", "")
    if forwarded:
        return forwarded.split(",")[0].strip()
    return request.remote_addr or "unknown"


def _rate_limit(key: str, max_per_s: int) -> bool:
    now = time.time()
    bucket_key = (_client_ip(), key)
    with _rate_lock:
        bucket = _rate_buckets[bucket_key]
        while bucket and now - bucket[0] > 1.0:
            bucket.popleft()
        if len(bucket) >= max_per_s:
            return False
        bucket.append(now)
        return True


def _token_ok() -> bool:
    if not API_TOKEN:
        return True
    provided = request.headers.get("x-robot-mock-token") or request.args.get("token")
    return bool(provided) and provided == API_TOKEN


def _fetch_system_snapshot() -> dict:
    control_url = _absolute_url(CONTROL_COMM_BASE_URL, CONTROL_COMM_CONTROL_PATH)
    state_url = _absolute_url(STATE_MACHINE_BASE_URL, STATE_MACHINE_STATES_PATH)
    return {
        "ts_ms": int(time.time() * 1000),
        "control": _fetch_json(control_url),
        "state": _fetch_json(state_url),
    }


def _update_cache(snapshot: dict) -> None:
    global _system_cache, _system_cache_updated_at_ms
    with _cache_lock:
        _system_cache = snapshot
        _system_cache_updated_at_ms = snapshot.get("ts_ms")


def _get_cache() -> tuple[dict | None, int | None]:
    with _cache_lock:
        return _system_cache, _system_cache_updated_at_ms


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
            "control_comm": {
                "base_url": CONTROL_COMM_BASE_URL,
                "paths": {
                    "control": CONTROL_COMM_CONTROL_PATH,
                },
            },
            "state_machine": {
                "base_url": STATE_MACHINE_BASE_URL,
                "paths": {
                    "inputs": STATE_MACHINE_INPUT_PATH,
                    "reset": STATE_MACHINE_RESET_PATH,
                    "states": STATE_MACHINE_STATES_PATH,
                },
            },
            "external_requests_disabled": DISABLE_EXTERNAL_REQUESTS,
            "request_timeout_s": REQUEST_TIMEOUT_S,
            "system_poll_interval_s": SYSTEM_POLL_INTERVAL_S,
            "token_required": bool(API_TOKEN),
        }
    )


@app.get("/api/system")
def system_snapshot():
    if not _token_ok():
        return jsonify({"message": "Unauthorized."}), 401

    # Without caching, each browser refresh hammers upstream services. Also helps
    # reduce impact from accidental/public scanning of this port.
    if not _rate_limit("system", max_per_s=20):
        return jsonify({"message": "Too many requests."}), 429

    cached, updated_at = _get_cache()
    force_fresh = request.args.get("fresh") in {"1", "true", "yes"}
    cache_age_ms = None if updated_at is None else int(time.time() * 1000) - int(updated_at)
    stale = cache_age_ms is None or cache_age_ms > int(SYSTEM_POLL_INTERVAL_S * 1000)

    if cached is None or force_fresh or stale:
        snapshot = _fetch_system_snapshot()
        _update_cache(snapshot)
        return jsonify({"cached": False, "updated_at_ms": snapshot.get("ts_ms"), **snapshot})

    return jsonify(
        {
            "cached": True,
            "updated_at_ms": updated_at,
            "cache_age_ms": cache_age_ms,
            **cached,
        }
    )


@app.post("/api/state-machine/inputs")
def send_state_machine_inputs():
    if not _token_ok():
        return jsonify({"message": "Unauthorized."}), 401
    if not request.is_json:
        return jsonify({"message": "Expected JSON payload."}), 400
    payload = request.get_json(silent=True) or {}
    url = _absolute_url(STATE_MACHINE_BASE_URL, STATE_MACHINE_INPUT_PATH)
    result = _post_json(url, payload)
    status = 200 if result.get("ok") else 502
    return jsonify({"result": result}), status


@app.post("/api/state-machine/restart")
def restart_state_machine():
    if not _token_ok():
        return jsonify({"message": "Unauthorized."}), 401
    url = _absolute_url(STATE_MACHINE_BASE_URL, STATE_MACHINE_RESET_PATH)
    result = _post_json(url, {})
    status = 200 if result.get("ok") else 502
    return jsonify({"result": result}), status


if __name__ == "__main__":
    port = int(os.getenv("PORT", "8200"))
    if QUIET_ACCESS_LOG:
        import logging

        logging.getLogger("werkzeug").setLevel(logging.WARNING)
    app.run(host="0.0.0.0", port=port)
