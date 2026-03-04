import json
import os
import time
from typing import Any, TextIO


def create_logger() -> TextIO | None:
    path = os.getenv("LOG_PATH", "/var/log/state-machine/state-machine.log")
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        return open(path, "a", encoding="utf-8")
    except OSError:
        return None


def log_event(handle: TextIO | None, event: str, payload: dict[str, Any]) -> None:
    entry = {
        "ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "event": event,
        "payload": payload,
    }
    line = json.dumps(entry)
    print(line, flush=True)
    if handle:
        handle.write(line + "\n")
        handle.flush()


def log_delivery_failure(
    logger: TextIO | None,
    event: str,
    result: dict[str, Any],
    payload: dict[str, Any],
) -> None:
    log_payload = {
        **payload,
        "url": result.get("url"),
        "status_code": result.get("status_code"),
        "error": result.get("error"),
    }
    detail = result.get("detail")
    if detail:
        log_payload["detail"] = detail
    log_event(logger, event, log_payload)
