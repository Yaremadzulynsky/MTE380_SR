"""Minimal structured logger."""

from __future__ import annotations

import datetime as _dt
from typing import Any


def log(event: str, **fields: Any) -> None:
    """Print timestamped key/value log line."""
    ts = _dt.datetime.now().isoformat(timespec="milliseconds")
    if not fields:
        print(f"[{ts}] {event}", flush=True)
        return
    flat = " ".join(f"{k}={v}" for k, v in fields.items())
    print(f"[{ts}] {event} {flat}", flush=True)
