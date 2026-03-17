"""Read perception JSON lines from stdin and POST to an inputs endpoint.

Payload schema:
  {
    "target": {"detected": bool, "vector": {"x": float, "y": float}},
    "red_line": {"detected": bool, "vector": {"x": float, "y": float}}
  }
  or blue_line/black_line when path_mask_key is blue/black.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import urllib.request


def _send_request(url: str, body: bytes, timeout: float) -> None:
    """Send one POST in a background thread so the main loop is not blocked."""
    req = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            pass
    except Exception as e:
        sys.stderr.write(f"post_vectors: POST failed: {e}\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read perception packets from stdin; POST red_line/blue_line/black_line schema to URL"
    )
    parser.add_argument(
        "url",
        help="Inputs endpoint URL (e.g. http://localhost:8000/inputs)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="Max POSTs per second (default: 5). Use 1 if the server rate-limits.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="HTTP request timeout in seconds (default: 30)",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Print each request and interval to stderr (to check rate)",
    )
    parser.add_argument(
        "--only-changes",
        action="store_true",
        help="Only send when (x,y) has changed (avoids repeating same vector and 429s)",
    )
    parser.add_argument(
        "--change-threshold",
        type=float,
        default=0.01,
        help="Min change in x or y to count as changed (default: 0.01). Used with --only-changes.",
    )
    args = parser.parse_args()
    url = args.url.rstrip("/")
    min_interval = 1.0 / args.rate if args.rate > 0 else 0.0
    last_send_time = 0.0
    last_sent_px: float | None = None
    last_sent_py: float | None = None
    request_count = 0
    # Always track the latest vector from the pipeline so we send current state, not first-in-window
    latest_px = 0.0
    latest_py = 1.0

    for line in sys.stdin:
        line = line.strip()
        if not line or not line.startswith("{"):
            continue
        try:
            data = json.loads(line)
            latest_px = data.get("px", 0.0)
            latest_py = data.get("py", 1.0)
            latest_detected = data.get("path_detected", False)
            latest_path_mask = data.get("path_mask_key", "red")
            # target.detected: True when blue circular blob (TARGET zone), False otherwise
            latest_target_detected = bool(data.get("target_detected", False))
            latest_target_px = data.get("target_px", 0.0)
            latest_target_py = data.get("target_py", 0.0)
        except (json.JSONDecodeError, TypeError):
            continue

        now = time.monotonic()
        if now - last_send_time < min_interval:
            continue

        if args.only_changes and last_sent_px is not None and last_sent_py is not None:
            if abs(latest_px - last_sent_px) < args.change_threshold and abs(latest_py - last_sent_py) < args.change_threshold:
                continue

        interval = now - last_send_time
        last_send_time = now
        last_sent_px, last_sent_py = latest_px, latest_py
        request_count += 1

        # State machine reads red_line or black_line (prefers black_line when present).
        line_key = "black_line" if latest_path_mask == "black" else "red_line"
        payload = {
            "target": {
                "detected": latest_target_detected,
                "vector": {"x": latest_target_px, "y": latest_target_py},
            },
            line_key: {
                "detected": bool(latest_detected),
                "vector": {"x": latest_px, "y": latest_py},
            },
        }
        body = json.dumps(payload).encode("utf-8")
        sys.stderr.write(f"post_vectors: sending {json.dumps(payload)}\n")
        if args.verbose:
            sys.stderr.write(
                f"post_vectors: request #{request_count} at {interval:.2f}s since last\n"
            )
        # Send in background so we don't block on server response; rate stays steady
        t = threading.Thread(target=_send_request, args=(url, body, args.timeout), daemon=True)
        t.start()


if __name__ == "__main__":
    main()
