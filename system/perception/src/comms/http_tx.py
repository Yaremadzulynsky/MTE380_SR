"""HTTP POST sender for perception packets."""

from __future__ import annotations

import json
import sys
import threading
import urllib.request

def _send_post(url: str, body: bytes, timeout: float = 30.0) -> None:
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
        sys.stderr.write(f"http_tx: POST failed: {e}\n")


class HTTPSender:
    """POST perception packets to an HTTP endpoint as target + red_line/black_line schema."""

    def __init__(self, url: str, timeout: float = 30.0) -> None:
        self.url = url.rstrip("/")
        self.timeout = timeout

    def send_line(self, text: str) -> None:
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            return
        px = data.get("px", 0.0)
        py = data.get("py", 1.0)
        path_detected = data.get("path_detected", False)
        path_mask_key = data.get("path_mask_key", "red")
        # target.detected: True when blue circular blob (TARGET zone), False otherwise
        target_detected = bool(data.get("target_detected", False))
        target_px = data.get("target_px", 0.0)
        target_py = data.get("target_py", 0.0)
        # State machine reads red_line or black_line (prefers black_line when present).
        line_key = "black_line" if path_mask_key == "black" else "red_line"
        payload = {
            "target": {
                "detected": target_detected,
                "vector": {"x": float(target_px), "y": float(target_py)},
            },
            line_key: {
                "detected": bool(path_detected),
                "vector": {"x": float(px), "y": float(py)},
            },
        }
        body = json.dumps(payload).encode("utf-8")
        print(f"[perception] send: {body.decode('utf-8')}", file=sys.stderr)
        t = threading.Thread(
            target=_send_post,
            args=(self.url, body, self.timeout),
            daemon=True,
        )
        t.start()

    def close(self) -> None:
        pass
