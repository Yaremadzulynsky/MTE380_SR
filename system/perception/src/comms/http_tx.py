"""HTTP POST sender for perception packets."""

from __future__ import annotations

import json
import os
import queue
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
        mode_raw = (os.environ.get("PERCEPTION_HTTP_MODE") or "").strip().lower()
        direct_raw = (os.environ.get("PERCEPTION_HTTP_DIRECT_CONTROL") or "").strip().lower()
        self.direct_control = mode_raw == "control" or direct_raw in {"1", "true", "yes", "on"}
        self._q: "queue.Queue[bytes]" = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def _enqueue(self, body: bytes) -> None:
        try:
            self._q.put_nowait(body)
        except queue.Full:
            try:
                _ = self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait(body)
            except queue.Full:
                pass

    def _worker_loop(self) -> None:
        while not self._stop.is_set():
            try:
                body = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            _send_post(self.url, body, self.timeout)

    def send_line(self, text: str) -> None:
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            return
        px = data.get("px", 0.0)
        py = data.get("py", 1.0)
        line_error_x = data.get("line_error_x", 0.0)
        line_error_y = data.get("line_error_y", py)
        path_detected = data.get("path_detected", False)
        path_mask_key = data.get("path_mask_key", "red")
        # target.detected: True when blue circular blob (TARGET zone), False otherwise
        target_detected = bool(data.get("target_detected", False))
        target_px = data.get("target_px", 0.0)
        target_py = data.get("target_py", 0.0)
        if self.direct_control:
            detected = bool(path_detected)
            steer_x = float(line_error_x)
            steer_y = float(line_error_y)
            if not detected:
                steer_x = 0.0
                steer_y = 0.0
                speed = 0.0
            else:
                speed = min(1.0, float((steer_x * steer_x + steer_y * steer_y) ** 0.5))
            payload = {
                "x": steer_x,
                "y": steer_y,
                "speed": speed,
            }
            body = json.dumps(payload).encode("utf-8")
            print(f"[perception] send_control: {body.decode('utf-8')}", file=sys.stderr)
            self._enqueue(body)
            return
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
            "line_error": {
                "detected": bool(path_detected),
                "vector": {"x": float(line_error_x), "y": float(line_error_y)},
            },
        }
        body = json.dumps(payload).encode("utf-8")
        print(f"[perception] send: {body.decode('utf-8')}", file=sys.stderr)
        self._enqueue(body)

    def close(self) -> None:
        self._stop.set()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
