from __future__ import annotations

import threading
import time
from urllib.parse import urlparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


class DebugStreamServer:
    def __init__(
        self,
        host: str,
        port: int,
        jpeg_quality: int = 80,
        max_fps: float = 15.0,
        tuning_api: object | None = None,
    ):
        self._host = host
        self._port = int(port)
        self._jpeg_quality = int(jpeg_quality)
        self._min_interval_s = 0.0 if max_fps <= 0 else 1.0 / float(max_fps)
        self._last_encode_at = 0.0
        self._lock = threading.Lock()
        self._frame: bytes | None = None
        self._tuning_api = tuning_api
        self._server = ThreadingHTTPServer((self._host, self._port), self._make_handler())
        self._thread = threading.Thread(target=self._server.serve_forever, name="debug-stream", daemon=True)

    def _make_handler(self) -> type[BaseHTTPRequestHandler]:
        parent = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                path = parsed.path

                if path in {"/", "/index.html"}:
                    body = (
                        "<html><body style='margin:0;background:#111'>"
                        "<img id='feed' src='/latest.jpg' style='width:100%;height:auto' />"
                        "<script>"
                        "const img=document.getElementById('feed');"
                        "setInterval(()=>{img.src='/latest.jpg?t='+Date.now();},100);"
                        "</script>"
                        "</body></html>"
                    ).encode("utf-8")
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if path == "/health":
                    body = b'{"status":"ok"}'
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if path == "/tuning/perception":
                    if parent._tuning_api is None:
                        self.send_error(HTTPStatus.NOT_FOUND)
                        return
                    payload = parent._tuning_api.get_payload()
                    parent._send_json(self, HTTPStatus.OK, payload)
                    return

                if path.startswith("/latest.jpg"):
                    frame = parent.latest_frame()
                    if frame is None:
                        self.send_error(HTTPStatus.SERVICE_UNAVAILABLE)
                        return
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Expires", "0")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.flush()
                    return

                if path != "/stream.mjpg":
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return

                self.send_response(HTTPStatus.OK)
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()

                try:
                    while True:
                        frame = parent.latest_frame()
                        if frame is None:
                            time.sleep(0.03)
                            continue
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return

            def do_POST(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                if parsed.path != "/tuning/perception" or parent._tuning_api is None:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                except ValueError:
                    parent._send_json(self, HTTPStatus.BAD_REQUEST, {"message": "Invalid Content-Length."})
                    return
                raw = self.rfile.read(length) if length > 0 else b"{}"
                try:
                    import json
                    payload = json.loads(raw.decode("utf-8") or "{}")
                except (UnicodeDecodeError, json.JSONDecodeError):
                    parent._send_json(self, HTTPStatus.BAD_REQUEST, {"message": "Payload must be valid JSON."})
                    return
                if not isinstance(payload, dict):
                    parent._send_json(self, HTTPStatus.BAD_REQUEST, {"message": "Payload must be an object."})
                    return
                body, status = parent._tuning_api.update(payload)
                parent._send_json(self, status, body)

            def log_message(self, format: str, *args: object) -> None:  # noqa: A003
                return

        return Handler

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=1.0)

    def latest_frame(self) -> bytes | None:
        with self._lock:
            return self._frame

    def _send_json(self, handler: BaseHTTPRequestHandler, status: int, payload: dict) -> None:
        import json

        body = json.dumps(payload).encode("utf-8")
        handler.send_response(status)
        handler.send_header("Content-Type", "application/json")
        handler.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        handler.send_header("Pragma", "no-cache")
        handler.send_header("Expires", "0")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)
        handler.wfile.flush()

    def update(self, frame_bgr: np.ndarray) -> None:
        now = time.time()
        if now - self._last_encode_at < self._min_interval_s:
            return
        ok, encoded = cv2.imencode(
            ".jpg",
            frame_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
        )
        if not ok:
            return
        with self._lock:
            self._frame = encoded.tobytes()
            self._last_encode_at = now
