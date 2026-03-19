"""Minimal HTTP debug stream for viewing perception overlays in a browser."""

from __future__ import annotations

import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


class DebugStreamServer:
    """Serve the latest annotated frame as MJPEG plus a small status page."""

    def __init__(self, host: str, port: int, jpeg_quality: int = 80) -> None:
        self.host = host
        self.port = port
        self.jpeg_quality = max(1, min(int(jpeg_quality), 100))
        self._lock = threading.Lock()
        self._latest_jpeg: bytes | None = None
        self._latest_status = "waiting for frames"
        self._server = ThreadingHTTPServer((self.host, self.port), self._make_handler())
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)

    def _make_handler(self) -> type[BaseHTTPRequestHandler]:
        parent = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                if self.path in {"/", "/index.html"}:
                    self._serve_index()
                    return
                if self.path == "/stream.mjpg":
                    self._serve_stream()
                    return
                if self.path == "/healthz":
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(parent._get_status().encode("utf-8"))
                    return
                self.send_error(HTTPStatus.NOT_FOUND)

            def log_message(self, format: str, *args: object) -> None:  # noqa: A003
                return

            def _serve_index(self) -> None:
                body = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Perception Debug Stream</title>
  <style>
    body {{ background: #111; color: #f5f5f5; font-family: Helvetica, Arial, sans-serif; margin: 0; }}
    main {{ padding: 16px; }}
    img {{ width: min(100%, 960px); border: 1px solid #444; display: block; }}
    p {{ color: #bbb; }}
  </style>
</head>
<body>
  <main>
    <h1>Perception Debug Stream</h1>
    <p>Status: {parent._get_status()}</p>
    <img src="/stream.mjpg" alt="Perception overlay stream">
  </main>
</body>
</html>"""
                encoded = body.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def _serve_stream(self) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()

                try:
                    while True:
                        frame = parent.get_frame()
                        if frame is None:
                            self.wfile.write(b"--frame\r\nContent-Type: text/plain\r\n\r\nwaiting for frames\r\n")
                            self.wfile.flush()
                            threading.Event().wait(0.25)
                            continue
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        threading.Event().wait(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    return

        return Handler

    def start(self) -> None:
        self._thread.start()

    def update_frame(self, frame: np.ndarray, status: str) -> None:
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return
        with self._lock:
            self._latest_jpeg = encoded.tobytes()
            self._latest_status = status

    def get_frame(self) -> bytes | None:
        with self._lock:
            return self._latest_jpeg

    def _get_status(self) -> str:
        with self._lock:
            return self._latest_status

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
