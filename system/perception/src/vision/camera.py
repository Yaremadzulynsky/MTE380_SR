"""Camera abstraction over OpenCV VideoCapture."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

import cv2
import numpy as np


@dataclass
class OpenCVCamera:
    """Simple camera/video source wrapper."""

    source: int | str
    width: int = 640
    height: int = 480
    fps: float = 30.0
    backend: str = "auto"  # auto | gstreamer | ffmpeg
    threaded: bool = False

    def __post_init__(self) -> None:
        if self.backend == "gstreamer":
            self.cap = cv2.VideoCapture(self.source, cv2.CAP_GSTREAMER)
        elif self.backend == "ffmpeg":
            self.cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
        else:
            self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera/video source: {self.source}")

        # Keep latency small for live feeds.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # For USB webcams this can materially improve capture throughput.
        if isinstance(self.source, int):
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        self._stop = False
        self._eof = False
        self._latest: np.ndarray | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        if self.threaded:
            self._thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._thread.start()

    def _reader_loop(self) -> None:
        while not self._stop:
            ok, frame = self.cap.read()
            if not ok:
                self._eof = True
                break
            with self._lock:
                self._latest = frame

    def read(self) -> np.ndarray | None:
        if self.threaded:
            # Wait briefly for the first frame.
            if self._latest is None and not self._eof:
                deadline = time.perf_counter() + 0.25
                while self._latest is None and not self._eof and time.perf_counter() < deadline:
                    time.sleep(0.001)
            with self._lock:
                if self._latest is None:
                    return None
                return self._latest

        ok, frame = self.cap.read()
        if not ok:
            return None
        return frame

    def release(self) -> None:
        self._stop = True
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        self.cap.release()


class PiCameraStub:
    """Reserved stub for future native PiCamera integration."""

    def __init__(self) -> None:
        raise NotImplementedError("PiCamera backend not implemented in this milestone.")
