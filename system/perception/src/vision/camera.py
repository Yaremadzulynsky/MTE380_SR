"""Camera abstraction over OpenCV VideoCapture and rpicam-vid (Pi Camera Module)."""

from __future__ import annotations

import os
import subprocess
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

# JPEG stream markers
_JPEG_SOI = bytes([0xFF, 0xD8])
_JPEG_EOI = bytes([0xFF, 0xD9])


def build_gstreamer_pipeline(
    source: int | str,
    width: int = 640,
    height: int = 480,
    fps: float = 30.0,
    device: str | None = None,
) -> str:
    """
    Build a GStreamer pipeline string for OpenCV VideoCapture.
    Pipeline must end with appsink for OpenCV to consume frames.
    """
    if isinstance(source, int):
        dev = device or f"/dev/video{source}"
        # v4l2src -> capsfilter (w,h,fps) -> videoconvert -> BGR -> appsink
        return (
            f"v4l2src device={dev} ! "
            f"video/x-raw,width={width},height={height},framerate={int(fps)}/1 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=1 max-buffers=1"
        )
    # Video file
    path = str(source)
    return (
        f"filesrc location={path} ! "
        "decodebin ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1"
    )


@dataclass
class OpenCVCamera:
    """Simple camera/video source wrapper."""

    source: int | str
    width: int = 640
    height: int = 480
    fps: float = 30.0
    backend: str = "auto"  # auto | gstreamer | ffmpeg
    threaded: bool = False
    gstreamer_device: str | None = None  # override /dev/videoN for GStreamer

    def __post_init__(self) -> None:
        if self.backend == "gstreamer":
            pipeline = build_gstreamer_pipeline(
                self.source,
                width=self.width,
                height=self.height,
                fps=self.fps,
                device=self.gstreamer_device,
            )
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not self.cap.isOpened():
                # Pip opencv lacks GStreamer; opencv-python-custom-gst can conflict with system
                # GStreamer (GLib type errors). Fallback to default backend for local dev.
                self.cap = cv2.VideoCapture(self.source)
        elif self.backend == "ffmpeg":
            # Reduce RTSP latency for perception consumers unless user overrides.
            if isinstance(self.source, str) and self.source.startswith("rtsp://"):
                if not os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS"):
                    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                        "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0"
                    )
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


def _read_one_jpeg_from_stream(stream: bytes) -> tuple[bytes | None, int]:
    """Extract one JPEG from the start of stream. Returns (jpeg_bytes or None, bytes_consumed)."""
    start = stream.find(_JPEG_SOI)
    if start < 0:
        return None, len(stream)
    end = stream.find(_JPEG_EOI, start)
    if end < 0:
        return None, start
    end += len(_JPEG_EOI)
    return stream[start:end], end


class RpicamVidCamera:
    """
    Raspberry Pi Camera Module (libcamera) via rpicam-vid subprocess.
    Uses MJPEG to stdout so we don't need V4L2. Requires rpicam-apps on the system.
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
        camera_index: int = 0,
    ) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self._process: subprocess.Popen | None = None
        self._buffer = bytearray()
        self._lock = threading.Lock()
        self._latest: np.ndarray | None = None
        self._thread: threading.Thread | None = None
        self._stop = False
        self._eof = False
        cmd = [
            "rpicam-vid",
            "-t", "0",
            "-o", "-",
            "--codec", "mjpeg",
            "-n",
            "--width", str(width),
            "--height", str(height),
            "--framerate", str(int(fps)),
            "--camera", str(camera_index),
        ]
        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except FileNotFoundError as e:
            raise RuntimeError(
                "rpicam-vid not found. Install rpicam-apps (e.g. apt install rpicam-apps) for Pi Camera Module."
            ) from e
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self) -> None:
        if self._process is None or self._process.stdout is None:
            return
        while not self._stop:
            chunk = self._process.stdout.read(65536)
            if not chunk:
                self._eof = True
                break
            with self._lock:
                self._buffer.extend(chunk)
            while True:
                with self._lock:
                    data = bytes(self._buffer)
                jpeg_bytes, consumed = _read_one_jpeg_from_stream(data)
                if jpeg_bytes is None:
                    if consumed > 0:
                        with self._lock:
                            del self._buffer[:consumed]
                    break
                with self._lock:
                    del self._buffer[:consumed]
                arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    with self._lock:
                        self._latest = frame

    def read(self) -> np.ndarray | None:
        # Allow a bit of startup latency for rpicam-vid and keep the latest frame.
        deadline = time.perf_counter() + 2.0
        while self._latest is None and not self._eof and time.perf_counter() < deadline:
            time.sleep(0.005)
        with self._lock:
            return self._latest

    def release(self) -> None:
        self._stop = True
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._process is not None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None


class PiCameraStub:
    """Reserved stub for future native PiCamera integration."""

    def __init__(self) -> None:
        raise NotImplementedError("PiCamera backend not implemented in this milestone.")


# """Camera abstraction over OpenCV VideoCapture and TCP-streamed Pi camera."""

# from __future__ import annotations

# import threading
# import time
# from dataclasses import dataclass

# import cv2
# import numpy as np


# def build_gstreamer_pipeline(
#     source: int | str,
#     width: int = 640,
#     height: int = 480,
#     fps: float = 30.0,
#     device: str | None = None,
# ) -> str:
#     """
#     Build a GStreamer pipeline string for OpenCV VideoCapture.
#     Pipeline must end with appsink for OpenCV to consume frames.
#     """
#     if isinstance(source, int):
#         dev = device or f"/dev/video{source}"
#         return (
#             f"v4l2src device={dev} ! "
#             f"video/x-raw,width={width},height={height},framerate={int(fps)}/1 ! "
#             "videoconvert ! video/x-raw,format=BGR ! "
#             "appsink drop=1 max-buffers=1"
#         )

#     path = str(source)

#     # If this is a TCP/UDP stream, let GStreamer decode it.
#     if path.startswith(("tcp://", "udp://", "rtsp://")):
#         return (
#             f"urisourcebin uri={path} ! "
#             "decodebin ! videoconvert ! video/x-raw,format=BGR ! "
#             "appsink drop=1 max-buffers=1"
#         )

#     # Otherwise assume file input.
#     return (
#         f"filesrc location={path} ! "
#         "decodebin ! videoconvert ! video/x-raw,format=BGR ! "
#         "appsink drop=1 max-buffers=1"
#     )


# @dataclass
# class OpenCVCamera:
#     """Simple camera/video source wrapper."""

#     source: int | str
#     width: int = 640
#     height: int = 480
#     fps: float = 30.0
#     backend: str = "auto"  # auto | gstreamer | ffmpeg
#     threaded: bool = False
#     gstreamer_device: str | None = None  # override /dev/videoN for GStreamer

#     def __post_init__(self) -> None:
#         if self.backend == "gstreamer":
#             pipeline = build_gstreamer_pipeline(
#                 self.source,
#                 width=self.width,
#                 height=self.height,
#                 fps=self.fps,
#                 device=self.gstreamer_device,
#             )
#             self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
#             if not self.cap.isOpened():
#                 self.cap = cv2.VideoCapture(self.source)
#         elif self.backend == "ffmpeg":
#             self.cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
#         else:
#             self.cap = cv2.VideoCapture(self.source)

#         if not self.cap.isOpened():
#             raise RuntimeError(f"Could not open camera/video source: {self.source}")

#         # Keep latency small for live feeds.
#         self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
#         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
#         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
#         self.cap.set(cv2.CAP_PROP_FPS, self.fps)

#         # For USB webcams this can materially improve capture throughput.
#         if isinstance(self.source, int):
#             fourcc = cv2.VideoWriter_fourcc(*"MJPG")
#             self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)

#         self._stop = False
#         self._eof = False
#         self._latest: np.ndarray | None = None
#         self._lock = threading.Lock()
#         self._thread: threading.Thread | None = None

#         if self.threaded:
#             self._thread = threading.Thread(target=self._reader_loop, daemon=True)
#             self._thread.start()

#     def _reader_loop(self) -> None:
#         while not self._stop:
#             ok, frame = self.cap.read()
#             if not ok:
#                 self._eof = True
#                 break
#             with self._lock:
#                 self._latest = frame

#     def read(self) -> np.ndarray | None:
#         if self.threaded:
#             if self._latest is None and not self._eof:
#                 deadline = time.perf_counter() + 0.25
#                 while self._latest is None and not self._eof and time.perf_counter() < deadline:
#                     time.sleep(0.001)
#             with self._lock:
#                 return self._latest

#         ok, frame = self.cap.read()
#         if not ok:
#             return None
#         return frame

#     def release(self) -> None:
#         self._stop = True
#         if self._thread is not None and self._thread.is_alive():
#             self._thread.join(timeout=0.5)
#         self.cap.release()


# class RpicamVidCamera:
#     """
#     Temporary compatibility wrapper.

#     Instead of launching rpicam-vid locally, this class connects directly to a
#     host-provided TCP stream. This keeps the rest of the app unchanged while
#     bypassing all in-container Pi camera handling.

#     Expected host stream example:
#         rpicam-vid -n -t 0 --inline --codec libav --libav-format mpegts --listen -o tcp://0.0.0.0:8555
#     """

#     def __init__(
#         self,
#         width: int = 640,
#         height: int = 480,
#         fps: float = 30.0,
#         camera_index: int = 0,  # kept for compatibility; ignored
#         stream_url: str = "udp://127.0.0.1:8555",
#     ) -> None:
#         self.width = width
#         self.height = height
#         self.fps = fps
#         self.stream_url = stream_url

#         self._cam = OpenCVCamera(
#             source=self.stream_url,
#             width=self.width,
#             height=self.height,
#             fps=self.fps,
#             backend="ffmpeg",
#             threaded=False,
#         )

#     def read(self) -> np.ndarray | None:
#         return self._cam.read()

#     def release(self) -> None:
#         self._cam.release()


# class PiCameraStub:
#     """Reserved stub for future native PiCamera integration."""

#     def __init__(self) -> None:
#         raise NotImplementedError("PiCamera backend not implemented in this milestone.")