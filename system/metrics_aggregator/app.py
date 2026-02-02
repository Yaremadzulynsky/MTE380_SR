import os
import stat
import sys
import time
from collections import deque
from threading import Lock, Thread

from flask import Flask, Response, jsonify
from prometheus_client import Counter, Gauge, CONTENT_TYPE_LATEST, generate_latest

METRICS_PIPE_PATH = os.getenv("METRICS_PIPE_PATH", "/var/run/metrics/metrics.prom")
METRICS_READ_FROM_START = os.getenv("METRICS_READ_FROM_START", "true").lower() in {
    "1",
    "true",
    "yes",
}
METRICS_MAX_LINES = int(os.getenv("METRICS_MAX_LINES", "2000"))
LOG_PATH = os.getenv("LOG_PATH", "/var/log/metrics-aggregator/metrics-aggregator.log")

app = Flask(__name__)

METRICS_LINES_TOTAL = Counter(
    "aggregator_metrics_lines_total", "Metrics lines ingested", []
)
METRICS_BUFFER_LINES = Gauge(
    "aggregator_metrics_buffer_lines", "Metrics lines buffered", []
)


class MetricsBuffer:
    def __init__(self, max_lines: int):
        self._lines = deque(maxlen=max_lines)
        self._lock = Lock()

    def add(self, line: str) -> None:
        with self._lock:
            self._lines.append(line)
            METRICS_BUFFER_LINES.set(len(self._lines))

    def render(self) -> str:
        with self._lock:
            if not self._lines:
                return ""
            return "\n".join(self._lines) + "\n"


METRICS_BUFFER = MetricsBuffer(METRICS_MAX_LINES)
LOG_FILE_HANDLE = None


def ensure_dir(path: str) -> None:
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)


def is_regular_file(path: str) -> bool:
    try:
        return stat.S_ISREG(os.stat(path).st_mode)
    except FileNotFoundError:
        return False


def follow_path(path: str, read_from_start: bool, on_line, name: str) -> None:
    ensure_dir(path)
    while True:
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as file_handle:
                if is_regular_file(path) and not read_from_start:
                    file_handle.seek(0, os.SEEK_END)

                while True:
                    line = file_handle.readline()
                    if line:
                        on_line(line.rstrip("\n"))
                    else:
                        time.sleep(0.2)
        except FileNotFoundError:
            time.sleep(1)
        except Exception as exc:  # pylint: disable=broad-except
            log_service_line(f"{name} reader error: {exc}")
            time.sleep(1)


def handle_metrics_line(line: str) -> None:
    if not line:
        return
    METRICS_LINES_TOTAL.inc()
    METRICS_BUFFER.add(line)


def init_logger() -> None:
    global LOG_FILE_HANDLE
    try:
        ensure_dir(LOG_PATH)
        LOG_FILE_HANDLE = open(LOG_PATH, "a", encoding="utf-8")
    except OSError as exc:
        print(f"Unable to open log file: {exc}", file=sys.stderr)


def log_service_line(message: str) -> None:
    line = f"{time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())} {message}"
    print(line, flush=True)
    if LOG_FILE_HANDLE:
        LOG_FILE_HANDLE.write(line + "\n")
        LOG_FILE_HANDLE.flush()


@app.get("/health")
def health():
    return jsonify({"status": "ok"})


@app.get("/metrics")
def metrics():
    internal = generate_latest()
    extra = METRICS_BUFFER.render()
    if not extra:
        return Response(internal, mimetype=CONTENT_TYPE_LATEST)

    return Response(
        internal + extra.encode("utf-8"),
        mimetype=CONTENT_TYPE_LATEST,
    )


def start_background_reader() -> None:
    Thread(
        target=follow_path,
        args=(
            METRICS_PIPE_PATH,
            METRICS_READ_FROM_START,
            handle_metrics_line,
            "metrics",
        ),
        daemon=True,
    ).start()


if __name__ == "__main__":
    port = int(os.getenv("PORT", "7000"))
    init_logger()
    log_service_line(f"Metrics aggregator running on 0.0.0.0:{port}")
    start_background_reader()
    app.run(host="0.0.0.0", port=port)
