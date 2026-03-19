#!/usr/bin/env python3
"""GPIO button controller for starting/stopping the full robot stack.

GO button:
  - Verifies camera availability
  - Starts docker compose services
  - Starts host-side rpicam perception script

STOP button:
  - Stops host-side perception process
  - Stops docker compose services
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from shutil import which

try:
    from gpiozero import Button
except ImportError as exc:
    print("ERROR: gpiozero is required. Install with: pip install gpiozero", file=sys.stderr)
    raise SystemExit(1) from exc


PROJECT_DIR = Path("/home/passwordiswaterloo/Documents/MTE380_SR/system")
PERCEPTION_SCRIPT = PROJECT_DIR / "run_perception_rpicam.sh"

SERVICES = [
    "control-communication",
    "state-machine",
    "control-screen",
    "computer-vision",
]

GO_PIN = int(os.getenv("GO_BUTTON_PIN", "17"))
STOP_PIN = int(os.getenv("STOP_BUTTON_PIN", "27"))
BOUNCE_TIME = float(os.getenv("BUTTON_BOUNCE_S", "0.2"))
CAMERA_RETRIES = int(os.getenv("CAMERA_RETRIES", "10"))
CAMERA_RETRY_DELAY_S = float(os.getenv("CAMERA_RETRY_DELAY_S", "1.5"))


def log(message: str) -> None:
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] [button-stack] {message}", flush=True)


class StackController:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._perception_proc: subprocess.Popen[str] | None = None
        self._running = False
        self._shutting_down = False

    def _require_prereqs(self) -> None:
        for cmd in ("docker", "rpicam-vid", "python3"):
            if which(cmd) is None:
                raise RuntimeError(f"Required command not found: {cmd}")
        if not PROJECT_DIR.is_dir():
            raise RuntimeError(f"Missing project directory: {PROJECT_DIR}")
        if not PERCEPTION_SCRIPT.exists():
            raise RuntimeError(f"Missing perception script: {PERCEPTION_SCRIPT}")
        if not os.access(PERCEPTION_SCRIPT, os.X_OK):
            raise RuntimeError(f"Perception script is not executable: {PERCEPTION_SCRIPT}")

    def _run_compose(self, compose_args: list[str]) -> None:
        cmd = ["docker", "compose", *compose_args]
        log(f"Running: {' '.join(cmd)}")
        subprocess.run(cmd, cwd=PROJECT_DIR, check=True)

    def _camera_detected(self) -> bool:
        result = subprocess.run(
            ["rpicam-vid", "--list-cameras"],
            capture_output=True,
            text=True,
            check=False,
        )
        out = (result.stdout or "") + (result.stderr or "")
        text = out.lower()
        if any(token in text for token in ("no cameras available", "failed", "error", "cannot", "could not")):
            return False
        return any(line.strip().split(":", 1)[0].isdigit() for line in out.splitlines() if ":" in line)

    def _wait_for_camera(self) -> bool:
        for attempt in range(1, CAMERA_RETRIES + 1):
            if self._camera_detected():
                log("Camera detected.")
                return True
            log(f"Camera not detected (attempt {attempt}/{CAMERA_RETRIES}).")
            time.sleep(CAMERA_RETRY_DELAY_S)
        return False

    def start(self) -> None:
        with self._lock:
            if self._running:
                log("GO pressed, but stack is already running.")
                return
            try:
                self._require_prereqs()
                if not self._wait_for_camera():
                    log("Start aborted: camera was not detected.")
                    return

                self._run_compose(["up", "-d", *SERVICES])
                log("Docker services started.")

                self._perception_proc = subprocess.Popen(
                    [str(PERCEPTION_SCRIPT)],
                    cwd=PROJECT_DIR,
                    preexec_fn=os.setsid,
                    text=True,
                )
                time.sleep(1.0)
                if self._perception_proc.poll() is not None:
                    code = self._perception_proc.returncode
                    self._perception_proc = None
                    raise RuntimeError(f"Perception process exited immediately (code {code}).")

                self._running = True
                log("Perception process started.")
            except Exception as exc:  # noqa: BLE001
                log(f"Start failed: {exc}")
                self._running = False

    def stop(self) -> None:
        with self._lock:
            if not self._running and self._perception_proc is None:
                log("STOP pressed, but stack is already stopped.")
                return

            if self._perception_proc and self._perception_proc.poll() is None:
                log("Stopping perception process...")
                try:
                    os.killpg(os.getpgid(self._perception_proc.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    self._perception_proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    log("Perception did not stop in time, force killing...")
                    os.killpg(os.getpgid(self._perception_proc.pid), signal.SIGKILL)
                    self._perception_proc.wait(timeout=5)
            self._perception_proc = None

            try:
                self._run_compose(["stop", *SERVICES])
                log("Docker services stopped.")
            except Exception as exc:  # noqa: BLE001
                log(f"Docker stop failed: {exc}")

            self._running = False

    def cleanup(self) -> None:
        if self._shutting_down:
            return
        self._shutting_down = True
        log("Shutting down controller; stopping stack...")
        self.stop()


def main() -> int:
    controller = StackController()

    # Pull-up wiring expectation: button press connects GPIO to GND.
    go_btn = Button(GO_PIN, pull_up=True, bounce_time=BOUNCE_TIME)
    stop_btn = Button(STOP_PIN, pull_up=True, bounce_time=BOUNCE_TIME)

    go_btn.when_pressed = controller.start
    stop_btn.when_pressed = controller.stop

    def handle_signal(signum: int, _frame: object) -> None:
        log(f"Received signal {signum}.")
        controller.cleanup()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    log(f"Ready. GO pin={GO_PIN}, STOP pin={STOP_PIN}. Press Ctrl+C to exit.")
    while True:
        # Keep process alive and detect unexpected perception exits.
        proc = controller._perception_proc
        if proc is not None and proc.poll() is not None:
            log(f"Perception exited unexpectedly with code {proc.returncode}.")
            controller._perception_proc = None
            controller._running = False
        time.sleep(0.5)


if __name__ == "__main__":
    raise SystemExit(main())
