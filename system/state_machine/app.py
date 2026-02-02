import json
import os
import time
from dataclasses import dataclass
from enum import Enum
from threading import Event, Lock, Thread
from typing import Optional
from urllib.parse import urljoin

import requests
from flask import Flask, jsonify, request


class State(Enum):
    SEARCHING = "searching"
    FIND_TARGET = "find_target"
    ALIGN_FOR_RETRIEVE = "align_for_retrieve"
    RETRIEVING = "retrieving"
    RETRIEVED = "retrieved"
    TRANSPORTING = "transporting"
    ALIGN_FOR_PLACE = "align_for_place"
    PLACING = "placing"
    PLACE_SUCCESS = "place_success"
    RETURN_HOME = "return_home"
    ERROR_RETRIEVE = "error_retrieve"
    ERROR_PLACE = "error_place"
    END = "end"


@dataclass
class VectorObservation:
    detected: bool
    x: float
    y: float

    @property
    def magnitude(self) -> float:
        return (self.x * self.x + self.y * self.y) ** 0.5

    @classmethod
    def from_dict(cls, data: Optional[dict]) -> "VectorObservation":
        data = data or {}
        detected_value = data.get("detected")
        if detected_value is None:
            detected_value = False
            for key, value in data.items():
                if "detected" in str(key).lower():
                    detected_value = value
                    break
        detected = bool(detected_value)
        vector = data.get("vector") or {}
        x = float(vector.get("x", data.get("x", 0.0)) or 0.0)
        y = float(vector.get("y", data.get("y", 0.0)) or 0.0)
        return cls(detected=detected, x=x, y=y)


@dataclass
class Inputs:
    lego_detected: bool = False
    target_detected: bool = False
    safe_zone_detected: bool = False
    danger_zone_detected: bool = False
    aligned_for_retrieve: bool = False
    aligned_for_place: bool = False
    retrieving: bool = False
    placing: bool = False
    pick_up_success: bool = False
    place_success: bool = False
    failed_pickup: bool = False
    at_home: bool = False


@dataclass
class TransitionResult:
    source: State
    target: State
    label: str


@dataclass
class StateContext:
    failed_pickups: int = 0
    transition_count: int = 0
    last_transition_at: float = 0.0


class InputBuffer:
    def __init__(self, target_align_threshold: float, place_align_threshold: float):
        self._lock = Lock()
        self._raw_payload = {}
        self._inputs = Inputs()
        self._updated_at: Optional[float] = None
        self._target_align_threshold = target_align_threshold
        self._place_align_threshold = place_align_threshold

    def update(self, payload: dict) -> Inputs:
        safe_zone = VectorObservation.from_dict(payload.get("safe_zone"))
        danger_zone = VectorObservation.from_dict(payload.get("danger_zone"))
        target = VectorObservation.from_dict(payload.get("target"))
        _line = VectorObservation.from_dict(payload.get("line"))

        inputs = Inputs(
            lego_detected=target.detected,
            target_detected=target.detected,
            safe_zone_detected=safe_zone.detected,
            danger_zone_detected=danger_zone.detected,
            aligned_for_retrieve=target.detected
            and target.magnitude <= self._target_align_threshold,
            aligned_for_place=safe_zone.detected
            and safe_zone.magnitude <= self._place_align_threshold,
            retrieving=bool(payload.get("retrieving", False)),
            placing=bool(payload.get("placing", False)),
            pick_up_success=bool(payload.get("pick_up_success", False)),
            place_success=bool(payload.get("place_success", False)),
            failed_pickup=bool(payload.get("failed_pickup", False)),
            at_home=bool(payload.get("at_home", False)),
        )

        with self._lock:
            self._raw_payload = payload
            self._inputs = inputs
            self._updated_at = time.time()

        return inputs

    def snapshot(self) -> tuple[Inputs, dict, Optional[float]]:
        with self._lock:
            return self._inputs, dict(self._raw_payload), self._updated_at


class ControlCommClient:
    def __init__(self, base_url: str, state_path: str, timeout: float = 0.5):
        self._state_url = urljoin(base_url.rstrip("/") + "/", state_path.lstrip("/"))
        self._timeout = timeout

    def send_state(self, state: State) -> None:
        payload = {"state": state.value}
        try:
            requests.post(self._state_url, json=payload, timeout=self._timeout)
        except requests.RequestException:
            pass


class MetricsEmitter:
    def __init__(self, path: Optional[str]):
        self._path = path
        self._last_emit = 0.0
        self._interval = float(os.getenv("METRICS_EMIT_INTERVAL", "1.0"))

    def emit(self, state: State, context: StateContext) -> None:
        if not self._path:
            return
        now = time.time()
        if now - self._last_emit < self._interval:
            return
        self._last_emit = now
        lines = [
            f'state_machine_state{{state="{state.value}"}} 1',
            f"state_machine_failed_pickups {context.failed_pickups}",
            f"state_machine_transitions_total {context.transition_count}",
        ]
        try:
            os.makedirs(os.path.dirname(self._path), exist_ok=True)
            with open(self._path, "a", encoding="utf-8") as handle:
                handle.write("\n".join(lines) + "\n")
        except OSError:
            return


class StateMachine:
    def __init__(self, failed_pickup_limit: int = 3):
        self.state = State.SEARCHING
        self.context = StateContext()
        self.failed_pickup_limit = failed_pickup_limit

    def step(self, inputs: Inputs) -> Optional[TransitionResult]:
        if inputs.failed_pickup:
            self.context.failed_pickups += 1
        if inputs.pick_up_success or inputs.place_success:
            self.context.failed_pickups = 0

        next_state = None
        label = None

        match self.state:
            case State.SEARCHING:
                if inputs.lego_detected:
                    next_state = State.FIND_TARGET
                    label = "LD"
            case State.FIND_TARGET:
                if inputs.target_detected or inputs.aligned_for_retrieve:
                    next_state = State.ALIGN_FOR_RETRIEVE
                    label = "TD"
            case State.ALIGN_FOR_RETRIEVE:
                if inputs.aligned_for_retrieve or inputs.retrieving:
                    next_state = State.RETRIEVING
                    label = "AFR"
            case State.RETRIEVING:
                if inputs.pick_up_success:
                    next_state = State.RETRIEVED
                    label = "PUS"
                elif inputs.failed_pickup and self.context.failed_pickups >= self.failed_pickup_limit:
                    next_state = State.ERROR_RETRIEVE
                    label = "FPU>limit"
                elif inputs.failed_pickup:
                    next_state = State.ALIGN_FOR_RETRIEVE
                    label = "FPU"
            case State.RETRIEVED:
                next_state = State.TRANSPORTING
                label = "R"
            case State.TRANSPORTING:
                if inputs.safe_zone_detected:
                    next_state = State.ALIGN_FOR_PLACE
                    label = "SZD"
            case State.ALIGN_FOR_PLACE:
                if inputs.aligned_for_place or inputs.placing:
                    next_state = State.PLACING
                    label = "AFP"
                elif inputs.failed_pickup:
                    next_state = State.ERROR_PLACE
                    label = "FPU"
            case State.PLACING:
                if inputs.place_success:
                    next_state = State.PLACE_SUCCESS
                    label = "PS"
            case State.PLACE_SUCCESS:
                next_state = State.RETURN_HOME
                label = "PS"
            case State.RETURN_HOME:
                if inputs.at_home:
                    next_state = State.END
                    label = "AH"
            case State.ERROR_RETRIEVE:
                if inputs.lego_detected:
                    next_state = State.SEARCHING
                    label = "LD"
                elif inputs.at_home:
                    next_state = State.RETURN_HOME
                    label = "AH"
            case State.ERROR_PLACE:
                if inputs.safe_zone_detected:
                    next_state = State.ALIGN_FOR_PLACE
                    label = "SZD"
                elif inputs.at_home:
                    next_state = State.RETURN_HOME
                    label = "AH"
            case State.END:
                next_state = None

        if not next_state:
            return None

        prev = self.state
        self.state = next_state
        self.context.transition_count += 1
        self.context.last_transition_at = time.time()
        return TransitionResult(prev, next_state, label or "")


def create_logger():
    path = os.getenv("LOG_PATH", "/var/log/state-machine/state-machine.log")
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        return open(path, "a", encoding="utf-8")
    except OSError:
        return None


def log_event(handle, event: str, payload: dict) -> None:
    entry = {
        "ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "event": event,
        "payload": payload,
    }
    line = json.dumps(entry)
    print(line, flush=True)
    if handle:
        handle.write(line + "\n")
        handle.flush()


def run_state_loop(
    state_machine: StateMachine,
    input_buffer: InputBuffer,
    metrics: MetricsEmitter,
    control_comm: Optional[ControlCommClient],
    logger,
    stop_event: Event,
) -> None:
    tick_interval = float(os.getenv("TICK_INTERVAL", "0.2"))

    log_event(logger, "state_machine_started", {"state": state_machine.state.value})

    while not stop_event.is_set() and state_machine.state != State.END:
        inputs, _, _ = input_buffer.snapshot()
        transition = state_machine.step(inputs)
        if transition:
            log_event(
                logger,
                "state_transition",
                {
                    "from": transition.source.value,
                    "to": transition.target.value,
                    "label": transition.label,
                    "failed_pickups": state_machine.context.failed_pickups,
                },
            )
            if control_comm:
                control_comm.send_state(state_machine.state)

        metrics.emit(state_machine.state, state_machine.context)
        time.sleep(tick_interval)

    log_event(logger, "state_machine_completed", {})


def create_app(
    input_buffer: InputBuffer,
    logger,
) -> Flask:
    app = Flask(__name__)

    @app.get("/health")
    def health():
        return jsonify({"status": "ok"})

    @app.get("/inputs")
    def get_inputs():
        inputs, payload, updated_at = input_buffer.snapshot()
        return jsonify(
            {
                "inputs": inputs.__dict__,
                "payload": payload,
                "updated_at": updated_at,
            }
        )

    @app.post("/inputs")
    def post_inputs():
        if not request.is_json:
            return jsonify({"message": "Expected JSON payload."}), 400
        payload = request.get_json(silent=True) or {}
        inputs = input_buffer.update(payload)
        log_event(
            logger,
            "inputs_received",
            {
                "lego_detected": inputs.lego_detected,
                "safe_zone_detected": inputs.safe_zone_detected,
                "danger_zone_detected": inputs.danger_zone_detected,
                "aligned_for_retrieve": inputs.aligned_for_retrieve,
                "aligned_for_place": inputs.aligned_for_place,
            },
        )
        return jsonify({"status": "ok", "inputs": inputs.__dict__})

    return app


def main() -> None:
    failed_pickup_limit = int(os.getenv("FAILED_PICKUP_LIMIT", "3"))
    metrics_path = os.getenv("METRICS_PIPE_PATH", "/var/run/metrics/metrics.prom")
    control_comm_base = os.getenv("CONTROL_COMM_BASE_URL", "")
    control_comm_state_path = os.getenv("CONTROL_COMM_STATE_PATH", "/state")
    target_align_threshold = float(os.getenv("TARGET_ALIGN_THRESHOLD", "0.15"))
    place_align_threshold = float(os.getenv("PLACE_ALIGN_THRESHOLD", "0.2"))
    port = int(os.getenv("PORT", "8000"))

    logger = create_logger()
    metrics = MetricsEmitter(metrics_path)
    input_buffer = InputBuffer(target_align_threshold, place_align_threshold)
    state_machine = StateMachine(failed_pickup_limit=failed_pickup_limit)
    control_comm = (
        ControlCommClient(control_comm_base, control_comm_state_path)
        if control_comm_base
        else None
    )

    stop_event = Event()
    loop_thread = Thread(
        target=run_state_loop,
        args=(state_machine, input_buffer, metrics, control_comm, logger, stop_event),
        daemon=True,
    )
    loop_thread.start()

    app = create_app(input_buffer, logger)
    log_event(logger, "state_machine_api_started", {"port": port})
    try:
        app.run(host="0.0.0.0", port=port)
    finally:
        stop_event.set()
        loop_thread.join(timeout=2)


if __name__ == "__main__":
    main()
