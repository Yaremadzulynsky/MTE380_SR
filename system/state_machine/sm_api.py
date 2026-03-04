from typing import Any, Callable, TextIO

from flask import Flask, jsonify, request

from sm_input_buffer import InputBuffer
from sm_logging import log_event
from sm_state_machine import StateMachine
from sm_models import State


def create_app(
    input_buffer: InputBuffer,
    logger: TextIO | None,
    state_machine: StateMachine,
    reset_callback: Callable[[], None] | None = None,
) -> Flask:
    app = Flask(__name__)

    @app.get("/health")
    def health() -> Any:
        return jsonify({"status": "ok"})

    @app.get("/inputs")
    def get_inputs() -> Any:
        inputs, payload, updated_at = input_buffer.snapshot()
        return jsonify(
            {
                "inputs": inputs.__dict__,
                "payload": payload,
                "updated_at": updated_at,
            }
        )


    @app.post("/inputs")
    def post_inputs() -> Any:
        if not request.is_json:
            return jsonify({"message": "Expected JSON payload."}), 400
        payload = request.get_json(silent=True) or {}
        try:
            inputs = input_buffer.update(payload)
        except ValueError as exc:
            return jsonify({"message": str(exc)}), 400
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
        # return jsonify({"status": "ok", "inputs": inputs.__dict__})
        return jsonify({"status": "ok", "inputs": ["inputs_hidden_for_log"]})

    @app.post("/reset")
    def reset() -> Any:
        input_buffer.clear()
        state_machine.reset()
        if reset_callback:
            reset_callback()
        log_event(
            logger,
            "state_machine_reset",
            {"state": state_machine.state.value},
        )
        return jsonify({"status": "ok", "state": state_machine.state.value})

    @app.get("/states")
    def get_states() -> Any:
        return jsonify(
            {
                "states": [state.value for state in State],
                "current_state": state_machine.state.value,
            }
        )

    @app.post("/set-state")
    def set_state() -> Any:
        if not request.is_json:
            return jsonify({"message": "Expected JSON payload."}), 400

        payload = request.get_json(silent=True) or {}
        raw_state = payload.get("state")
        if not isinstance(raw_state, str) or not raw_state.strip():
            return jsonify({"message": "Missing 'state' in payload."}), 400

        target_state = next((state for state in State if state.value == raw_state.strip()), None)
        if target_state is None:
            return jsonify(
                {
                    "message": "Invalid state value.",
                    "allowed_states": [state.value for state in State],
                }
            ), 400

        input_buffer.clear()
        state_machine.force_state(target_state)
        if reset_callback:
            reset_callback()
        log_event(
            logger,
            "state_machine_forced_state",
            {"state": state_machine.state.value},
        )
        return jsonify({"status": "ok", "state": state_machine.state.value})

    return app
