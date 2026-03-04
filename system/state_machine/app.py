import os
import json
import time
from threading import Event, Thread

from sm_api import create_app
from sm_control_comm import ControlCommClient
from sm_input_buffer import InputBuffer
from sm_logging import create_logger, log_delivery_failure, log_event
from sm_models import State
from sm_state_machine import LINE_FOLLOW_PID_BOUNDS, StateMachine

logger = create_logger()


def load_pid_settings(path: str) -> dict:
    if not path:
        return {}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        if isinstance(payload, dict):
            return payload
    except FileNotFoundError:
        return {}
    except (OSError, json.JSONDecodeError) as exc:
        log_event(logger, "line_follow_pid_load_failed", {"path": path, "error": str(exc)})
        return {}
    return {}


def persist_pid_settings(path: str, state_machine: StateMachine) -> bool:
    if not path:
        return False
    payload = {
        "line_follow_pid": state_machine.get_line_follow_pid_settings(),
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    try:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temp_path = f"{path}.tmp"
        with open(temp_path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temp_path, path)
        return True
    except OSError as exc:
        log_event(logger, "line_follow_pid_persist_failed", {"path": path, "error": str(exc)})
        return False

def main() -> None:
    failed_pickup_limit = int(os.getenv("FAILED_PICKUP_LIMIT", "3"))
    control_comm_base = os.getenv("CONTROL_COMM_BASE_URL", "http://control-communication:5001").strip()
    control_comm_state_path = os.getenv("CONTROL_COMM_STATE_PATH", "/state")
    control_comm_control_path = os.getenv("CONTROL_COMM_CONTROL_PATH", "/control")
    target_align_threshold = float(os.getenv("TARGET_ALIGN_THRESHOLD", "0.15"))
    place_align_threshold = float(os.getenv("PLACE_ALIGN_THRESHOLD", "0.2"))
    state_sync_interval = float(os.getenv("STATE_SYNC_INTERVAL", "1.0"))
    line_pid_settings_path = (
        os.getenv("LINE_PID_SETTINGS_PATH") or "/var/log/state-machine/line-follow-pid.json"
    ).strip()
    port = int(os.getenv("PORT", "8000"))

    # logger = create_logger()
    input_buffer = InputBuffer(target_align_threshold, place_align_threshold)
    state_machine = StateMachine(failed_pickup_limit=failed_pickup_limit)
    persisted_line_pid = load_pid_settings(line_pid_settings_path).get("line_follow_pid")
    if isinstance(persisted_line_pid, dict):
        updated, errors = state_machine.apply_line_follow_pid_updates(persisted_line_pid)
        if errors:
            log_event(
                logger,
                "line_follow_pid_load_rejected",
                {"path": line_pid_settings_path, "errors": errors},
            )
        elif updated:
            log_event(
                logger,
                "line_follow_pid_loaded",
                {"path": line_pid_settings_path, "updated": updated},
            )
    if not control_comm_base:
        raise ValueError(
            "CONTROL_COMM_BASE_URL must be set because state machine requires control communication."
        )
    control_comm = ControlCommClient(
        control_comm_base,
        control_comm_state_path,
        control_path=control_comm_control_path,
    )

    stop_event = Event()

    def state_worker() -> None:
        last_state_sync_at = 0.0
        last_state_sent: str | None = None

        def sync_state(context: str) -> None:
            nonlocal last_state_sync_at, last_state_sent
            result = control_comm.send_state(state_machine.state)
            if bool(result.get("ok")):
                last_state_sync_at = time.time()
                last_state_sent = state_machine.state.value
                return
            log_delivery_failure(
                logger,
                "state_delivery_failed",
                result,
                {"state": state_machine.state.value, "context": context},
            )

        # Publish current state immediately so control-communication `/state`
        # does not remain `unknown` while waiting for a transition.
        sync_state("startup")
        while not stop_event.is_set():
            transition = state_machine.step(input_buffer.latest_inputs())
            now = time.time()
            state_changed = transition is not None
            stale = state_sync_interval > 0 and (now - last_state_sync_at) >= state_sync_interval
            if state_changed or stale or last_state_sent != state_machine.state.value:
                sync_state("state_worker")
            time.sleep(state_machine.tick_interval)

    loop_thread = Thread(
        target=state_worker,
        daemon=True,
    )
    # flag_thread = Thread(
    #     target=run_flag_loop,
    #     args=(input_buffer, stop_event, logger),
    #     daemon=True,
    # )
    loop_thread.start()
    # flag_thread.start()

    def reset_runtime_state() -> None:
        result = control_comm.send_state(state_machine.state)
        if not bool(result.get("ok")):
            log_delivery_failure(
                logger,
                "state_delivery_failed",
                result,
                {"state": state_machine.state.value, "context": "manual_reset"},
            )

    app = create_app(input_buffer, logger, state_machine, reset_callback=reset_runtime_state)

    @app.get("/line-follow-pid")
    def get_line_follow_pid():
        return {
            "values": state_machine.get_line_follow_pid_settings(),
            "bounds": LINE_FOLLOW_PID_BOUNDS,
        }

    @app.post("/line-follow-pid")
    def set_line_follow_pid():
        from flask import jsonify, request
        if not request.is_json:
            return jsonify({"message": "Expected JSON payload."}), 400

        payload = request.get_json(silent=True) or {}
        updates, errors = state_machine.apply_line_follow_pid_updates(payload)
        if errors:
            return jsonify({"message": "Invalid line-follow PID payload.", "errors": errors}), 400

        persist_pid_settings(line_pid_settings_path, state_machine)
        return jsonify(
            {
                "values": state_machine.get_line_follow_pid_settings(),
                "updated": updates,
            }
        )

    persist_pid_settings(line_pid_settings_path, state_machine)
    log_event(logger, "state_machine_api_started", {"port": port})
    try:
        app.run(host="0.0.0.0", port=port)
    finally:
        stop_event.set()
        loop_thread.join(timeout=2)
        # flag_thread.join(timeout=2)


if __name__ == "__main__":
    main()
