import os
import json
import time
from threading import Event, Thread

from sm_api import create_app
from sm_input_buffer import InputBuffer
from sm_logging import create_logger, log_event
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
    target_align_threshold = float(os.getenv("TARGET_ALIGN_THRESHOLD", "0.15"))
    place_align_threshold = float(os.getenv("PLACE_ALIGN_THRESHOLD", "0.2"))
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

    stop_event = Event()

    def state_worker() -> None:
        while not stop_event.is_set():
            state_machine.step(input_buffer.latest_inputs())
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
        return None

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
