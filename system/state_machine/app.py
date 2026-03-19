import os
import time
from threading import Event, Thread

from sm_api import create_app
from sm_input_buffer import InputBuffer
from sm_logging import create_logger, log_event
from sm_state_machine import StateMachine

logger = create_logger()

def main() -> None:
    failed_pickup_limit = int(os.getenv("FAILED_PICKUP_LIMIT", "3"))
    target_align_threshold = float(os.getenv("TARGET_ALIGN_THRESHOLD", "0.15"))
    place_align_threshold = float(os.getenv("PLACE_ALIGN_THRESHOLD", "0.2"))
    port = int(os.getenv("PORT", "8000"))

    # logger = create_logger()
    input_buffer = InputBuffer(target_align_threshold, place_align_threshold)
    state_machine = StateMachine(failed_pickup_limit=failed_pickup_limit)

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
    log_event(logger, "state_machine_api_started", {"port": port})
    try:
        app.run(host="0.0.0.0", port=port)
    finally:
        stop_event.set()
        loop_thread.join(timeout=2)
        # flag_thread.join(timeout=2)


if __name__ == "__main__":
    main()
