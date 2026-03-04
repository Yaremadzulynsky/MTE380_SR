import os
import time
from threading import Event
from typing import Optional, Tuple

from sm_control_comm import ControlCommClient
from sm_input_buffer import InputBuffer
from sm_logging import log_delivery_failure, log_event
from sm_models import State, VectorObservation
from sm_state_machine import StateMachine, clamp, normalized_direction


STOP_COMMAND = {"x": 0.0, "y": 0.0, "speed": 0.0}


def _send_state_update(
    control_comm: Optional[ControlCommClient],
    logger,
    state: State,
    *,
    failure_event: str,
    failure_context: dict,
    success_event: Optional[str] = None,
    success_context: Optional[dict] = None,
) -> bool:
    if not control_comm:
        return False

    result = control_comm.send_state(state)
    if result.get("ok"):
        if success_event:
            log_event(logger, success_event, success_context or {})
        return True

    log_delivery_failure(logger, failure_event, result, failure_context)
    return False


def _send_control_command(
    control_comm: Optional[ControlCommClient],
    logger,
    command: dict,
    *,
    context: str,
) -> None:
    if not control_comm:
        return

    result = control_comm.send_control(
        command["x"],
        command["y"],
        command["speed"],
    )
    if not result.get("ok"):
        log_delivery_failure(
            logger,
            "control_delivery_failed",
            result,
            {"command": command, "context": context},
        )


def _send_search_line_stop(
    control_comm: Optional[ControlCommClient],
    logger,
) -> None:
    _send_control_command(
        control_comm,
        logger,
        STOP_COMMAND,
        context="search_line_stop",
    )
    log_event(logger, "search_line_control_stop", {})


def _handle_searching_control(
    control_comm: Optional[ControlCommClient],
    logger,
    payload: dict,
    *,
    search_line_speed: float,
    search_line_interval: float,
    line_active: bool,
    last_line_command_at: float,
    now: float,
    reverse_direction: bool,
) -> Tuple[bool, float]:
    line = VectorObservation.from_dict(payload.get("line"), "line")
    direction = normalized_direction(line.x, line.y)

    if line.detected and direction:
        should_send = (
            not line_active or (now - last_line_command_at) >= search_line_interval
        )
        if not should_send:
            return line_active, last_line_command_at

        x, y = direction
        if reverse_direction:
            x, y = -x, -y
        command = {
            "x": clamp(float(x), -1.0, 1.0),
            "y": clamp(float(y), -1.0, 1.0),
            "speed": search_line_speed,
        }
        _send_control_command(
            control_comm,
            logger,
            command,
            context="search_line_control",
        )
        log_event(
            logger,
            "search_line_control_command",
            {
                "command": command,
                "line": {
                    "x": line.x,
                    "y": line.y,
                    "magnitude": line.magnitude,
                },
                "reverse_direction": reverse_direction,
            },
        )
        return True, now

    if line_active:
        _send_search_line_stop(control_comm, logger)
        return False, now

    return line_active, last_line_command_at


def process_flags(input_buffer: InputBuffer) -> None:
    input_buffer.recompute_flags()


def run_flag_loop(
    input_buffer: InputBuffer,
    stop_event: Event,
    logger,
) -> None:
    interval = max(float(os.getenv("FLAG_LOOP_INTERVAL", "0.05")), 0.01)
    log_event(logger, "flag_loop_started", {"interval_s": interval})
    while not stop_event.is_set():
        try:
            process_flags(input_buffer)
        except Exception as exc:  # pragma: no cover
            log_event(logger, "flag_loop_error", {"message": str(exc)})
        time.sleep(interval)
    log_event(logger, "flag_loop_stopped", {})



def run_state_loop(
    state_machine: StateMachine,
    input_buffer: InputBuffer,
    control_comm: Optional[ControlCommClient],
    logger,
    stop_event: Event,
) -> None:
    tick_interval = float(os.getenv("TICK_INTERVAL", "0.01"))
    state_sync_interval = float(os.getenv("STATE_SYNC_INTERVAL", "1.0"))
    search_line_speed = clamp(float(os.getenv("SEARCH_LINE_SPEED", "0.4")), 0.0, 1.0)
    search_line_interval = float(os.getenv("SEARCH_LINE_COMMAND_INTERVAL", "0.2"))
    last_line_command_at = 0.0
    last_state_sync_at = 0.0
    line_active = False
    reverse_search_line = False
    

    log_event(logger, "state_machine_started", {"state": state_machine.state.value})
    if control_comm:
        _send_state_update(
            control_comm,
            logger,
            state_machine.state,
            failure_event="state_sync_failed",
            failure_context={"state": state_machine.state.value},
            success_event="state_sync_sent",
            success_context={"state": state_machine.state.value},
        )
        last_state_sync_at = time.time()

    while not stop_event.is_set() and state_machine.state != State.END:
        inputs, payload, _ = input_buffer.snapshot()
        transition = state_machine.step(inputs)
        now = time.time()

        if (
            control_comm
            and state_sync_interval > 0
            and (now - last_state_sync_at) >= state_sync_interval
        ):
            _send_state_update(
                control_comm,
                logger,
                state_machine.state,
                failure_event="state_delivery_failed",
                failure_context={
                    "state": state_machine.state.value,
                    "context": "periodic_sync",
                },
            )
            last_state_sync_at = now

        if transition:
            if transition.target == State.SEARCHING:
                reverse_search_line = transition.source == State.RETRIEVED
            elif transition.target != State.SEARCHING:
                reverse_search_line = False
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
            if control_comm and _send_state_update(
                control_comm,
                logger,
                state_machine.state,
                failure_event="state_delivery_failed",
                failure_context={"state": state_machine.state.value},
            ):
                last_state_sync_at = now

        if control_comm:
            if state_machine.state == State.SEARCHING:
                line_active, last_line_command_at = _handle_searching_control(
                    control_comm,
                    logger,
                    payload,
                    search_line_speed=search_line_speed,
                    search_line_interval=search_line_interval,
                    line_active=line_active,
                    last_line_command_at=last_line_command_at,
                    now=now,
                    reverse_direction=reverse_search_line,
                )
            elif state_machine.state == State.ALIGN_FOR_RETRIEVE:
                target = VectorObservation.from_dict(payload.get("target"), "target")
                direction = normalized_direction(target.x, target.y)

                if target.detected and direction:
                    should_send = (now - last_line_command_at) >= search_line_interval

                    x, y = direction
                    command = {
                        "x": clamp(float(x), -1.0, 1.0),
                        "y": clamp(float(y), -1.0, 1.0),
                        "speed": 0.1,
                    }
                    if should_send:
                        _send_control_command(
                            control_comm,
                            logger,
                            command,
                            context="search_line_control",
                        )
                        last_line_command_at = now
            elif state_machine.state == State.TRANSPORTING:
                # turn 180 degrees 
                pass
            
        time.sleep(tick_interval)

    log_event(logger, "state_machine_completed", {})
