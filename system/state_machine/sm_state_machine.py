import math
import os
import threading
import time
from typing import Optional

from sm_control_comm import ControlCommClient
from sm_models import Inputs, State, StateContext, TransitionResult, Vector


Command = dict[str, float]
STOP_COMMAND: Command = {"x": 0.0, "y": 0.0, "speed": 0.0}
MAX_CONTROL_SPEED = 5.0
SERVO_MIN_DEG = int(os.getenv("SERVO_MIN_DEG", "0"))
SERVO_MAX_DEG = int(os.getenv("SERVO_MAX_DEG", "90"))
SERVO_CENTER_DEG = int(os.getenv("SERVO_CENTER_DEG", "45"))
val = 0

LINE_FOLLOW_PID_BOUNDS: dict[str, tuple[float, float]] = {
    "kp": (0.0, 20.0),
    "ki": (0.0, 20.0),
    "kd": (0.0, 20.0),
    "i_max": (0.0, 20.0),
    "out_max": (0.0, 20.0),
    "base_speed": (0.0, 5.0),
    "min_speed": (0.0, 5.0),
    "max_speed": (0.0, 5.0),
    "follow_max_speed": (0.0, 5.0),
    "turn_slowdown": (0.0, 5.0),
    "error_slowdown": (0.0, 5.0),
    "deadband": (0.0, 1.0),
}

control = ControlCommClient(
    base_url=os.getenv("CONTROL_COMM_BASE_URL", "http://localhost:5001").strip(),
    state_path=os.getenv("CONTROL_COMM_STATE_PATH", "/state"),
    control_path=os.getenv("CONTROL_COMM_CONTROL_PATH", "/control"),
)

def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def parse_env_bool(name: str, default: str = "0") -> bool:
    raw = (os.getenv(name, default) or "").strip().lower()
    return raw in {"1", "true", "yes", "on"}


LINE_FOLLOW_DEBUG = parse_env_bool("LINE_FOLLOW_DEBUG", "0")
ASYNC_CONTROL_SENDER = parse_env_bool("ASYNC_CONTROL_SENDER", "1")
ASYNC_CONTROL_SEND_HZ = max(float(os.getenv("ASYNC_CONTROL_SEND_HZ", "50.0")), 1.0)


class _AsyncControlSender:
    def __init__(self, client: ControlCommClient, send_hz: float) -> None:
        self._client = client
        self._interval = 1.0 / max(send_hz, 1.0)
        self._lock = threading.Lock()
        self._latest: tuple[float, float, float, int | None] | None = None
        self._event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sm_async_control_sender", daemon=True)
        self._thread.start()

    def set_latest(self, x: float, y: float, speed: float, servo: int | None) -> None:
        with self._lock:
            self._latest = (float(x), float(y), float(speed), int(servo) if servo is not None else None)
        self._event.set()

    def _run(self) -> None:
        next_deadline = time.monotonic()
        while True:
            timeout = max(0.0, next_deadline - time.monotonic())
            self._event.wait(timeout=timeout)
            self._event.clear()
            with self._lock:
                cmd = self._latest
            if cmd is not None:
                x, y, speed, servo = cmd
                _ = self._client.send_control(x, y, speed, servo=servo)
            next_deadline = time.monotonic() + self._interval


_ASYNC_SENDER = _AsyncControlSender(control, ASYNC_CONTROL_SEND_HZ) if ASYNC_CONTROL_SENDER else None


def wrap_angle(rad: float) -> float:
    a = rad
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def normalized_direction(
    x: float, y: float, eps: float = 1e-6
) -> Optional[tuple[float, float]]:
    magnitude = (x * x + y * y) ** 0.5
    if magnitude <= eps:
        return None
    return x / magnitude, y / magnitude


def send_stop_command(context: str = "stop") -> None:
    _send_control_command(
        x=STOP_COMMAND["x"],
        y=STOP_COMMAND["y"],
        speed=STOP_COMMAND["speed"],
        servo=SERVO_CENTER_DEG,
        context=context,
    )


def _angle_rad_to_servo_deg(angle_rad: float) -> int:
    # Map steering angle [-pi/2, +pi/2] to servo range [min, max].
    clamped = clamp(angle_rad, -math.pi / 2.0, math.pi / 2.0)
    unit = (clamped + (math.pi / 2.0)) / math.pi
    servo = int(round(SERVO_MIN_DEG + unit * (SERVO_MAX_DEG - SERVO_MIN_DEG)))
    return int(clamp(servo, SERVO_MIN_DEG, SERVO_MAX_DEG))


def _send_control_command(
    x: float,
    y: float,
    speed: float,
    context: str,
    servo: int | None = None,
) -> None:
    if _ASYNC_SENDER is not None:
        _ASYNC_SENDER.set_latest(x, y, speed, servo)
        return
    result = control.send_control(x, y, speed, servo=servo)
    # Keep failure handling minimal for now; context retained for future logs.
    if not bool(result.get("ok")):
        _ = context


def _send_control_polar(angle_rad: float, speed: float, context: str) -> None:
    clamped_speed = clamp(speed, 0.0, MAX_CONTROL_SPEED)
    if clamped_speed <= 1e-6:
        send_stop_command(context=context)
        return
    x = math.sin(angle_rad)
    y = math.cos(angle_rad)
    _send_control_command(
        x=x,
        y=y,
        speed=clamped_speed,
        context=context,
        servo=_angle_rad_to_servo_deg(angle_rad),
    )


class StateMachine:
    FOLLOW_LINE_PHASE_IDLE = "idle"
    FOLLOW_LINE_PHASE_SCAN_LEFT = "to_left"
    FOLLOW_LINE_PHASE_SCAN_RIGHT = "to_right"

    def __init__(self, failed_pickup_limit: int = 3) -> None:
        self.state = State.REMOTE_CONTROL
        # self.state = State.SEARCHING
        self.context = StateContext()
        self.failed_pickup_limit = failed_pickup_limit
        self.tick_interval = float(os.getenv("TICK_INTERVAL", "0.01"))
        self.search_line_interval = float(os.getenv("SEARCH_LINE_COMMAND_INTERVAL", "0.2"))
        self.pid_relinquish_speed = clamp(float(os.getenv("PID_RELINQUISH_SPEED", "0.0")), 0.0, 1.0)
        self.pid_relinquish_y = clamp(float(os.getenv("PID_RELINQUISH_Y", "1.0")), -1.0, 1.0)
        self.retrieved_turn_speed = clamp(float(os.getenv("RETRIEVED_TURN_SPEED", "0.35")), 0.0, 1.0)
        self.retrieved_turn_tol_rad = clamp(float(os.getenv("RETRIEVED_TURN_TOL_RAD", "0.12")), 0.01, math.pi)
        self.line_pid_kp = float(os.getenv("LINE_PID_KP", "0.2"))
        self.line_pid_ki = float(os.getenv("LINE_PID_KI", "0.04"))
        self.line_pid_kd = float(os.getenv("LINE_PID_KD", "1.92"))
        self.line_pid_i_max = abs(float(os.getenv("LINE_PID_I_MAX", "20")))
        self.line_pid_out_max = abs(float(os.getenv("LINE_PID_OUT_MAX", "20")))
        self.line_pid_base_speed = clamp(float(os.getenv("LINE_PID_BASE_SPEED", "0.2")), 0.0, 5.0)
        self.line_pid_min_speed = clamp(float(os.getenv("LINE_PID_MIN_SPEED", "0.05")), 0.0, 5.0)
        self.line_pid_max_speed = clamp(float(os.getenv("LINE_PID_MAX_SPEED", "0.3")), 0.0, 5.0)
        self.line_pid_follow_max_speed = clamp(float(os.getenv("LINE_PID_FOLLOW_MAX_SPEED", "0.3")), 0.0, 5.0)
        self.line_pid_turn_slowdown = abs(float(os.getenv("LINE_PID_TURN_SLOWDOWN", "5")))
        self.line_pid_error_slowdown = abs(float(os.getenv("LINE_PID_ERROR_SLOWDOWN", "0.14")))
        self.line_pid_deadband = abs(float(os.getenv("LINE_PID_DEADBAND", "0.01")))
        self.line_pid_sync_interval = max(float(os.getenv("LINE_PID_SYNC_INTERVAL", "0.4")), 0.05)
        self.line_pid_dt_min = 1e-3
        self.line_pid_dt_max = 0.25
        self._line_pid_integral = 0.0
        self._line_pid_prev_error = 0.0
        self._line_pid_last_at = 0.0
        self._line_pid_last_sync_at = 0.0
        self.find_line_speed = clamp(float(os.getenv("FIND_LINE_SPEED", "0.02")), 0.0, 1.0)
        self.line_lost_timeout_s = max(float(os.getenv("LINE_LOST_TIMEOUT_S", "0.8")), 0.05)
        self.scan_switch_interval_s = max(float(os.getenv("SCAN_SWITCH_INTERVAL_S", "0.35")), 0.05)
        self._find_line_phase = self.FOLLOW_LINE_PHASE_IDLE
        self._line_last_turn_sign = 1.0
        self._scan_next_right = True
        self._scan_started_at = 0.0
        self._line_lost_since = 0.0
        self._debug_last_at = 0.0
        self._debug_interval_s = max(float(os.getenv("LINE_FOLLOW_DEBUG_INTERVAL_S", "0.2")), 0.05)
        self._retrieved_turn_target_heading: float | None = None

       
        self.remote_control_poll_interval = max(
            float(os.getenv("REMOTE_CONTROL_POLL_INTERVAL", "0.1")),
            0.5,
        )
        self._last_remote_control_poll_at = 0.0
        self._last_command_at = 0.0

    def _fetch_remote_control_command(self) -> Command | None:
        result = control.get_control()
        if not bool(result.get("ok")):
            return None

        data = result.get("data")
        if not isinstance(data, dict):
            return None

        raw = data.get("command", data)
        if not isinstance(raw, dict):
            return None

        try:
            x = float(raw.get("x", 0.0))
            y = float(raw.get("y", 0.0))
            speed = float(raw.get("speed", 0.0))
        except (TypeError, ValueError):
            return None

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(speed)):
            return None

        x = clamp(x, -1.0, 1.0)
        y = clamp(y, -1.0, 1.0)
        speed = clamp(speed, 0.0, MAX_CONTROL_SPEED)
        if ((x * x + y * y) ** 0.5) <= 1e-6 or speed <= 1e-6:
            return dict(STOP_COMMAND)
        return {"x": x, "y": y, "speed": speed}

    def _relinquish_to_pid_command(self) -> Command:
        return {"x": 0.0, "y": self.pid_relinquish_y, "speed": self.pid_relinquish_speed}

    def _can_send_command(self) -> bool:
        now = time.monotonic()
        return (now - self._last_command_at) >= self.search_line_interval

    def _reset_line_pid(self) -> None:
        self._line_pid_integral = 0.0
        self._line_pid_prev_error = 0.0
        self._line_pid_last_at = 0.0

    def _reset_find_line(self) -> None:
        self._find_line_phase = self.FOLLOW_LINE_PHASE_IDLE
        self._line_lost_since = 0.0
        self._scan_started_at = 0.0

    def _begin_follow_line_scan(self, now: float) -> None:
        self._reset_line_pid()
        # Turn opposite to the last PID turn direction: if we were steering right
        # when the line disappeared, turn left to come back to it.
        if self._line_last_turn_sign >= 0.0:
            self._find_line_phase = self.FOLLOW_LINE_PHASE_SCAN_LEFT
        else:
            self._find_line_phase = self.FOLLOW_LINE_PHASE_SCAN_RIGHT
        self._scan_started_at = now
        if self._line_lost_since <= 0.0:
            self._line_lost_since = now

    def _maybe_flip_scan_direction(self, now: float) -> None:
        if self._scan_started_at <= 0.0:
            self._scan_started_at = now
            return
        if (now - self._scan_started_at) < self.scan_switch_interval_s:
            return
        self._find_line_phase = (
            self.FOLLOW_LINE_PHASE_SCAN_LEFT
            if self._find_line_phase == self.FOLLOW_LINE_PHASE_SCAN_RIGHT
            else self.FOLLOW_LINE_PHASE_SCAN_RIGHT
        )
        self._scan_started_at = now

    def _debug_line_follow(self, *, now: float, stage: str, context: str, payload: dict[str, float | str | bool]) -> None:
        if not LINE_FOLLOW_DEBUG:
            return
        if (now - self._debug_last_at) < self._debug_interval_s:
            return
        fields = " ".join(f"{k}={v}" for k, v in payload.items())
        print(
            f'ts={time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())} '
            f"event=line_follow_debug stage={stage} state={self.state.value} context={context} {fields}",
            flush=True,
        )
        self._debug_last_at = now

    def _step_follow_line_scan(self, *, line: Vector, now: float, context: str) -> bool:
        if line.detected:
            self._reset_find_line()
            return False
        if self._line_lost_since <= 0.0:
            self._line_lost_since = now
        lost_for_s = now - self._line_lost_since
        if lost_for_s >= self.line_lost_timeout_s:
            send_stop_command(context=f"{context}_line_lost_stop")
            self._last_command_at = now
            self._reset_line_pid()
            self._debug_line_follow(
                now=now,
                stage="scan_timeout_stop",
                context=context,
                payload={
                    "line_detected": False,
                    "lost_for_s": round(lost_for_s, 3),
                    "timeout_s": round(self.line_lost_timeout_s, 3),
                },
            )
            return False
        self._maybe_flip_scan_direction(now)
        turn_sign = 1.0 if self._find_line_phase == self.FOLLOW_LINE_PHASE_SCAN_RIGHT else -1.0
        _send_control_polar(
            angle_rad=turn_sign * 2.8,
            speed=self.find_line_speed,
            context=f"{context}_reacquire",
        )
        self._last_command_at = now
        self._debug_line_follow(
            now=now,
            stage="scan",
            context=context,
            payload={
                "line_detected": False,
                "scan_phase": self._find_line_phase,
                "turn_sign": turn_sign,
                "lost_for_s": round(lost_for_s, 3),
                "scan_switch_s": round(self.scan_switch_interval_s, 3),
            },
        )
        return False

    def _step_follow_line_pid(
        self,
        *,
        line: Vector,
        now: float,
        context: str,
        follow_max_speed: float,
        debug_red_line_x: float = 0.0,
        debug_red_line_detected: bool = False,
        debug_line_error_x: float = 0.0,
        debug_line_error_detected: bool = False,
    ) -> bool:
        dt = now - self._line_pid_last_at if self._line_pid_last_at > 0.0 else self.tick_interval
        dt = clamp(dt, self.line_pid_dt_min, self.line_pid_dt_max)
        self._line_pid_last_at = now

        error = line.x
        if abs(error) < self.line_pid_deadband:
            error = 0.0

        self._line_pid_integral = clamp(
            self._line_pid_integral + error * dt,
            -self.line_pid_i_max,
            self.line_pid_i_max,
        )
        derivative = (error - self._line_pid_prev_error) / dt
        self._line_pid_prev_error = error

        turn_raw = (
            self.line_pid_kp * error
            + self.line_pid_ki * self._line_pid_integral
            + self.line_pid_kd * derivative
        )
        turn = clamp(turn_raw, -self.line_pid_out_max, self.line_pid_out_max)
        angle_rad = clamp(turn, -2.5, 2.5)
        if abs(turn) > 1e-4:
            self._line_last_turn_sign = 1.0 if turn > 0.0 else -1.0

        speed = (
            self.line_pid_base_speed
            - self.line_pid_turn_slowdown * abs(turn)
            - self.line_pid_error_slowdown * abs(error)
        )
        speed = clamp(speed, self.line_pid_min_speed, self.line_pid_max_speed)
        speed = min(speed, follow_max_speed)
        speed = clamp(speed, 0.0, MAX_CONTROL_SPEED)

        _send_control_polar(
            angle_rad=angle_rad,
            speed=speed,
            context=context,
        )
        self._last_command_at = now
        self._debug_line_follow(
            now=now,
            stage="pid",
            context=context,
            payload={
                "line_detected": bool(line.detected),
                "red_line_detected": bool(debug_red_line_detected),
                "line_error_detected": bool(debug_line_error_detected),
                "red_line_x": round(float(debug_red_line_x), 4),
                "line_error_x": round(float(debug_line_error_x), 4),
                "pid_error": round(float(error), 4),
                "turn_raw": round(float(turn_raw), 4),
                "turn": round(float(turn), 4),
                "angle_rad": round(float(angle_rad), 4),
                "speed": round(float(speed), 4),
                "scan_phase": self._find_line_phase,
                "last_turn_sign": self._line_last_turn_sign,
            },
        )
        return True

    def _init_relative_turn_target(
        self,
        current_heading_rad: float,
        delta_rad: float,
        existing_target_rad: float | None,
    ) -> float:
        if existing_target_rad is not None:
            return existing_target_rad
        return wrap_angle(current_heading_rad + delta_rad)

    def _step_turn_to_heading(
        self,
        *,
        current_heading_rad: float,
        target_heading_rad: float,
        tolerance_rad: float,
        turn_speed: float,
        turn_context: str,
        done_context: str,
        turn_angle_rad: float = 2.8,
    ) -> bool:
        heading_error = wrap_angle(target_heading_rad - current_heading_rad)
        if abs(heading_error) <= tolerance_rad:
            send_stop_command(context=done_context)
            return True

        turn_sign = 1.0 if heading_error > 0 else -1.0
        _send_control_polar(
            angle_rad=turn_sign * turn_angle_rad,
            speed=turn_speed,
            context=turn_context,
        )
        return False

    def _sync_line_pid_from_bridge(self, *, now: float, force: bool = False) -> None:
        # Line-follow PID values are now owned by this state-machine instance.
        _ = now
        _ = force

    def get_line_follow_pid_settings(self) -> dict[str, float]:
        return {
            "kp": self.line_pid_kp,
            "ki": self.line_pid_ki,
            "kd": self.line_pid_kd,
            "i_max": self.line_pid_i_max,
            "out_max": self.line_pid_out_max,
            "base_speed": self.line_pid_base_speed,
            "min_speed": self.line_pid_min_speed,
            "max_speed": self.line_pid_max_speed,
            "follow_max_speed": self.line_pid_follow_max_speed,
            "turn_slowdown": self.line_pid_turn_slowdown,
            "error_slowdown": self.line_pid_error_slowdown,
            "deadband": self.line_pid_deadband,
        }

    def apply_line_follow_pid_updates(self, payload: dict) -> tuple[dict[str, float], dict[str, str]]:
        if not isinstance(payload, dict):
            return {}, {"payload": "Payload must be an object."}

        updates: dict[str, float] = {}
        errors: dict[str, str] = {}
        for key, value in payload.items():
            bounds = LINE_FOLLOW_PID_BOUNDS.get(key)
            if not bounds:
                errors[key] = "Unknown setting."
                continue
            try:
                parsed = float(value)
            except (TypeError, ValueError):
                errors[key] = "Value must be a finite number."
                continue
            if not math.isfinite(parsed):
                errors[key] = "Value must be a finite number."
                continue
            low, high = bounds
            if parsed < low or parsed > high:
                errors[key] = f"Value must be between {low} and {high}."
                continue
            updates[key] = parsed

        if errors:
            return {}, errors
        if not updates:
            return {}, {"payload": "No settings to update."}

        preview = self.get_line_follow_pid_settings()
        preview.update(updates)
        if preview["min_speed"] > preview["max_speed"]:
            return {}, {
                "min_speed": "min_speed must be <= max_speed.",
                "max_speed": "max_speed must be >= min_speed.",
            }

        self.line_pid_kp = updates.get("kp", self.line_pid_kp)
        self.line_pid_ki = updates.get("ki", self.line_pid_ki)
        self.line_pid_kd = updates.get("kd", self.line_pid_kd)
        self.line_pid_i_max = abs(updates.get("i_max", self.line_pid_i_max))
        self.line_pid_out_max = abs(updates.get("out_max", self.line_pid_out_max))
        self.line_pid_base_speed = clamp(updates.get("base_speed", self.line_pid_base_speed), 0.0, 5.0)
        self.line_pid_min_speed = clamp(updates.get("min_speed", self.line_pid_min_speed), 0.0, 5.0)
        self.line_pid_max_speed = clamp(updates.get("max_speed", self.line_pid_max_speed), 0.0, 5.0)
        self.line_pid_follow_max_speed = clamp(
            updates.get("follow_max_speed", self.line_pid_follow_max_speed),
            0.0,
            5.0,
        )
        self.line_pid_turn_slowdown = abs(updates.get("turn_slowdown", self.line_pid_turn_slowdown))
        self.line_pid_error_slowdown = abs(updates.get("error_slowdown", self.line_pid_error_slowdown))
        self.line_pid_deadband = abs(updates.get("deadband", self.line_pid_deadband))
        if self.line_pid_min_speed > self.line_pid_max_speed:
            self.line_pid_min_speed, self.line_pid_max_speed = self.line_pid_max_speed, self.line_pid_min_speed
        return updates, {}

    def follow_line(
        self,
        line: Vector,
        *,
        now: float,
        context: str,
        follow_max_speed: float,
        heading_rad: float,
        debug_red_line: Vector | None = None,
        debug_line_error: Vector | None = None,
    ) -> bool:
        red_line = debug_red_line if debug_red_line is not None else line
        line_error = debug_line_error if debug_line_error is not None else line
        match self._find_line_phase:
            case self.FOLLOW_LINE_PHASE_SCAN_LEFT | self.FOLLOW_LINE_PHASE_SCAN_RIGHT:
                return self._step_follow_line_scan(
                    line=line,
                    now=now,
                    context=context,
                )
            case self.FOLLOW_LINE_PHASE_IDLE:
                if not line.detected:
                    self._begin_follow_line_scan(now)
                    return self._step_follow_line_scan(
                        line=line,
                        now=now,
                        context=context,
                    )
                self._reset_find_line()
                return self._step_follow_line_pid(
                    line=line,
                    now=now,
                    context=context,
                    follow_max_speed=follow_max_speed,
                    debug_red_line_x=red_line.x,
                    debug_red_line_detected=red_line.detected,
                    debug_line_error_x=line_error.x,
                    debug_line_error_detected=line_error.detected,
                )
            case _:
                # Recover safely if phase was corrupted.
                self._reset_find_line()
                if not line.detected:
                    self._begin_follow_line_scan(now)
                    return self._step_follow_line_scan(
                        line=line,
                        now=now,
                        context=context,
                    )
                return self._step_follow_line_pid(
                    line=line,
                    now=now,
                    context=context,
                    follow_max_speed=follow_max_speed,
                    debug_red_line_x=red_line.x,
                    debug_red_line_detected=red_line.detected,
                    debug_line_error_x=line_error.x,
                    debug_line_error_detected=line_error.detected,
                )

    def step(self, inputs: Inputs) -> TransitionResult | None:
        now = time.monotonic()
        wall_now = time.time()
        self._sync_line_pid_from_bridge(now=now)
        next_state: State | None = None
        label: str | None = None

        global val
        match self.state:
            case State.SEARCHING_DEMO:
                if inputs.red_line.detected:
                    x_cmd = clamp(inputs.red_line.x, -1.0, 1.0)
                    y_cmd = clamp(inputs.red_line.y, -1.0, 1.0)
                    clamped_magnitude = (x_cmd * x_cmd + y_cmd * y_cmd) ** 0.5
                    if clamped_magnitude <= 1e-6:
                        send_stop_command(context="testing_red_line_passthrough_stop")
                        self._last_command_at = now
                    else:
                        speed = clamped_magnitude
                        _send_control_command(
                            x=x_cmd,
                            y=y_cmd,
                            speed=speed,
                            servo=0, 
                            context="line input",
                        )
                        self._last_command_at = now
                else:
                    
                    
                    send_stop_command(context="line input stop")
                    self._last_command_at = now
                
                if inputs.target.detected:
                    next_state = State.PICKUP
                    label = "PU"

            case State.PICKUP:
                if inputs.red_line.detected:
                    _send_control_command(x=0, y=0, speed=0, servo=90, context="pickup_stop")
                    self._last_command_at = now    
                if not inputs.target.detected:
                    next_state = State.SEARCHING_DEMO
                    label = "PU"
                
            case State.REMOTE_CONTROL:
                if inputs.red_line.detected:
                    x_cmd = clamp(inputs.red_line.x, -1.0, 1.0)
                    y_cmd = clamp(inputs.red_line.y, -1.0, 1.0)
                    magnitude = (x_cmd * x_cmd + y_cmd * y_cmd) ** 0.5
                    if magnitude <= 1e-6:
                        send_stop_command(context="remote_control_red_line_stop")
                    else:
                        _send_control_command(
                            x=x_cmd,
                            y=y_cmd,
                            speed=clamp(magnitude, 0.0, MAX_CONTROL_SPEED),
                            context="remote_control_red_line",
                        )
                    self._last_command_at = now
                    self._last_remote_control_poll_at = now
                else:
                    should_forward = (
                        now - self._last_remote_control_poll_at
                    ) >= self.remote_control_poll_interval
                    if should_forward:
                        command = self._fetch_remote_control_command()
                        if command is not None:
                            _send_control_command(
                                x=command["x"],
                                y=command["y"],
                                speed=command["speed"],
                                context="remote_control_passthrough",
                            )
                            self._last_command_at = now
                        self._last_remote_control_poll_at = now
            case State.SEARCHING:
                self.follow_line(
                    inputs.line_error,
                    now=now,
                    context="searching_follow_line",
                    follow_max_speed=self.line_pid_follow_max_speed,
                    heading_rad=inputs.heading_rad,
                    debug_red_line=inputs.red_line,
                    debug_line_error=inputs.line_error,
                )
                if inputs.lego_detected:
                    next_state = State.FIND_TARGET
                    label = "LD"
            case State.FIND_TARGET:
                if inputs.target_detected:
                    next_state = State.ALIGN_FOR_RETRIEVE
                    label = "TD"
            case State.ALIGN_FOR_RETRIEVE:
                self.follow_line(
                    inputs.line_error,
                    now=now,
                    context="align_retrieve_follow_line",
                    follow_max_speed=self.line_pid_follow_max_speed*0.5,
                    heading_rad=inputs.heading_rad,
                    debug_red_line=inputs.red_line,
                    debug_line_error=inputs.line_error,
                )
                if inputs.aligned_for_retrieve:
                    # send_stop_command()
                    self._last_command_at = now
                    next_state = State.RETRIEVING
                    label = "AFR"
            case State.RETRIEVING:
                # Temporary simulation behavior.
                inputs.pick_up_success = True
                self._last_command_at = now
                if inputs.pick_up_success:
                    self._retrieved_turn_target_heading = None
                    next_state = State.RETRIEVED
                    label = "PUS"
                elif inputs.failed_pickup and self.context.failed_pickups >= self.failed_pickup_limit:
                    next_state = State.ERROR_RETRIEVE
                    label = "FPU>limit"
                elif inputs.failed_pickup:
                    next_state = State.ALIGN_FOR_RETRIEVE
                    label = "FPU"
            case State.RETRIEVED:
                self._retrieved_turn_target_heading = self._init_relative_turn_target(
                    current_heading_rad=inputs.heading_rad,
                    delta_rad=math.pi,
                    existing_target_rad=self._retrieved_turn_target_heading,
                )
                turn_done = self._step_turn_to_heading(
                    current_heading_rad=inputs.heading_rad,
                    target_heading_rad=self._retrieved_turn_target_heading,
                    tolerance_rad=self.retrieved_turn_tol_rad,
                    turn_speed=self.retrieved_turn_speed,
                    turn_context="retrieved_turn_180",
                    done_context="retrieved_turn_done",
                )
                if turn_done:
                    self._retrieved_turn_target_heading = None
                    next_state = State.TRANSPORTING
                    label = "R180"
            case State.TRANSPORTING:
                self.follow_line(
                    inputs.line_error,
                    now=now,
                    context="transporting_follow_line",
                    follow_max_speed=self.line_pid_follow_max_speed,
                    heading_rad=inputs.heading_rad,
                    debug_red_line=inputs.red_line,
                    debug_line_error=inputs.line_error,
                )
                if inputs.safe_zone_detected:
                    send_stop_command()
                    next_state = State.ALIGN_FOR_PLACE
                    label = "SZD"
            case State.ALIGN_FOR_PLACE:
                inputs.aligned_for_place = True
                if inputs.aligned_for_place or inputs.placing:
                    next_state = State.RETURN_HOME
                    label = "AFP"
                elif inputs.failed_pickup:
                    next_state = State.ERROR_PLACE
                    label = "FPU"
            case State.PLACING:
                self.follow_line(
                    inputs.line_error,
                    now=now,
                    context="placing_follow_line",
                    follow_max_speed=self.line_pid_follow_max_speed,
                    heading_rad=inputs.heading_rad,
                    debug_red_line=inputs.red_line,
                    debug_line_error=inputs.line_error,
                )
                #this should be sent from CV but just for now its for testing
                # if inputs.safe_zone.detected and inputs.safe_zone.magnitude < 0.3:
                inputs.place_success = True
                if inputs.place_success:
                    next_state = State.PLACE_SUCCESS
                    label = "PS"
            case State.PLACE_SUCCESS:
                next_state = State.RETURN_HOME
                label = "PS"
            case State.RETURN_HOME:
                self.follow_line(
                    inputs.line_error,
                    now=now,
                    context="return_home_follow_line",
                    follow_max_speed=self.line_pid_follow_max_speed,
                    heading_rad=inputs.heading_rad,
                    debug_red_line=inputs.red_line,
                    debug_line_error=inputs.line_error,
                )
                # if inputs.at_home:
                if inputs.home.detected and inputs.home.magnitude < 0.3:
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
                self._retrieved_turn_target_heading = self._init_relative_turn_target(
                    current_heading_rad=inputs.heading_rad,
                    delta_rad=math.pi,
                    existing_target_rad=self._retrieved_turn_target_heading,
                )
                turn_done = self._step_turn_to_heading(
                    current_heading_rad=inputs.heading_rad,
                    target_heading_rad=self._retrieved_turn_target_heading,
                    tolerance_rad=self.retrieved_turn_tol_rad,
                    turn_speed=self.retrieved_turn_speed,
                    turn_context="retrieved_turn_180",
                    done_context="retrieved_turn_done",
                )
                if turn_done:
                    self._retrieved_turn_target_heading = None
                    self.reset()
                # send_stop_command(context="end")
                # return None

        if next_state is None or label is None:
            return None

        source = self.state
        self.state = next_state
        if source != self.state:
            self._reset_line_pid()
            self._reset_find_line()
        self.context.transition_count += 1
        self.context.last_transition_at = wall_now
        return TransitionResult(source=source, target=next_state, label=label)

    def reset(self) -> None:
        self.force_state(State.SEARCHING)

    def force_state(self, target_state: State) -> None:
        self.state = target_state
        self.context = StateContext()
        self._last_command_at = 0.0
        self._last_remote_control_poll_at = 0.0
        self._retrieved_turn_target_heading = None
        self._reset_line_pid()
        self._reset_find_line()
        
