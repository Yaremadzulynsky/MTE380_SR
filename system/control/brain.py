"""
RobotBrain — central robot controller.

One background thread runs one loop.  Public methods like go(), start_drive(),
and run_rotation() change _mode; the loop picks up the new mode on the next
tick and runs the appropriate logic.  No per-operation threads are spawned.

Loop modes
──────────
  idle      — motors off, loop sleeps at IDLE_HZ
  mission   — perception → state machine → speed PID; camera read paces the tick
  drive     — continuous speed PID at fixed fractions; runs at DRIVE_HZ
  rotation  — RotationController until done; runs at ROTATION_HZ

    brain = RobotBrain("/dev/ttyACM0", 115200)
    brain.start_loop()
    brain.go()         # mode → mission
    brain.stop()       # mode → idle
    brain.shutdown()   # quit loop, close serial
"""
from __future__ import annotations

import collections
import threading
import time

import config as _config_module
from bridge import SerialBridge


# ── Hardware constants ────────────────────────────────────────────────────────

TICKS_PER_REV = 700
MAX_RPM       = 320

_LOOP_HZ = 200


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ── Brain ─────────────────────────────────────────────────────────────────────

class RobotBrain:
    """Central robot controller — hardware, PIDs, perception, and state machine."""

    def __init__(
        self,
        serial_port: str,
        baud:        int,
        dry_run:     bool = False,
        cfg:         _config_module.Config | None = None,
    ) -> None:
        self.serial_port = serial_port
        self.baud        = baud
        self.dry_run     = dry_run

        c = cfg if cfg is not None else _config_module.get()

        self._lock = threading.Lock()
        self._quit = threading.Event()

        # ── Loop mode ─────────────────────────────────────────────────────────
        self._mode: str = "idle"   # "idle" | "mission" | "drive" | "rotation" | "position"

        # ── Encoder / RPM state ───────────────────────────────────────────────
        self._left_ticks  = 0
        self._right_ticks = 0
        self._prev_left   = 0
        self._prev_right  = 0
        self._enc_time    = 0.0
        self._rpm_left    = 0.0
        self._rpm_right   = 0.0

        # ── Mission state ─────────────────────────────────────────────────────
        self._sm     = None
        self._period = 1.0 / max(c.fps, 1.0)   # camera tick interval (seconds)

        # ── Motion control ────────────────────────────────────────────────────
        self._active_ctrl:        object | None = None   # RotationController
        self._active_ctrl_type:   str    | None = None
        self._rotation_degrees:   float         = 0.0
        self._last_move_result:   dict   | None = None  # final state after done

        # ── Telemetry ─────────────────────────────────────────────────────────
        self._last_det    = None
        self._last_output = None
        self._telem_history: collections.deque = collections.deque(maxlen=300)
        self._frame_n    = 0
        self._t_start    = time.monotonic()
        self._last_idle_telem = 0.0
        self._cam_fps     = 0.0
        self._last_frame_t = 0.0
        self._manual_claw_until = 0.0  # suppress heartbeat claw until this monotonic time

        # ── Loop display / telemetry settings (populated by start_loop) ───────
        self._no_display    = True
        self._no_telemetry  = False
        self._telem_every   = 1
        self._telem_idle_s  = 0.5
        self._last_idle_cam = 0.0

        # ── Perception (lazy — created in start_loop) ─────────────────────────
        self._perception = None

        # ── Speed controller (lazy import avoids circular dependency) ─────────
        from control.speed_controller import SpeedController
        self._speed_ctrl = SpeedController(self, c)

        # ── Serial bridge ─────────────────────────────────────────────────────
        self.bridge: SerialBridge | None = None
        if not dry_run:
            self.bridge = SerialBridge(serial_port, baud)
            self.bridge.on_encoders  = self._on_encoders
            self.bridge.on_heartbeat = self._on_heartbeat
            self.bridge.start()

        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="brain-loop"
        )

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start_loop(
        self,
        no_display:       bool  = True,
        no_telemetry:     bool  = False,
        telemetry_every:  int   = 1,
        telemetry_idle_s: float = 0.5,
    ) -> None:
        """Open the camera and start the background loop."""
        from perception import Perception
        self._perception   = Perception(_config_module.get())
        self._no_display   = no_display
        self._no_telemetry = no_telemetry
        self._telem_every  = max(1, telemetry_every)
        self._telem_idle_s = telemetry_idle_s
        self._thread.start()

    def go(self, initial_state: str | None = None) -> None:
        """Reload config.yaml and start the mission (mode → mission).

        initial_state: optional State enum value name, e.g. "DRIVE_FORWARD".
                       Defaults to LINE_FOLLOW.
        """
        fresh = _config_module.reload()
        self._period = 1.0 / max(fresh.fps, 1.0)
        self._speed_ctrl.set_gains(fresh)
        if self._perception is not None:
            self._perception.reconfigure(fresh)

        from state_machine import MissionStateMachine
        from states import State
        sm = MissionStateMachine(fresh, brain=self)
        if initial_state is not None:
            try:
                sm.state = State(initial_state)
            except ValueError:
                print(f"[go] unknown initial_state {initial_state!r} — using LINE_FOLLOW", flush=True)
        with self._lock:
            self._sm   = sm
            self._mode = "mission"
        self.send_claw(fresh.claw_open)
        print(f"[go] claw → open ({fresh.claw_open}°)", flush=True)

        print(
            f"[go] mission started — initial={sm.state.value}  "
            f"steer_kp={fresh.steer_kp}  max_speed={fresh.max_speed}  "
            f"motor_kp={fresh.motor_kp:.6f}",
            flush=True,
        )

    def stop(self) -> None:
        """Idle motors and return to idle mode."""
        with self._lock:
            self._mode = "idle"
        self._idle()
        print("[stop] motors idle, serial open", flush=True)

    def shutdown(self) -> None:
        """Stop everything and close serial — call once on process exit."""
        self._quit.set()
        with self._lock:
            self._mode = "idle"
        self._idle()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._perception is not None:
            self._perception.release()
        self._close_serial()

    def reconfigure(self, cfg) -> None:
        """Apply updated config to all live components without restarting."""
        self._speed_ctrl.set_gains(cfg)
        if self._perception is not None:
            self._perception.reconfigure(cfg)
        with self._lock:
            if self._sm is not None:
                self._sm.cfg = cfg
                self._sm._steer_pid.tunings = (cfg.steer_kp, cfg.steer_ki, cfg.steer_kd)
                self._sm._steer_pid.output_limits = (-cfg.steer_out_limit, cfg.steer_out_limit)
                self._sm._heading_pid.tunings = (cfg.heading_kp, cfg.heading_ki, cfg.heading_kd)
                self._sm._heading_pid.output_limits = (-cfg.steer_out_limit, cfg.steer_out_limit)

    # ── Motion commands ───────────────────────────────────────────────────────

    def start_drive(self, left: float, right: float) -> None:
        """Drive at fixed speed fractions via RPM PID (mode → drive)."""
        self._speed_ctrl.reset()
        self._speed_ctrl.set_target(left, right)
        with self._lock:
            self._mode = "drive"

    def stop_drive(self) -> None:
        """Stop driving and return to idle."""
        with self._lock:
            self._mode = "idle"
        self._idle()

    def run_position(self, meters: float) -> None:
        """Drive forward/backward by metres in the main loop (mode → position)."""
        from control.position_controller import PositionController
        self._idle()
        ctrl = PositionController(self, meters)
        self._last_move_result = None
        with self._lock:
            self._active_ctrl      = ctrl
            self._active_ctrl_type = "position"
            self._mode             = "position"

    def run_rotation(self, degrees: float) -> None:
        """Rotate by degrees in the main loop (mode → rotation)."""
        from control.rotation_controller import RotationController
        self._idle()
        ctrl = RotationController(self, degrees)
        self._last_move_result = None
        with self._lock:
            self._active_ctrl      = ctrl
            self._active_ctrl_type = "rotation"
            self._rotation_degrees = degrees
            self._mode             = "rotation"

    def stop_move(self) -> None:
        """Cancel the active rotation and return to idle."""
        with self._lock:
            self._mode             = "idle"
            self._active_ctrl      = None
            self._active_ctrl_type = None
            self._last_move_result = None
        self._idle()

    def move_status(self) -> dict:
        """Progress of the currently running rotation, or the last completed result."""
        with self._lock:
            ctrl = self._active_ctrl
            typ  = self._active_ctrl_type
        if ctrl is None:
            return self._last_move_result or {"active": None}
        traveled, target = ctrl.progress()
        if hasattr(ctrl, "error_deg"):
            error = ctrl.error_deg()
        elif hasattr(ctrl, "error_m"):
            error = ctrl.error_m()
        else:
            error = target - traveled
        enc_l, enc_r = self.encoder_ticks
        return {
            "active":   typ,
            "done":     ctrl.done,
            "traveled": round(traveled, 1),
            "target":   round(target,   1),
            "error":    round(error,    1),
            "enc_l":    enc_l,
            "enc_r":    enc_r,
        }

    # ── Direct hardware commands ──────────────────────────────────────────────

    def drive(self, left: float, right: float) -> None:
        """One speed-PID tick at the given fraction targets [-1, 1]."""
        self._speed_ctrl.set_target(left, right)
        self._speed_ctrl.step()

    def send_voltage(self, left: float, right: float) -> None:
        """Send motor voltages directly, bypassing the RPM PID. Range [-1, 1]."""
        left  = _clamp(left,  -1.0, 1.0)
        right = _clamp(right, -1.0, 1.0)
        if self.dry_run:
            print(f"[dry-run]  send_voltage  L={left:+.3f}  R={right:+.3f}", flush=True)
            return
        assert self.bridge is not None
        self.bridge.send_drive(left, right)

    def send_claw(self, angle: float, manual: bool = False) -> None:
        if manual:
            self._manual_claw_until = time.monotonic() + 5.0
        if self.dry_run:
            print(f"[dry-run]  claw  angle={angle:.1f}°", flush=True)
            return
        assert self.bridge is not None
        self.bridge.send_claw(angle)

    def idle(self) -> None:
        """Zero motor voltages; keep serial open. Does not reset PID integrators."""
        if self.dry_run:
            return
        if self.bridge is not None:
            try:
                self.bridge.send_drive(0.0, 0.0)
            except Exception:
                pass

    # ── Gain helpers ──────────────────────────────────────────────────────────

    def set_steer_gains(self, kp: float, ki: float, kd: float) -> None:
        cfg = _config_module.get()
        cfg.steer_kp = kp; cfg.steer_ki = ki; cfg.steer_kd = kd
        _config_module.save(cfg)
        with self._lock:
            if self._sm is not None:
                self._sm.cfg.steer_kp = kp
                self._sm.cfg.steer_ki = ki
                self._sm.cfg.steer_kd = kd
                self._sm._steer_pid.tunings = (kp, ki, kd)
                self._sm._steer_pid.reset()

    def set_motor_gains(self, kp: float, ki: float, kd: float) -> None:
        cfg = _config_module.get()
        cfg.motor_kp = kp; cfg.motor_ki = ki; cfg.motor_kd = kd
        _config_module.save(cfg)
        self._speed_ctrl.set_gains(cfg)

    # ── Telemetry / state ─────────────────────────────────────────────────────

    @property
    def state(self) -> str:
        with self._lock:
            return self._sm.state.value if self._sm else "not started"

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._mode == "mission"

    @property
    def encoder_ticks(self) -> tuple[int, int]:
        with self._lock:
            return self._left_ticks, self._right_ticks

    @property
    def measured_rpm(self) -> tuple[float, float]:
        with self._lock:
            return self._rpm_left, self._rpm_right

    def get_annotated_frame(self):
        if self._perception is None:
            return None
        frame = self._perception.get_frame()
        if frame is None:
            return None
        with self._lock:
            det = self._last_det
        return self._perception.draw_debug(frame, det) if det is not None else frame

    def get_mask_frame(self):
        return self._perception.get_mask_frame() if self._perception else None

    def set_mask_channel(self, channel: str) -> None:
        """Switch which HSV mask the /stream/mask endpoint shows ('red'|'green'|'blue')."""
        if self._perception is not None:
            self._perception.mask_channel = channel

    def telemetry_history(self) -> dict:
        rows = list(self._telem_history)
        if not rows:
            return {"t": [], "red_error": [], "rpm_l": [], "rpm_r": []}
        t, err, rpm_l, rpm_r = zip(*rows)
        return {"t": list(t), "red_error": list(err),
                "rpm_l": list(rpm_l), "rpm_r": list(rpm_r)}

    def snapshot(self) -> dict:
        with self._lock:
            det      = self._last_det
            output   = self._last_output
            sm_state = self._sm.state.value if self._sm else "N/A"
            running  = self._mode == "mission"
            lateral_turn = self._sm._heading_lateral_turn if self._sm else 0.0
            heading_turn = self._sm._heading_heading_turn if self._sm else 0.0
        enc_l, enc_r = self.encoder_ticks
        rpm_l, rpm_r = self.measured_rpm
        return {
            "sm_state":    sm_state,
            "running":     running,
            "enc_l":       enc_l,  "enc_r": enc_r,
            "rpm_l":       rpm_l,  "rpm_r": rpm_r,
            "det":         det,
            "output":      output,
            "motor_telem": self._speed_ctrl.telemetry_line(),
            "lateral_turn": lateral_turn,
            "heading_turn": heading_turn,
            "cam_fps":     round(self._cam_fps, 1),
        }

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self) -> None:
        """Single background loop — dispatches on _mode each tick at _LOOP_HZ."""
        period = 1.0 / _LOOP_HZ

        while not self._quit.is_set():
            t0 = time.monotonic()

            with self._lock:
                mode = self._mode

            if mode == "mission":
                self._mission_tick()

            elif mode == "drive":
                self._speed_ctrl.step()

            elif mode == "rotation":
                with self._lock:
                    ctrl = self._active_ctrl
                if ctrl is not None:
                    ctrl.step()
                    if ctrl.done:
                        traveled, target = ctrl.progress()
                        err = ctrl.error_deg()
                        self._last_move_result = {
                            "active":   "rotation",
                            "done":     True,
                            "traveled": round(traveled, 1),
                            "target":   round(target,   1),
                            "error":    round(err,      1),
                        }
                        with self._lock:
                            self._mode             = "idle"
                            self._active_ctrl      = None
                            self._active_ctrl_type = None
                        self._idle()
                        print(
                            f"[rot-move] done  "
                            f"traveled={traveled:.1f}°  target={target:.1f}°  error={err:.1f}°",
                            flush=True,
                        )

            elif mode == "position":
                with self._lock:
                    ctrl = self._active_ctrl
                if ctrl is not None:
                    ctrl.step()
                    if ctrl.done:
                        traveled, target = ctrl.progress()
                        err = ctrl.error_m()
                        self._last_move_result = {
                            "active":   "position",
                            "done":     True,
                            "traveled": round(traveled, 3),
                            "target":   round(target,   3),
                            "error":    round(err,      3),
                        }
                        with self._lock:
                            self._mode             = "idle"
                            self._active_ctrl      = None
                            self._active_ctrl_type = None
                        self._idle()
                        print(
                            f"[pos-move] done  "
                            f"traveled={traveled:.3f}m  target={target:.3f}m  error={err:.3f}m",
                            flush=True,
                        )

            else:
                # idle: keep camera ticking so web streams have a live frame
                now = time.monotonic()
                if now - self._last_idle_cam >= self._period:
                    self._last_idle_cam = now
                    self._idle_camera_tick()

            time.sleep(max(0.0, period - (time.monotonic() - t0)))

        if not self._no_display:
            import cv2 as _cv2
            _cv2.destroyAllWindows()

    def _mission_tick(self) -> None:
        """One camera + state machine step, called from the main loop."""
        from state_machine import State

        frame = self._perception.read_frame()
        if frame is None:
            print("Camera read failed — stopping mission.", flush=True)
            with self._lock:
                self._mode = "idle"
            return

        self._update_cam_fps()
        det          = self._perception.detect(frame)
        enc_l, enc_r = self.encoder_ticks

        with self._lock:
            sm = self._sm

        if sm is None:
            return

        output = sm.step(det, enc_l, enc_r)

        if not output.skip:
            if output.direct_voltage:
                self.send_voltage(output.left, output.right)
            else:
                self._speed_ctrl.set_target(output.left, output.right)
                self._speed_ctrl.step()
        if output.claw is not None:
            self.send_claw(output.claw)

        if output.state == State.DONE:
            with self._lock:
                self._mode = "idle"
            self._idle()
            print("Mission complete. Type 'start' to run again.", flush=True)

        with self._lock:
            self._last_det    = det
            self._last_output = output

        rpm_l, rpm_r = self.measured_rpm
        self._frame_n += 1
        self._telem_history.append((
            time.monotonic() - self._t_start,
            det.red_error if det else 0.0,
            rpm_l, rpm_r,
        ))

        self._print_mission_telem(det, output)

        if not self._no_display:
            import cv2
            overlay = self._perception.draw_debug(frame, det)
            if output:
                label = f"{output.state.value}  L={output.left:+.2f}  R={output.right:+.2f}"
                cv2.putText(overlay, label, (10, overlay.shape[0] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("mission", overlay)
            cv2.waitKey(1)

    def _print_mission_telem(self, det, output) -> None:
        if self._no_telemetry:
            return
        t_run = time.monotonic() - self._t_start
        if output is not None and (self._frame_n % self._telem_every == 0):
            enc_l, enc_r = self.encoder_ticks
            claw_s = f"{output.claw:.0f}°" if output.claw is not None else "--"
            print(
                f"{t_run:8.3f}s  {output.state.value:12s}  "
                f"red={'Y' if det.red_found else 'N'} err={det.red_error:+.3f}  "
                f"blue={'Y' if det.blue_found else 'N'} green={'Y' if det.green_found else 'N'}  "
                f"claw={claw_s}  enc_L={enc_l:6d} enc_R={enc_r:6d}  "
                f"{self._speed_ctrl.telemetry_line()}",
                flush=True,
            )
        elif output is None and (time.monotonic() - self._last_idle_telem >= self._telem_idle_s):
            self._last_idle_telem = time.monotonic()
            print(
                f"{t_run:8.3f}s  IDLE  "
                f"red={'Y' if det.red_found else 'N'} err={det.red_error:+.3f}  "
                f"blue={'Y' if det.blue_found else 'N'} green={'Y' if det.green_found else 'N'}",
                flush=True,
            )

    # ── Internal ──────────────────────────────────────────────────────────────

    def _update_cam_fps(self) -> None:
        """EMA update of camera FPS — call once per frame read."""
        now = time.monotonic()
        if self._last_frame_t > 0.0:
            dt = now - self._last_frame_t
            if dt > 0.0:
                instant = 1.0 / dt
                self._cam_fps = 0.1 * instant + 0.9 * self._cam_fps if self._cam_fps > 0.0 else instant
        self._last_frame_t = now

    def _idle_camera_tick(self) -> None:
        """Read one frame and run detection so web streams stay live outside mission mode."""
        if self._perception is None:
            return
        frame = self._perception.read_frame()
        if frame is not None:
            self._update_cam_fps()
            det = self._perception.detect(frame)
            with self._lock:
                self._last_det = det

    def _idle(self) -> None:
        """Reset speed PID integrators and zero motor voltages."""
        self._speed_ctrl.reset()
        self.idle()

    def _close_serial(self) -> None:
        self.idle()
        if self.dry_run:
            print("[dry-run]  shutdown", flush=True)
            return
        if self.bridge is not None:
            self.bridge.stop()
            self.bridge = None

    def _on_heartbeat(self) -> None:
        """Send claw-open on every heartbeat so it reliably opens after Arduino reset.
        Suppressed while a mission is running (state machine owns the claw) and for
        5 s after a manual web UI command."""
        if time.monotonic() < self._manual_claw_until:
            return
        with self._lock:
            mode = self._mode
        if mode == "mission":
            return
        self.send_claw(_config_module.get().claw_open)

    def _on_encoders(self, left: int, right: int) -> None:
        now = time.monotonic()
        with self._lock:
            dt = now - self._enc_time
            if self._enc_time > 0.0 and dt >= 0.005:
                dl = left  - self._prev_left
                dr = right - self._prev_right
                self._rpm_left  = (dl / TICKS_PER_REV) * 60.0 / dt
                self._rpm_right = (dr / TICKS_PER_REV) * 60.0 / dt
            self._left_ticks  = left
            self._right_ticks = right
            self._prev_left   = left
            self._prev_right  = right
            self._enc_time    = now
