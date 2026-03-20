"""
state_machine/machine.py

Drives the robot by ticking the active State at a fixed rate.
The StateMachine is the single entry point for the robot brain — hardware
and vision are accessed through it, not imported directly by outside code.
"""

from __future__ import annotations
import logging
import threading
import time
from typing import Optional, TYPE_CHECKING

from state_machine.state    import State
from state_machine.odometry import Odometry

if TYPE_CHECKING:
    from state_machine.hardware.robot       import Robot
    from state_machine.vision.line_detector import LineDetector

log = logging.getLogger(__name__)

TICK_RATE_HZ = 20.0


class StateMachine:
    """
    Runs a fixed-rate tick loop, delegating each tick to the active State.

    The StateMachine owns the Robot and LineDetector instances and exposes
    them as properties so callers (e.g. WebServer) never need to import from
    the hardware or vision packages directly.

    Usage
    -----
        sm = StateMachine(robot, detector)
        sm.register(Idle())
        sm.register(LineFollow())
        sm.start('idle')
        ...
        sm.transition('line_follow')
        ...
        sm.stop()
    """

    def __init__(self, robot: Optional['Robot'] = None,
                 detector: Optional['LineDetector'] = None):
        self._robot    = robot
        self._detector = detector
        self._odometry = Odometry()
        self._states:  dict[str, State] = {}
        self._current: Optional[State]  = None
        self._pending: Optional[str]    = None
        self._lock     = threading.Lock()
        self._running  = False
        self._thread:  Optional[threading.Thread] = None

    # ── Public accessors ──────────────────────────────────────────────────────

    @property
    def robot(self) -> Optional['Robot']:
        """The hardware robot instance (None if running without hardware)."""
        return self._robot

    @property
    def detector(self) -> Optional['LineDetector']:
        """The vision line detector instance (None if running without camera)."""
        return self._detector

    @property
    def odometry(self) -> Odometry:
        """Live pose estimate (x_m, y_m, heading_rad) from encoder dead-reckoning."""
        return self._odometry

    @property
    def current_state(self) -> Optional[str]:
        """Name of the currently active state."""
        return self._current.name if self._current else None

    # ── Registration ──────────────────────────────────────────────────────────

    def register(self, state: State) -> 'StateMachine':
        """Register a state. Returns self for chaining."""
        if not state.name:
            raise ValueError(f'{type(state).__name__} has no name set')
        self._states[state.name] = state
        return self

    # ── Control ───────────────────────────────────────────────────────────────

    def start(self, initial_state: str = 'idle') -> None:
        """Start the tick loop in a background thread."""
        if initial_state not in self._states:
            raise ValueError(f'Unknown state: {initial_state!r}')
        self._odometry.reset()
        self._transition(initial_state)
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name='state_machine'
        )
        self._thread.start()
        log.info('StateMachine started in state %r', initial_state)

    def stop(self) -> None:
        """Stop the tick loop and enter idle if registered."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if 'idle' in self._states and (
            self._current is None or self._current.name != 'idle'
        ):
            self._transition('idle')
        log.info('StateMachine stopped')

    def transition(self, state_name: str) -> None:
        """Externally request a state transition (thread-safe)."""
        with self._lock:
            self._pending = state_name

    # ── Internal ──────────────────────────────────────────────────────────────

    def _transition(self, name: str) -> None:
        if name not in self._states:
            log.error('Transition to unknown state %r — ignoring', name)
            return
        if self._current is not None:
            self._current.exit(self._robot, self._detector, self._odometry)
        self._current = self._states[name]
        self._current.enter(self._robot, self._detector, self._odometry)
        log.info('State → %s', name)

    def _loop(self) -> None:
        interval = 1.0 / TICK_RATE_HZ
        while self._running:
            t0 = time.monotonic()

            # Apply any externally requested transition
            with self._lock:
                pending = self._pending
                self._pending = None
            if pending:
                self._transition(pending)

            # Update odometry from latest encoder counts
            if self._robot is not None:
                left, right = self._robot.get_encoders()
                self._odometry.update(left, right)

            # Tick the active state
            if self._current is not None:
                next_state = self._current.tick(self._robot, self._detector, self._odometry)
                if next_state is not None:
                    self._transition(next_state)

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))
