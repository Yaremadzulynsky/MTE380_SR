"""
state_machine/machine.py

Drives the robot by ticking the active State at a fixed rate.
"""

from __future__ import annotations
import logging
import threading
import time
from typing import Optional, TYPE_CHECKING

from state_machine.state import State

if TYPE_CHECKING:
    from hardware.robot       import Robot
    from vision.line_detector import LineDetector

log = logging.getLogger(__name__)

TICK_RATE_HZ = 20.0


class StateMachine:
    """
    Runs a fixed-rate tick loop, delegating each tick to the active State.

    Usage
    -----
        sm = StateMachine(robot, detector)
        sm.register(Stopped())
        sm.register(LineFollow())
        sm.start('stopped')
        ...
        sm.transition('line_follow')
        ...
        sm.stop()
    """

    def __init__(self, robot: 'Robot', detector: Optional['LineDetector'] = None):
        self._robot    = robot
        self._detector = detector
        self._states:  dict[str, State] = {}
        self._current: Optional[State]  = None
        self._pending: Optional[str]    = None
        self._lock     = threading.Lock()
        self._running  = False
        self._thread:  Optional[threading.Thread] = None

    # ── Registration ──────────────────────────────────────────────────────────

    def register(self, state: State) -> 'StateMachine':
        """Register a state. Returns self for chaining."""
        if not state.name:
            raise ValueError(f'{type(state).__name__} has no name set')
        self._states[state.name] = state
        return self

    # ── Control ───────────────────────────────────────────────────────────────

    def start(self, initial_state: str) -> None:
        """Start the tick loop in a background thread."""
        if initial_state not in self._states:
            raise ValueError(f'Unknown state: {initial_state!r}')
        self._transition(initial_state)
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name='state_machine'
        )
        self._thread.start()
        log.info('StateMachine started in state %r', initial_state)

    def stop(self) -> None:
        """Stop the tick loop and enter STOPPED if registered."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if 'stopped' in self._states and (
            self._current is None or self._current.name != 'stopped'
        ):
            self._transition('stopped')
        log.info('StateMachine stopped')

    def transition(self, state_name: str) -> None:
        """Externally request a state transition (thread-safe)."""
        with self._lock:
            self._pending = state_name

    @property
    def current_state(self) -> Optional[str]:
        """Name of the currently active state."""
        return self._current.name if self._current else None

    # ── Internal ──────────────────────────────────────────────────────────────

    def _transition(self, name: str) -> None:
        if name not in self._states:
            log.error('Transition to unknown state %r — ignoring', name)
            return
        if self._current is not None:
            self._current.exit(self._robot, self._detector)
        self._current = self._states[name]
        self._current.enter(self._robot, self._detector)
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

            # Tick the active state
            if self._current is not None:
                next_state = self._current.tick(self._robot, self._detector)
                if next_state is not None:
                    self._transition(next_state)

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))
