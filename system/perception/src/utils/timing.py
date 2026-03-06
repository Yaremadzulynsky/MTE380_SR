"""Deterministic loop timing helpers."""

from __future__ import annotations

import time


class LoopRegulator:
    """Regulates a loop near target_hz by sleeping until next deadline."""

    def __init__(self, target_hz: float) -> None:
        self.period = 1.0 / float(target_hz)
        self.next_tick = time.perf_counter()

    def sleep(self) -> None:
        """Sleep until the next tick. If late, resync without drifting."""
        self.next_tick += self.period
        now = time.perf_counter()
        delay = self.next_tick - now
        if delay > 0:
            time.sleep(delay)
            return
        # We are late; jump to now so stale deadlines do not accumulate.
        self.next_tick = now
