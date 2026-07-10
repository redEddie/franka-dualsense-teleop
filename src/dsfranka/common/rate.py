"""Simple fixed-rate loop helper."""
import time


class Rate:
    def __init__(self, hz: float):
        self.dt = 1.0 / hz
        self._next = time.perf_counter() + self.dt

    def sleep(self) -> float:
        """Sleep until the next tick. Returns lateness in seconds (>=0 means on time budget)."""
        now = time.perf_counter()
        late = now - self._next
        if late < 0:
            time.sleep(-late)
        # if we fell far behind, resync instead of bursting
        self._next = max(self._next + self.dt, time.perf_counter() - self.dt)
        return max(late, 0.0)
