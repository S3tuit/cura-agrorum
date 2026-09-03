"""Manually advanced fake for the receiver's operating-system clocks."""

from __future__ import annotations

from threading import Lock


_UINT64_MAX = (1 << 64) - 1
_INT64_MIN = -(1 << 63)
_INT64_MAX = (1 << 63) - 1


def _require_exact_int(value: int, *, name: str) -> int:
    if type(value) is not int:
        raise TypeError(f"{name} must be an integer")
    return value


def _require_range(value: int, *, name: str, minimum: int, maximum: int) -> int:
    if not minimum <= value <= maximum:
        raise OverflowError(f"{name} is outside the clock scalar range")
    return value


class FakeOsClock:
    """Expose fixed clock reads changed only by explicit test operations."""

    def __init__(self, *, monotonic_us: int = 0, realtime_us: int = 0) -> None:
        monotonic_us = _require_exact_int(monotonic_us, name="monotonic_us")
        realtime_us = _require_exact_int(realtime_us, name="realtime_us")
        self._monotonic_us = _require_range(
            monotonic_us,
            name="monotonic_us",
            minimum=0,
            maximum=_UINT64_MAX,
        )
        self._realtime_us = _require_range(
            realtime_us,
            name="realtime_us",
            minimum=_INT64_MIN,
            maximum=_INT64_MAX,
        )
        self._lock = Lock()

    def now_monotonic_us(self) -> int:
        """Return the current fake monotonic value without advancing it."""

        with self._lock:
            return self._monotonic_us

    def now_realtime_us(self) -> int:
        """Return the current fake realtime value without advancing it."""

        with self._lock:
            return self._realtime_us

    def advance_elapsed_us(self, delta_us: int) -> None:
        """Advance monotonic and realtime together by non-negative elapsed time."""

        delta_us = _require_exact_int(delta_us, name="delta_us")
        if delta_us < 0:
            raise ValueError("delta_us must not move monotonic time backward")

        with self._lock:
            monotonic_us = _require_range(
                self._monotonic_us + delta_us,
                name="monotonic_us",
                minimum=0,
                maximum=_UINT64_MAX,
            )
            realtime_us = _require_range(
                self._realtime_us + delta_us,
                name="realtime_us",
                minimum=_INT64_MIN,
                maximum=_INT64_MAX,
            )
            self._monotonic_us = monotonic_us
            self._realtime_us = realtime_us

    def step_realtime_us(self, delta_us: int) -> None:
        """Step realtime in either direction without changing monotonic time."""

        delta_us = _require_exact_int(delta_us, name="delta_us")
        with self._lock:
            self._realtime_us = _require_range(
                self._realtime_us + delta_us,
                name="realtime_us",
                minimum=_INT64_MIN,
                maximum=_INT64_MAX,
            )
