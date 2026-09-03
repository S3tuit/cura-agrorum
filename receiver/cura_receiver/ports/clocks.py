"""Clock capabilities used by receiver components."""

from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class MonotonicClock(Protocol):
    """Read Linux-boot-scoped monotonic time in integer microseconds."""

    def now_monotonic_us(self) -> int:
        """Return a value that may repeat but never moves backward."""

        ...


@runtime_checkable
class RealtimeClock(Protocol):
    """Read POSIX UTC realtime in signed integer microseconds."""

    def now_realtime_us(self) -> int:
        """Return realtime, which may move forward or backward when stepped."""

        ...
