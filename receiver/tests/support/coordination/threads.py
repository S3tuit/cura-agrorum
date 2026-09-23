"""Named threads with bounded failure propagation for host tests."""

from __future__ import annotations

from collections.abc import Callable, Iterable
from threading import Thread
from types import TracebackType


class CheckedThread(Thread):
    """Capture a worker failure so the owning test can re-raise it."""

    def __init__(self, *, name: str, target: Callable[[], None]) -> None:
        super().__init__(name=name, target=target, daemon=True)
        self._failure: tuple[BaseException, TracebackType | None] | None = None

    def run(self) -> None:
        try:
            super().run()
        except BaseException as error:  # Propagated by join_checked_threads.
            self._failure = (error, error.__traceback__)


def start_checked_threads(
    targets: Iterable[tuple[str, Callable[[], None]]],
) -> tuple[CheckedThread, ...]:
    threads = tuple(CheckedThread(name=name, target=target) for name, target in targets)
    for thread in threads:
        thread.start()
    return threads


def join_checked_threads(
    threads: Iterable[CheckedThread],
    *,
    timeout_seconds: float = 5.0,
) -> None:
    """Join each worker; the timeout is a deadlock detector, not test timing."""

    checked_threads = tuple(threads)
    for thread in checked_threads:
        thread.join(timeout_seconds)
    alive = tuple(thread.name for thread in checked_threads if thread.is_alive())
    if alive:
        raise AssertionError(f"threads did not finish: {', '.join(alive)}")
    for thread in checked_threads:
        if thread._failure is not None:
            error, traceback = thread._failure
            raise error.with_traceback(traceback)


__all__ = [
    "CheckedThread",
    "start_checked_threads",
    "join_checked_threads",
]
