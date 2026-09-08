"""Private lock-protected execution boundary for synchronous persistence commands."""

from dataclasses import dataclass
from enum import Enum, auto
from threading import Event, Lock

from .ports.clocks import MonotonicClock
from .sqlite_database import SQLITE_BUSY_TIMEOUT_MS


class ControlCommandKind(Enum):
    CONFIGURATION = auto()
    LOAD_STATE = auto()
    COMMIT_STATE = auto()
    CLEAN_STOP = auto()


class ControlExecutionState(Enum):
    QUEUED = auto()
    RUNNING_PRECOMMIT = auto()
    COMMIT_MAY_HAVE_RUN = auto()
    DONE = auto()
    CANCELLED = auto()


@dataclass(frozen=True, slots=True)
class ControlRequest:
    kind: ControlCommandKind
    deadline_monotonic_us: int
    payload: object = None


class ControlDeadlineExceeded(Exception):
    """The command cannot perform another precommit operation."""


class ControlCommand:
    """One immutable request, private completion event and atomic effect boundary.

    This object is private to the channel/worker. Neither owner exposes its
    locks, mutable execution state or completion event to application callers.
    """

    def __init__(self, request: ControlRequest, clock: MonotonicClock):
        self.request = request
        self.clock = clock
        self.completion = Event()
        self.lock = Lock()
        self.state = ControlExecutionState.QUEUED
        self.cancel_requested = False
        self.result = None

    def _check_locked(self):
        if (
            self.cancel_requested
            or self.state is ControlExecutionState.CANCELLED
            or self.clock.now_monotonic_us() >= self.request.deadline_monotonic_us
        ):
            raise ControlDeadlineExceeded()

    def start(self):
        with self.lock:
            self._check_locked()
            if self.state is ControlExecutionState.RUNNING_PRECOMMIT:
                return  # Worker reserved dispatch under the shared scheduler lock.
            if self.state is not ControlExecutionState.QUEUED:
                raise RuntimeError("command has already started")
            self.state = ControlExecutionState.RUNNING_PRECOMMIT

    def check(self, connection=None):
        with self.lock:
            self._check_locked()
            remaining = (
                self.request.deadline_monotonic_us - self.clock.now_monotonic_us()
            )
        if connection is not None:
            # Floor milliseconds so SQLite lock waits never exceed remaining time.
            timeout = max(0, min(SQLITE_BUSY_TIMEOUT_MS, remaining // 1000))
            connection.execute(f"PRAGMA busy_timeout = {timeout}")

    def before_commit(self, connection):
        self.check(connection)
        with self.lock:
            self._check_locked()
            if self.state is not ControlExecutionState.RUNNING_PRECOMMIT:
                raise RuntimeError("invalid command commit boundary")
            self.state = ControlExecutionState.COMMIT_MAY_HAVE_RUN

    def expire(self):
        """Caller timeout atomically cancels only a definitely effect-free command."""
        with self.lock:
            if self.state is ControlExecutionState.QUEUED:
                self.state = ControlExecutionState.CANCELLED
            elif self.state is ControlExecutionState.RUNNING_PRECOMMIT:
                self.cancel_requested = True
            return self.state, self.result

    def finish(self, result):
        """Only the persistence owner installs a result and signals completion."""
        with self.lock:
            self.result = result
            self.state = ControlExecutionState.DONE
            self.completion.set()
