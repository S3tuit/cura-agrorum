"""Shared, caller-driven storage recovery on the sole persistence thread.

This component owns no FIFO work or control commands. Its caller supplies the
retained-work predicate before checkpointing or reopening ordinary admission.
"""

from __future__ import annotations

import sqlite3
from dataclasses import dataclass, replace

from .elapsed_duration import checked_monotonic_deadline, minimum_wait_monotonic_us
from .generated.receiver_enums_generated import PersistenceAdmissionState as State
from .persist_queue import PersistenceAdmissionSnapshot
from .persistence_failures import classify_global_failure
from .reading_persistence import PersistenceIdentityCollision
from .sqlite_database import DatabaseFailure, StorageUnavailable
from .sqlite_repository import SqliteRepository

_INT64_MAX = (1 << 63) - 1
_OPERATOR_STATES = (State.UNAVAILABLE_CORRUPT, State.UNAVAILABLE_INCOMPATIBLE_SCHEMA)


@dataclass(frozen=True, slots=True)
class CheckpointResult:
    duration_us: int
    wal_frames: int | None = None
    checkpointed_frames: int | None = None
    failure: DatabaseFailure | None = None


@dataclass(frozen=True, slots=True)
class PersistenceCounters:
    durable_quarantine_successes: int = 0
    durable_quarantine_failures: int = 0
    batch_transaction_attempts: int = 0
    batch_transaction_commits: int = 0
    batch_transaction_failures: int = 0
    batch_entities_committed: int = 0
    batch_commit_duration_total_us: int = 0
    batch_commit_duration_max_us: int = 0
    wal_checkpoint_attempts: int = 0
    wal_checkpoint_successes: int = 0
    wal_checkpoint_failures: int = 0


class PersistenceRecovery:
    def __init__(
        self,
        database,
        queue,
        *,
        instance,
        clock,
        transactions,
        minimum_free_bytes,
        monotonic_rate_bound_ppm,
    ):
        minimum_wait_monotonic_us(0, rate_bound_ppm=monotonic_rate_bound_ppm)
        self.database = database
        self.queue = queue
        self.instance = instance
        self.clock = clock
        self.transactions = transactions
        self.minimum_free_bytes = minimum_free_bytes
        self._rate_bound = monotonic_rate_bound_ppm
        self._enabled = False
        self.needs_validation = False
        self.retry_deadline_monotonic_us = None
        self._backoff_us = 0
        self._operator_recovery_requested = False
        self.checkpoint_pending = False
        self.last_failure = None
        self.counters = PersistenceCounters()
        self.transitions = [0] * len(State)
        self._clean_stop = None

    def increment(self, **increments):
        if any(type(value) is not int or value < 0 for value in increments.values()):
            raise ValueError("counter increments must be non-negative integers")
        self.counters = replace(
            self.counters,
            **{
                name: min(_INT64_MAX, getattr(self.counters, name) + value)
                for name, value in increments.items()
            },
        )

    @property
    def operator_required(self):
        return self.queue.snapshot().admission_snapshot.state in _OPERATOR_STATES

    def due(self):
        return (
            self._enabled
            and (not self.operator_required or self._operator_recovery_requested)
            and (
                self.retry_deadline_monotonic_us is None
                or self.clock.now_monotonic_us() >= self.retry_deadline_monotonic_us
            )
        )

    def begin_attempt(self):
        self._operator_recovery_requested = False

    def publish(self, state):
        previous = self.queue.snapshot().admission_snapshot
        if previous.state is not state:
            self.queue.publish_admission_state(
                PersistenceAdmissionSnapshot(
                    previous.generation + 1, state, self.clock.now_monotonic_us()
                )
            )
            self.transitions[state.value] = min(
                _INT64_MAX, self.transitions[state.value] + 1
            )

    def enable_admission(self):
        if self._enabled:
            raise RuntimeError("startup admission has already been enabled")
        self._enabled = True
        self.publish(State.AVAILABLE)

    def request_operator_recovery(self):
        if not self.operator_required:
            raise RuntimeError("operator recovery requires an operator-recovery state")
        self._operator_recovery_requested = True
        self.needs_validation = True

    def fail(self, failure, *, control=False):
        """Join an episode; unrelated control failures cannot postpone its retry."""
        if control:
            self.checkpoint_pending = True
            if self.operator_required:
                return  # A control request cannot authorize operator recovery.
        self.needs_validation = True
        self.last_failure = failure
        self.publish(failure.admission_state)
        if failure.admission_state in _OPERATOR_STATES:
            self.retry_deadline_monotonic_us = None
        elif not control or self.retry_deadline_monotonic_us is None:
            self._backoff_us = (
                min(5_000_000, self._backoff_us * 2) if self._backoff_us else 250_000
            )
            wait = minimum_wait_monotonic_us(
                self._backoff_us, rate_bound_ppm=self._rate_bound
            )
            self.retry_deadline_monotonic_us = checked_monotonic_deadline(
                self.clock.now_monotonic_us(), wait
            )
        if failure.admission_state is State.UNAVAILABLE_CORRUPT:
            try:
                self.database.close()
            except (sqlite3.Error, OSError):
                pass  # Closing must not replace the original classification.

    def allow_clean_stop(self, marker):
        """Retain the exact submitted marker after COMMIT may have run."""
        self._clean_stop = (
            marker.stopped_at_monotonic_us,
            marker.communicator_state_generation,
        )

    def revalidate(self):
        failure = self.database.revalidate(minimum_free_bytes=self.minimum_free_bytes)
        if failure is not None:
            raise StorageUnavailable(failure)
        start = SqliteRepository(self.database.connection).find_receiver_instance(
            self.instance.receiver_instance_id
        )
        if (
            start is None
            or start[3] != self.instance.started_at_monotonic_us
            or start[4:] not in ((None, None), self._clean_stop)
        ):
            raise PersistenceIdentityCollision(
                "recovery lost the exact receiver instance"
            )
        self.needs_validation = False

    def complete(self, *, ordinary_pending):
        if (
            not ordinary_pending
            and not self.checkpoint_pending
            and not self.needs_validation
        ):
            self.retry_deadline_monotonic_us = None
            self._backoff_us = 0
            self.last_failure = None
            self.publish(State.AVAILABLE)

    def checkpoint(self, *, ordinary_pending):
        """One explicit PASSIVE attempt; partial progress is a non-error result.

        No hard I/O time bound or fresh application-write proof is implied.
        """
        if ordinary_pending or not self.due():
            return None
        self.begin_attempt()
        started = self.clock.now_monotonic_us()
        self.increment(wal_checkpoint_attempts=1)
        try:
            if self.needs_validation or self.checkpoint_pending:
                self.revalidate()
            failure = self.database.inspect_storage(
                minimum_free_bytes=self.minimum_free_bytes
            )
            if failure is not None:
                raise StorageUnavailable(failure)
            busy, total, completed = self.transactions.checkpoint(
                self.database.connection
            )
            if busy:
                raise StorageUnavailable(
                    DatabaseFailure(
                        State.UNAVAILABLE_IO, sqlite3.SQLITE_BUSY, sqlite3.SQLITE_BUSY
                    )
                )
        except (
            sqlite3.Error,
            OSError,
            StorageUnavailable,
            PersistenceIdentityCollision,
        ) as error:
            failure = classify_global_failure(error)
            self.checkpoint_pending = True
            self.increment(wal_checkpoint_failures=1)
            self.fail(failure)
            return CheckpointResult(
                self.clock.now_monotonic_us() - started, failure=failure
            )
        self.checkpoint_pending = False
        self.increment(wal_checkpoint_successes=1)
        self.complete(ordinary_pending=False)
        return CheckpointResult(
            self.clock.now_monotonic_us() - started, total, completed
        )
