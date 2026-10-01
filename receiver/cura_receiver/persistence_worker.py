"""The receiver's sole configuration/SQLite owner, built from persistence components."""

from __future__ import annotations

import sqlite3
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from threading import Event, Lock, Thread

from .communicator_state_persistence import (
    CommunicatorStatePolicy,
    classify_communicator_state_rows,
)
from .checkpoint_maintenance import CheckpointMaintenance
from .generated.receiver_enums_generated import PersistenceAdmissionState as State
from .ordinary_persistence import OrdinaryPersistence
from .persist_queue import PersistQueue, PersistenceAdmissionSnapshot
from .persistence_control_operations import PersistenceControlOperations
from .persistence_control_channel import PersistenceControlChannel, control_failure
from .persistence_control_execution import (
    ControlDeadlineExceeded,
    ControlCommandKind as Kind,
)
from .persistence_control_values import (
    CommunicatorStateLoadResult,
    ReceiverCleanStopCommitDisposition as StopDisposition,
)
from .platform.linux_boot_identity import LINUX_BOOT_ID_PATH
from .platform.linux_clocks import LinuxOsClock
from .receiver_configuration import (
    ReceiverConfigurationReader,
    ReceiverConfigurationLoadResult,
    DEFAULT_CONFIGURATION_PATH,
)
from .receiver_startup import (
    ReceiverInstanceStart,
    ReceiverInstanceStartResult,
    start_receiver_instance,
)
from .sqlite_database import DatabaseFailure, StorageUnavailable
from .sqlite_repository import SqliteRepository
from .sqlite_transactions import ObservedSqliteTransactions, SqliteTransactions
from .persistence_failures import classify_global_failure


@dataclass(frozen=True, slots=True)
class PersistenceWorkerStartup:
    """Immutable startup evidence; no SQLite handle crosses the owner boundary."""

    configuration_load: ReceiverConfigurationLoadResult
    instance_start: ReceiverInstanceStartResult | None = None
    database_failure: DatabaseFailure | None = None
    state_load: CommunicatorStateLoadResult | None = None


class PersistenceWorker(Thread):
    def __init__(
        self,
        *,
        instance: ReceiverInstanceStart,
        database_path: Path,
        configuration_path: Path = DEFAULT_CONFIGURATION_PATH,
        boot_id_path: Path = LINUX_BOOT_ID_PATH,
        expected_owner_uid=None,
        clock=None,
        policy: CommunicatorStatePolicy | None = None,
        minimum_free_bytes: int = 0,
        transactions=None,
        host_observations=None,
        flush_interval_us: int = 5_000_000,
        wake_threshold_entities: int = 64,
        batch_limit_entities: int = 64,
        checkpoint_interval_us: int = 5_000_000,
    ):
        super().__init__(name="receiver-persistence", daemon=True)
        if type(instance) is not ReceiverInstanceStart:
            raise TypeError("worker requires its immutable process start")
        if not all(
            isinstance(value, Path)
            for value in (database_path, configuration_path, boot_id_path)
        ):
            raise TypeError("worker paths must be Path values")
        if type(minimum_free_bytes) is not int or minimum_free_bytes < 0:
            raise ValueError("minimum free bytes must be nonnegative")
        for value in (wake_threshold_entities, batch_limit_entities):
            if type(value) is not int or not 1 <= value <= 500:
                raise ValueError("worker entity thresholds must be in 1..500")
        for value in (flush_interval_us, checkpoint_interval_us):
            if type(value) is not int or value < 1:
                raise ValueError(
                    "worker flush and checkpoint intervals must be positive"
                )
        self._instance = instance
        self._database_path = database_path
        self._configuration_path = configuration_path
        self._boot_id_path = boot_id_path
        self._expected_owner_uid = expected_owner_uid
        self._clock = clock if clock is not None else LinuxOsClock()
        self._policy = policy if policy is not None else CommunicatorStatePolicy()
        self._minimum_free_bytes = minimum_free_bytes
        self._checkpoint = CheckpointMaintenance(
            clock=self._clock, interval_us=checkpoint_interval_us
        )
        self._transactions = ObservedSqliteTransactions(
            transactions if transactions is not None else SqliteTransactions(),
            self._checkpoint,
        )
        self._host_observations = host_observations
        self._wake = Event()
        self.queue = PersistQueue(wake_event=self._wake)
        self._scheduler_lock = Lock()
        self._mailbox = deque()
        self._channel_closed = False
        self.control = PersistenceControlChannel(self)
        self._bypassed_persistence_action = None
        self._startup_completed = Event()
        self._startup = None
        self._stop_deadline = None
        self._database = None
        self._ordinary = None
        self._recovery = None
        self._controls = None
        self._flush_interval = flush_interval_us
        self._wake_threshold = wake_threshold_entities
        self._batch_limit = batch_limit_entities
        self._flush_deadline = 0
        self._last_persistence_action = None
        self._draining = False

    @property
    def startup_snapshot(self):
        with self._scheduler_lock:
            return self._startup

    def wait_started(self, *, deadline_monotonic_us):
        remaining = max(0, deadline_monotonic_us - self._clock.now_monotonic_us())
        self._startup_completed.wait(remaining / 1_000_000)
        return self.startup_snapshot

    def request_stop(self, *, deadline_monotonic_us):
        """Close submissions and drain within the budget, then checkpoint/close.

        The caller closes/drains the queue and requests its clean marker before
        this final handoff when a clean stop is possible. This method never
        manufactures a marker and cannot preempt an in-flight kernel call.
        """
        if (
            type(deadline_monotonic_us) is not int
            or not 0 <= deadline_monotonic_us <= (1 << 64) - 1
        ):
            raise ValueError("stop deadline must be an unsigned monotonic timestamp")
        self.queue.close()
        with self._scheduler_lock:
            self._channel_closed = True
            if (
                self._stop_deadline is None
                or deadline_monotonic_us < self._stop_deadline
            ):
                self._stop_deadline = deadline_monotonic_us
            self._wake.set()

    def _publish_failure(self, failure):
        previous = self.queue.snapshot().admission_snapshot
        if previous.state is not failure.admission_state:
            self.queue.publish_admission_state(
                PersistenceAdmissionSnapshot(
                    previous.generation + 1,
                    failure.admission_state,
                    self._clock.now_monotonic_us(),
                )
            )

    def _initialize(self):
        reader = ReceiverConfigurationReader(
            self._configuration_path,
            boot_id_path=self._boot_id_path,
            expected_owner_uid=self._expected_owner_uid,
        )
        started = start_receiver_instance(
            self._instance,
            configuration_reader=reader,
            database_path=self._database_path,
            minimum_free_bytes=self._minimum_free_bytes,
        )
        failure = started.database_failure or (
            started.instance_start.failure if started.instance_start else None
        )
        state_load = None
        if started.database is not None:
            self._database = started.database
            # Startup commits outside the ordinary/control helper. Its durable
            # lifecycle row and inherited WAL both have unknown coverage here.
            self._checkpoint.mark_possible_work()
            try:
                self._ordinary = OrdinaryPersistence(
                    self._database,
                    self.queue,
                    instance=self._instance,
                    clock=self._clock,
                    transactions=self._transactions,
                    checkpoint_maintenance=self._checkpoint,
                    host_observations=self._host_observations,
                    minimum_free_bytes=self._minimum_free_bytes,
                )
                self._controls = PersistenceControlOperations(
                    self._database,
                    instance=self._instance,
                    queue=self.queue,
                    clock=self._clock,
                    policy=self._policy,
                    transactions=self._transactions,
                    minimum_free_bytes=self._minimum_free_bytes,
                )
                self._recovery = self._ordinary.recovery
                repository = SqliteRepository(self._database.connection)
                state_load = classify_communicator_state_rows(
                    repository.read_communicator_state_rows(), repository, self._policy
                )
                self._ordinary.enable_admission()
            except (sqlite3.Error, OSError, StorageUnavailable) as error:
                failure = classify_global_failure(error)
                self._database.close()
                self._ordinary = self._controls = None
                self._recovery = None
        if failure is not None:
            self._publish_failure(failure)
        with self._scheduler_lock:
            self._startup = PersistenceWorkerStartup(
                started.configuration_load, started.instance_start, failure, state_load
            )
            self._startup_completed.set()

    def _work_due(self, now, snapshot, *, preferred_action=None):
        ordinary = self._ordinary
        if ordinary is None or snapshot.admission_snapshot.state in (
            State.UNAVAILABLE_CORRUPT,
            State.UNAVAILABLE_INCOMPATIBLE_SCHEMA,
        ):
            return None
        retry = ordinary.retry_deadline_monotonic_us
        if retry is not None and now < retry:
            return None
        ordinary_due = snapshot.published_entities and (
            retry is not None
            or self._draining
            or snapshot.closed
            or self._stop_deadline is not None
            or snapshot.published_entities >= self._wake_threshold
            or now >= self._flush_deadline
        )
        checkpoint_due = not ordinary.pending_entities and (
            ordinary.checkpoint_pending or self._checkpoint.due(now)
        )
        if retry is not None and ordinary_due:
            # Recovery admission is closed, so retained FIFO work is finite.
            # Resolve it before the episode's required checkpoint.
            return "ordinary"
        if preferred_action == "ordinary" and ordinary_due:
            return "ordinary"
        if preferred_action == "checkpoint" and checkpoint_due:
            return "checkpoint"
        if checkpoint_due and (
            not ordinary_due or self._last_persistence_action == "ordinary"
        ):
            return "checkpoint"
        if ordinary_due:
            return "ordinary"
        if checkpoint_due:
            return "checkpoint"
        return None

    def _next_deadline(self, now, snapshot):
        deadlines = []
        if self._stop_deadline is not None:
            deadlines.append(self._stop_deadline)
        ordinary = self._ordinary
        if ordinary is not None and snapshot.admission_snapshot.state not in (
            State.UNAVAILABLE_CORRUPT,
            State.UNAVAILABLE_INCOMPATIBLE_SCHEMA,
        ):
            retry = ordinary.retry_deadline_monotonic_us
            if retry is not None:
                deadlines.append(retry)
            else:
                if snapshot.published_entities:
                    deadlines.append(self._flush_deadline)
                if self._checkpoint.pending and not ordinary.pending_entities:
                    deadlines.append(self._checkpoint.deadline_monotonic_us)
        return min(deadlines) if deadlines else None

    def _wait_for_work(self, timeout_seconds):
        """Interruptible wait after clearing and rechecking every work predicate."""
        self._wake.wait(timeout_seconds)

    def _dispatch_work(self, action):
        self._last_persistence_action = action
        if action == "ordinary":
            self._draining = True
            self._ordinary.attempt(max_entities=self._batch_limit)
            if not self.queue.snapshot().published_entities:
                self._draining = False
                self._flush_deadline = (
                    self._clock.now_monotonic_us() + self._flush_interval
                )
        elif action == "checkpoint":
            self._ordinary.checkpoint()

    def _select_dispatch(self, now, snapshot):
        """Called under the same short lock as mailbox insertion, with no I/O."""
        action = self._work_due(
            now, snapshot, preferred_action=self._bypassed_persistence_action
        )
        # Preserve the selected kind across a control turn, but never bypass
        # eligibility or recovery gates when that turn changes storage state.
        if action != self._bypassed_persistence_action:
            self._bypassed_persistence_action = None
        if self._mailbox and (
            action is None or self._bypassed_persistence_action is None
        ):
            command = self._mailbox.popleft()
            self._bypassed_persistence_action = action
            if self._channel_closed:
                command.finish(control_failure(command.request.kind, "CHANNEL_CLOSED"))
                return "completed_control", None
            try:
                command.start()
            except ControlDeadlineExceeded:
                command.finish(
                    control_failure(command.request.kind, "DEADLINE_EXCEEDED")
                )
                return "completed_control", None
            return "control", command
        self._bypassed_persistence_action = None
        return action, None

    def _dispatch_control(self, command):
        request = command.request
        executed = False
        if request.kind is Kind.CONFIGURATION:
            result = self._startup.configuration_load
        elif self._controls is None:
            failure = self._startup.database_failure
            if failure is None:
                result = control_failure(request.kind, "CHANNEL_CLOSED")
            else:
                result = control_failure(
                    request.kind,
                    "DATABASE_ERROR",
                    evidence=dict(
                        sqlite_primary_code=failure.sqlite_primary_code,
                        sqlite_extended_code=failure.sqlite_extended_code,
                        os_errno=failure.os_errno,
                    ),
                )
        elif self._recovery.operator_required:
            failure = self._recovery.last_failure
            result = control_failure(
                request.kind,
                "DATABASE_ERROR",
                evidence=dict(
                    sqlite_primary_code=failure.sqlite_primary_code,
                    sqlite_extended_code=failure.sqlite_extended_code,
                    os_errno=failure.os_errno,
                ),
            )
        else:
            executed = True
            if request.kind is Kind.LOAD_STATE:
                result = self._controls.load_state(command)
            elif request.kind is Kind.COMMIT_STATE:
                result = self._controls.commit_state(request.payload, command)
            else:
                result = self._controls.commit_clean_stop(request.payload, command)
        if executed:
            if request.kind is Kind.CLEAN_STOP and result.disposition in (
                StopDisposition.COMMITTED,
                StopDisposition.ALREADY_COMMITTED,
                StopDisposition.OUTCOME_UNKNOWN,
            ):
                self._recovery.allow_clean_stop(request.payload)
            if self._controls.last_failure is not None:
                self._recovery.fail(self._controls.last_failure, control=True)
        command.finish(result)

    def run(self):
        try:
            self._initialize()
            self._flush_deadline = self._clock.now_monotonic_us() + self._flush_interval
            while True:
                self._wake.clear()
                with self._scheduler_lock:
                    now = self._clock.now_monotonic_us()
                    snapshot = self.queue.snapshot()
                    if self._stop_deadline is not None:
                        if now >= self._stop_deadline:
                            break
                        if snapshot.closed_and_drained:
                            retry = (
                                self._ordinary.retry_deadline_monotonic_us
                                if self._ordinary
                                else None
                            )
                            if retry is None or now >= retry:
                                break
                    action, command = self._select_dispatch(now, snapshot)
                    deadline = self._next_deadline(now, snapshot)
                if action == "control":
                    self._dispatch_control(command)
                elif action == "completed_control":
                    continue
                elif action is not None:
                    self._dispatch_work(action)
                else:
                    self._wait_for_work(
                        None if deadline is None else max(0, deadline - now) / 1_000_000
                    )
        finally:
            with self._scheduler_lock:
                self._channel_closed = True
                while self._mailbox:
                    command = self._mailbox.popleft()
                    command.finish(
                        control_failure(command.request.kind, "CHANNEL_CLOSED")
                    )
            if self._database is not None:
                try:
                    if (
                        self._ordinary is not None
                        and (
                            self._checkpoint.pending
                            or self._ordinary.checkpoint_pending
                        )
                        and self._stop_deadline is not None
                        and self._clock.now_monotonic_us() < self._stop_deadline
                    ):
                        # The same recovery gate enforces deadlines, retained work
                        # and operator authorization. A failure still permits close.
                        self._ordinary.checkpoint()
                finally:
                    self._database.close()
