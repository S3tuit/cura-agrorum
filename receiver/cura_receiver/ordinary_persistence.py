"""Caller-driven ordinary transactions, with queue ownership until durability.

The caller is the sole persistence owner. It completes startup before calling
``enable_admission`` and drives attempts at safe boundaries. This component
never starts a thread or waits for work.
"""

from __future__ import annotations

import sqlite3
from dataclasses import dataclass, fields, replace
from enum import Enum, auto

from .elapsed_duration import (
    MONOTONIC_ELAPSED_RATE_BOUND_PPM,
    checked_monotonic_deadline,
    minimum_wait_monotonic_us,
)
from .generated import receiver_entities_generated as rows
from .generated.receiver_enums_generated import (
    DATABASE_SCHEMA_VERSION,
    AckSelection,
    AckTxResult,
    DiagnosticErrorDomain,
    DiagnosticSeverity,
    ProcessingResult,
    RadioState,
    RtcHealth,
    SystemTimeQuality,
)
from .generated.receiver_enums_generated import (
    DiagnosticOperation as Operation,
)
from .generated.receiver_enums_generated import (
    PersistenceAdmissionState as State,
)
from .generated.receiver_enums_generated import (
    PersistenceClassification as Classification,
)
from .generated.receiver_enums_generated import (
    PersistQueueEntityKind as Kind,
)
from .persist_queue import (
    PersistenceAdmissionSnapshot,
    PersistQueue,
    PersistQueueBatchLease,
)
from .persist_queue_entities import (
    MeasurementProfileUnitV1,
    ProfileOnlyUnitV1,
    ReceiverHealthRequestV1,
)
from .persistence_failures import classify_entity_failure, classify_global_failure
from .platform.linux_host_observations import LinuxHostObservations
from .ports.clocks import MonotonicClock
from .ports.host_observations import HostObservations, HostObservationSource
from .quarantine_evidence import (
    QuarantineEvidenceError,
    encode_quarantine_evidence_v1,
    quarantine_evidence_sha256,
)
from .reading_persistence import (
    PersistenceIdentityCollision,
    PreparedReading,
    exact_sql_row,
    prepare_reading,
    prepare_replayed_reading,
    validate_prepared_reading,
)
from .receiver_startup import ReceiverInstanceStart
from .sqlite_database import (
    DatabaseFailure,
    StorageUnavailable,
    ReceiverDatabase,
    validate_receiver_connection,
)
from .sqlite_repository import SqliteRepository, validate_sqlite_parameters
from .sqlite_transactions import SqliteTransactions

_INT64_MAX = (1 << 63) - 1


def _require_enum_fields(entity: object, **expected_types: type[Enum]) -> None:
    # Projection extracts .value; check the logical type before that information
    # is lost, including before a reading replay compares its projected profile.
    for name, expected in expected_types.items():
        if type(getattr(entity, name)) is not expected:
            raise TypeError(f"{name} must be a {expected.__name__}")


def _validate_profile_enums(profile: rows.MessageProfilingV1) -> None:
    if type(profile) is not rows.MessageProfilingV1:
        raise TypeError("invalid profiling record")
    _require_enum_fields(
        profile,
        processing_result=ProcessingResult,
        ack_selected=AckSelection,
        ack_tx_result=AckTxResult,
    )


class OrdinaryBatchCommitOutcome(Enum):
    COMMITTED = auto()
    NOT_COMMITTED = auto()
    OUTCOME_UNKNOWN = auto()


@dataclass(frozen=True, slots=True)
class PersistenceAttemptResult:
    outcome: OrdinaryBatchCommitOutcome
    failure: DatabaseFailure | None = None
    acknowledged_entities: int = 0


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


@dataclass(frozen=True, slots=True)
class _PreparedUnit:
    row: (
        rows.ClockObservationV1
        | rows.DiagnosticV1
        | rows.MessageProfileRowV1
        | rows.ReceiverHealthV1
    )
    reading: PreparedReading | None = None
    measurement: MeasurementProfileUnitV1 | None = None


class OrdinaryPersistence:
    """Own one validated connection and at most one pending FIFO batch.

    Construction checks the durable process start; it does not complete state,
    radio or time startup. The startup owner calls ``enable_admission`` only
    after all startup prerequisites are satisfied. Connection ownership moves
    here; callers must not operate it concurrently or open nested transactions.
    """

    def __init__(
        self,
        database: ReceiverDatabase,
        queue: PersistQueue,
        *,
        instance: ReceiverInstanceStart,
        clock: MonotonicClock,
        transactions: SqliteTransactions | None = None,
        host_observations: HostObservationSource | None = None,
        minimum_free_bytes: int = 0,
        monotonic_rate_bound_ppm: int = MONOTONIC_ELAPSED_RATE_BOUND_PPM,
    ) -> None:
        if type(instance) is not ReceiverInstanceStart:
            raise TypeError("instance must be a ReceiverInstanceStart")
        if type(database) is not ReceiverDatabase:
            raise TypeError("database must be a validated ReceiverDatabase")
        connection = database.connection
        if type(minimum_free_bytes) is not int or minimum_free_bytes < 0:
            raise ValueError("minimum_free_bytes must be a non-negative integer")
        if connection.in_transaction:
            raise ValueError(
                "ordinary persistence requires a transaction-free connection"
            )
        failure = database.inspect_storage(minimum_free_bytes=0)
        if failure is None:
            failure = validate_receiver_connection(connection, database.group_id)
        if failure is not None:
            raise StorageUnavailable(failure)
        repository = SqliteRepository(connection)
        start = repository.find_receiver_instance(instance.receiver_instance_id)
        if (
            start is None
            or start[3] != instance.started_at_monotonic_us
            or start[4:] != (None, None)
        ):
            raise ValueError(
                "ordinary persistence requires its exact durable active instance"
            )
        if queue.snapshot().admission_snapshot.generation != 0:
            raise ValueError(
                "ordinary persistence requires ownership before admission publication"
            )
        self._database = database
        self._repository = repository
        self._queue = queue
        self._instance = instance
        self._minimum_free_bytes = minimum_free_bytes
        minimum_wait_monotonic_us(0, rate_bound_ppm=monotonic_rate_bound_ppm)
        self._rate_bound = monotonic_rate_bound_ppm
        self._clock = clock
        self._transactions = (
            transactions if transactions is not None else SqliteTransactions()
        )
        self._lease: PersistQueueBatchLease | None = None
        self._prepared: list[_PreparedUnit | None] = []
        self._tokens: tuple[object, ...] = ()
        self._unknown = False
        self._enabled = False
        self._host = (
            host_observations
            if host_observations is not None
            else LinuxHostObservations(database.path)
        )
        self._counters = PersistenceCounters()
        self._transitions = [0] * len(State)
        self._suspected = {}
        self._isolation_attempts = {}
        self._isolating = False
        self._quarantine = {}
        self._unknown_indices = ()
        self._unknown_quarantine = False
        self._needs_validation = False
        self._retry_deadline: int | None = None
        self._backoff_us = 0
        self._operator_recovery_requested = False
        self._checkpoint_pending = False

    @property
    def counters(self) -> PersistenceCounters:
        return self._counters

    def _increment(self, **increments: int) -> None:
        if any(type(value) is not int or value < 0 for value in increments.values()):
            raise ValueError("counter increments must be non-negative integers")
        self._counters = replace(
            self._counters,
            **{
                name: min(_INT64_MAX, getattr(self._counters, name) + value)
                for name, value in increments.items()
            },
        )

    @property
    def retry_deadline_monotonic_us(self) -> int | None:
        return self._retry_deadline

    @property
    def checkpoint_pending(self) -> bool:
        return self._checkpoint_pending

    def _recovery_due(self) -> bool:
        if not self._enabled:
            return False
        state = self._queue.snapshot().admission_snapshot.state
        if state in (State.UNAVAILABLE_CORRUPT, State.UNAVAILABLE_INCOMPATIBLE_SCHEMA):
            if not self._operator_recovery_requested:
                return False
        if (
            self._retry_deadline is not None
            and self._clock.now_monotonic_us() < self._retry_deadline
        ):
            return False
        return True

    def _complete_recovery(self) -> None:
        if not self._tokens and not self._checkpoint_pending:
            self._retry_deadline = None
            self._backoff_us = 0
            self._publish(State.AVAILABLE)

    def checkpoint(self) -> CheckpointResult | None:
        """Run one PASSIVE checkpoint after retained batch work is resolved.

        None means startup is incomplete, ordinary work remains retained, or
        recovery is not due/authorized. The caller owns dispatch and retries,
        including when ``checkpoint_pending`` is true with an empty queue.
        Reader-limited progress is success; no hard I/O time bound is implied.
        """
        if self._tokens or not self._recovery_due():
            return None
        self._operator_recovery_requested = False
        started = self._clock.now_monotonic_us()
        self._increment(wal_checkpoint_attempts=1)
        try:
            if self._needs_validation or self._checkpoint_pending:
                self._revalidate()
            failure = self._database.inspect_storage(
                minimum_free_bytes=self._minimum_free_bytes
            )
            if failure is not None:
                raise StorageUnavailable(failure)
            busy, total, completed = self._transactions.checkpoint(
                self._database.connection
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
            self._checkpoint_pending = True
            self._needs_validation = True
            self._increment(wal_checkpoint_failures=1)
            self._schedule_recovery(failure)
            if failure.admission_state is State.UNAVAILABLE_CORRUPT:
                self.close()
            return CheckpointResult(
                self._clock.now_monotonic_us() - started, failure=failure
            )
        self._checkpoint_pending = False
        self._needs_validation = False
        self._increment(wal_checkpoint_successes=1)
        self._complete_recovery()
        return CheckpointResult(
            self._clock.now_monotonic_us() - started, total, completed
        )

    def request_operator_recovery(self) -> None:
        """Explicit maintenance handoff; the next attempt still validates everything."""
        if self._queue.snapshot().admission_snapshot.state not in (
            State.UNAVAILABLE_CORRUPT,
            State.UNAVAILABLE_INCOMPATIBLE_SCHEMA,
        ):
            raise RuntimeError("operator recovery requires an operator-recovery state")
        self._operator_recovery_requested = True
        self._needs_validation = True

    def _schedule_recovery(self, failure: DatabaseFailure) -> None:
        self._publish(failure.admission_state)
        if failure.admission_state in (
            State.UNAVAILABLE_CORRUPT,
            State.UNAVAILABLE_INCOMPATIBLE_SCHEMA,
        ):
            self._retry_deadline = None
        else:
            self._backoff_us = (
                min(5_000_000, self._backoff_us * 2) if self._backoff_us else 250_000
            )
            wait = minimum_wait_monotonic_us(
                self._backoff_us, rate_bound_ppm=self._rate_bound
            )
            self._retry_deadline = checked_monotonic_deadline(
                self._clock.now_monotonic_us(), wait
            )

    def _revalidate(self) -> None:
        failure = self._database.revalidate(minimum_free_bytes=self._minimum_free_bytes)
        if failure is not None:
            raise StorageUnavailable(failure)
        self._repository = SqliteRepository(self._database.connection)
        start = self._repository.find_receiver_instance(
            self._instance.receiver_instance_id
        )
        if (
            start is None
            or start[3] != self._instance.started_at_monotonic_us
            or start[4:] != (None, None)
        ):
            raise PersistenceIdentityCollision(
                "recovery lost the exact active receiver instance"
            )

    def enable_admission(self) -> None:
        """Startup's explicit handoff, never a storage-recovery shortcut."""
        if self._enabled:
            raise RuntimeError("startup admission has already been enabled")
        self._enabled = True
        self._publish(State.AVAILABLE)

    def _publish(self, state: State) -> None:
        previous = self._queue.snapshot().admission_snapshot
        if previous.state is not state:
            self._queue.publish_admission_state(
                PersistenceAdmissionSnapshot(
                    previous.generation + 1, state, self._clock.now_monotonic_us()
                )
            )
            self._transitions[state.value] = min(
                _INT64_MAX, self._transitions[state.value] + 1
            )

    def _claim(self, max_entities: int) -> bool:
        if self._lease is None:
            lease = self._queue.claim_batch(
                max_entities=len(self._tokens) or max_entities
            )
            if lease is None:
                return False
            tokens = tuple(entry.token for entry in lease.entries)
            if self._tokens and tokens != self._tokens:
                raise RuntimeError("pending FIFO prefix changed")
            self._lease = lease
            if not self._tokens:
                self._tokens = tokens
                self._prepared = [None] * len(tokens)
        return True

    def _prepare(self, entity: object, kind: Kind) -> _PreparedUnit:
        if (
            kind is Kind.MEASUREMENT_PROFILE
            and type(entity) is MeasurementProfileUnitV1
        ):
            _validate_profile_enums(entity.profile)
            existing = self._repository.find_message_profile(
                entity.profile.receiver_instance_id, entity.profile.occurrence_sequence
            )
            reading = (
                prepare_reading(self._repository, entity)
                if existing is None
                else prepare_replayed_reading(self._repository, entity, existing)
            )
            return _PreparedUnit(reading.profile, reading, entity)
        if kind is Kind.CLOCK_OBSERVATION and type(entity) is rows.ClockObservationV1:
            _require_enum_fields(
                entity, system_time_quality=SystemTimeQuality, rtc_health=RtcHealth
            )
            return _PreparedUnit(entity)
        if kind is Kind.DIAGNOSTIC and type(entity) is rows.DiagnosticV1:
            _require_enum_fields(
                entity,
                severity=DiagnosticSeverity,
                error_domain=DiagnosticErrorDomain,
                operation=Operation,
            )
            return _PreparedUnit(entity)
        if (
            kind is Kind.PROFILE_ONLY
            and type(entity) is ProfileOnlyUnitV1
            and type(entity.profile) is rows.MessageProfilingV1
        ):
            _validate_profile_enums(entity.profile)
            return _PreparedUnit(
                rows.MessageProfileRowV1(entity.profile, Classification.NOT_APPLICABLE)
            )
        if (
            kind is Kind.RECEIVER_HEALTH_REQUEST
            and type(entity) is ReceiverHealthRequestV1
        ):
            _require_enum_fields(
                entity,
                radio_state=RadioState,
                system_time_quality=SystemTimeQuality,
                rtc_health=RtcHealth,
            )
            for array in (
                entity.radio_recovery_attempts_by_reason,
                entity.chrony_step_command_results,
                entity.rtc_write_results,
                entity.persist_queue_admission_counts,
            ):
                if type(array) is not tuple:
                    raise TypeError("health arrays must be immutable tuples")
            if any(
                type(part) is not tuple
                for part in entity.persist_queue_admission_counts
            ):
                raise TypeError("health matrix rows must be immutable tuples")
            sampled = self._clock.now_monotonic_us()
            try:
                host = self._host.sample()
                if type(host) is not HostObservations:
                    raise TypeError("invalid host observation source value")
            except OSError:
                host = HostObservations()
            except (TypeError, ValueError, AttributeError, OverflowError):
                raise StorageUnavailable(
                    DatabaseFailure(State.UNAVAILABLE_INCOMPATIBLE_SCHEMA)
                ) from None
            admission = self._queue.snapshot().admission_snapshot
            return _PreparedUnit(
                rows.ReceiverHealthV1(
                    **{
                        field.name: getattr(entity, field.name)
                        for field in fields(entity)
                    },
                    persistence_sampled_at_monotonic_us=sampled,
                    persistence_admission_generation=admission.generation,
                    persistence_admission_state=admission.state,
                    persistence_admission_changed_at_monotonic_us=admission.changed_at_monotonic_us,
                    persistence_admission_transition_counts=tuple(self._transitions),
                    **{
                        field.name: getattr(self._counters, field.name)
                        for field in fields(self._counters)
                    },
                    **{field.name: getattr(host, field.name) for field in fields(host)},
                )
            )
        raise TypeError("entity does not match its queue specification")

    def _write(self, work: _PreparedUnit) -> None:
        entity = work.row
        bindings = {
            rows.ClockObservationV1: (
                rows.CLOCK_OBSERVATION_V1_COLUMNS,
                rows.clock_observation_v1_parameters,
            ),
            rows.DiagnosticV1: (
                rows.DIAGNOSTIC_V1_COLUMNS,
                rows.diagnostic_v1_parameters,
            ),
            rows.MessageProfileRowV1: (
                rows.MESSAGE_PROFILE_ROW_V1_COLUMNS,
                rows.message_profile_row_v1_parameters,
            ),
            rows.ReceiverHealthV1: (
                rows.RECEIVER_HEALTH_V1_COLUMNS,
                rows.receiver_health_v1_parameters,
            ),
        }
        columns, binder = bindings[type(entity)]
        validate_sqlite_parameters(columns, binder(entity))
        if type(entity) is rows.ClockObservationV1:
            existing = self._repository.find_clock_observation(
                entity.receiver_instance_id, entity.observation_sequence
            )
            parameters = rows.clock_observation_v1_parameters(entity)
            insert = self._repository.insert_clock_observation
        elif type(entity) is rows.DiagnosticV1:
            existing = self._repository.find_diagnostic(
                entity.receiver_instance_id, entity.diagnostic_sequence
            )
            parameters = rows.diagnostic_v1_parameters(entity)
            insert = self._repository.insert_diagnostic
        elif type(entity) is rows.MessageProfileRowV1:
            profile = entity.profile
            existing = self._repository.find_message_profile(
                profile.receiver_instance_id, profile.occurrence_sequence
            )
            parameters = rows.message_profile_row_v1_parameters(entity)
            insert = self._repository.insert_message_profile
            if work.reading is not None:
                candidate = work.measurement.candidate
                reading = self._repository.find_reading_message(
                    candidate.node_id, candidate.message_id
                )
                if (
                    existing is None
                    and reading is not None
                    and reading[-2:]
                    == (profile.receiver_instance_id, profile.occurrence_sequence)
                ):
                    raise PersistenceIdentityCollision(
                        "reading effect exists without its profile"
                    )
                validate_prepared_reading(
                    self._repository, work.reading, work.measurement
                )
        elif type(entity) is rows.ReceiverHealthV1:
            existing = self._repository.find_receiver_health(
                entity.receiver_instance_id, entity.health_sequence
            )
            parameters = rows.receiver_health_v1_parameters(entity)
            insert = self._repository.insert_receiver_health
        else:
            raise TypeError("invalid prepared row")
        if existing is not None:
            if not exact_sql_row(existing, parameters):
                raise PersistenceIdentityCollision("ordinary row identity collision")
        else:
            insert(entity)
        if work.reading is not None and work.reading.insertion is not None:
            effect = work.reading.insertion
            existing = self._repository.find_reading_message(
                effect.node_id, effect.message_id
            )
            if existing is None:
                self._repository.insert_reading_message(effect)
            elif not exact_sql_row(
                existing, rows.reading_message_row_v1_parameters(effect)
            ):
                raise PersistenceIdentityCollision("reading effect collision")

    def _rollback_failure(
        self, failure: DatabaseFailure, *, item_failure: bool = False
    ) -> tuple[DatabaseFailure, bool]:
        try:
            self._transactions.rollback(self._database.connection)
            return failure, True
        except (sqlite3.Error, OSError) as error:
            cleanup = classify_global_failure(error)
            if item_failure or cleanup.admission_state is State.UNAVAILABLE_CORRUPT:
                failure = cleanup
            self.close()
            return failure, False

    def _fail_global(
        self, failure: DatabaseFailure, *, unknown: bool, retain: bool
    ) -> PersistenceAttemptResult:
        self._needs_validation = True
        self._unknown = unknown
        if not unknown and not retain and self._lease is not None:
            self._lease.release_for_retry()
            self._lease = None
        self._schedule_recovery(failure)
        if failure.admission_state is State.UNAVAILABLE_CORRUPT:
            self.close()
        return PersistenceAttemptResult(
            (
                OrdinaryBatchCommitOutcome.OUTCOME_UNKNOWN
                if unknown
                else OrdinaryBatchCommitOutcome.NOT_COMMITTED
            ),
            failure,
        )

    def _finish(
        self, indices: tuple[int, ...], *, quarantine: bool, duration: int
    ) -> PersistenceAttemptResult:
        count = len(indices)
        if indices != tuple(range(count)):
            raise RuntimeError("durable completion must name the pending FIFO prefix")
        if quarantine:
            self._increment(durable_quarantine_successes=count)
        else:
            self._increment(
                batch_transaction_commits=1,
                batch_entities_committed=count,
                batch_commit_duration_total_us=duration,
            )
            self._counters = replace(
                self._counters,
                batch_commit_duration_max_us=min(
                    _INT64_MAX,
                    max(self._counters.batch_commit_duration_max_us, duration),
                ),
            )
        self._unknown = False
        self._unknown_indices = ()
        self._unknown_quarantine = False
        self._needs_validation = False
        self._lease.acknowledge_durable(completed_entities=count)
        self._tokens = self._tokens[count:]
        self._prepared = self._prepared[count:]
        self._suspected = {
            index - count: value
            for index, value in self._suspected.items()
            if index >= count
        }
        self._isolation_attempts = {
            index - count: value
            for index, value in self._isolation_attempts.items()
            if index >= count
        }
        self._quarantine = {
            index - count: value
            for index, value in self._quarantine.items()
            if index >= count
        }
        if not self._tokens:
            self._lease = None
            self._isolating = False
            self._complete_recovery()
        return PersistenceAttemptResult(
            OrdinaryBatchCommitOutcome.COMMITTED, acknowledged_entities=count
        )

    def _freeze_quarantine(self, index: int, failure) -> None:
        entry = self._lease.entries[index]
        evidence = encode_quarantine_evidence_v1(entry.entity, spec=entry.spec)
        self._quarantine[index] = rows.QuarantinedEntityRowV1(
            quarantine_id=quarantine_evidence_sha256(evidence),
            entity_kind=entry.spec.kind,
            entity_schema_version=entry.spec.schema_version,
            entity_length=len(evidence),
            entity_bytes=evidence,
            receiver_instance_id=self._instance.receiver_instance_id,
            quarantined_at_monotonic_us=self._clock.now_monotonic_us(),
            database_schema_version=DATABASE_SCHEMA_VERSION,
            failure_reason=failure.reason,
            failure_operation=failure.operation,
            sqlite_primary_code=failure.sqlite_primary_code,
            sqlite_extended_code=failure.sqlite_extended_code,
            os_errno=failure.os_errno,
            isolation_attempt_count=self._isolation_attempts[index],
        )

    def _quarantine_attempt(self, index: int) -> PersistenceAttemptResult:
        may_have_committed = self._unknown
        commit_started = 0
        try:
            if self._needs_validation:
                self._revalidate()
            failure = self._database.inspect_storage(
                minimum_free_bytes=self._minimum_free_bytes
            )
            if failure is not None:
                raise StorageUnavailable(failure)
            intended = self._quarantine[index]
            self._transactions.begin(self._database.connection)
            existing = self._repository.find_quarantined_entity(intended.quarantine_id)
            if existing is None:
                self._repository.insert_quarantined_entity(intended)
            elif not exact_sql_row(
                existing, rows.quarantined_entity_row_v1_parameters(intended)
            ):
                raise PersistenceIdentityCollision("quarantine identity collision")
            commit_started = self._clock.now_monotonic_us()
            may_have_committed = True
            self._transactions.commit(self._database.connection)
        except (
            sqlite3.Error,
            OSError,
            StorageUnavailable,
            PersistenceIdentityCollision,
            TypeError,
            ValueError,
            OverflowError,
            AttributeError,
        ) as error:
            self._increment(durable_quarantine_failures=1)
            failure = classify_global_failure(error)
            if not isinstance(
                error,
                (
                    sqlite3.Error,
                    OSError,
                    StorageUnavailable,
                    PersistenceIdentityCollision,
                ),
            ):
                failure = DatabaseFailure(State.UNAVAILABLE_INCOMPATIBLE_SCHEMA)
            failure, _ = self._rollback_failure(failure)
            if may_have_committed:
                self._unknown_indices = (index,)
                self._unknown_quarantine = True
            return self._fail_global(failure, unknown=may_have_committed, retain=True)
        return self._finish(
            (index,),
            quarantine=True,
            duration=self._clock.now_monotonic_us() - commit_started,
        )

    def attempt(self, *, max_entities: int) -> PersistenceAttemptResult | None:
        """Run at most one transaction, or one rolled-back isolation boundary.

        Each completed FIFO prefix is removed immediately. None means idle,
        not yet due, or awaiting operator recovery.
        """
        if type(max_entities) is not int or max_entities < 1:
            raise ValueError("max_entities must be positive")
        if not self._recovery_due():
            return None
        if not self._claim(max_entities):
            return None
        self._operator_recovery_requested = False
        pending = tuple(range(len(self._tokens)))
        indices = (
            self._unknown_indices
            if self._unknown
            else pending[:1] if self._isolating else pending
        )
        if self._unknown_quarantine or indices[0] in self._quarantine:
            return self._quarantine_attempt(indices[0])
        was_unknown = self._unknown
        commit_may_have_run = False
        entity_operation = False
        operation = Operation.INITIALIZE
        index = indices[0]
        isolated = self._isolating and len(indices) == 1 and not was_unknown
        if isolated:
            self._isolation_attempts[index] = self._isolation_attempts.get(index, 0) + 1
        self._increment(batch_transaction_attempts=1)
        try:
            if self._needs_validation:
                self._revalidate()
            # Only selected health work is sampled before BEGIN. Reading work
            # follows FIFO SQL dependencies and is never reconstructed on retry.
            for index in indices:
                entry = self._lease.entries[index]
                if (
                    self._prepared[index] is None
                    and entry.spec.kind is Kind.RECEIVER_HEALTH_REQUEST
                ):
                    entity_operation = True
                    operation = Operation.VALIDATE
                    self._prepared[index] = self._prepare(entry.entity, entry.spec.kind)
            entity_operation = False
            failure = self._database.inspect_storage(
                minimum_free_bytes=self._minimum_free_bytes
            )
            if failure is not None:
                raise StorageUnavailable(failure)
            self._transactions.begin(self._database.connection)
            for index in indices:
                entry = self._lease.entries[index]
                entity_operation = True
                operation = Operation.DECODE
                work = self._prepared[index]
                if work is None:
                    work = self._prepare(entry.entity, entry.spec.kind)
                    self._prepared[index] = work
                operation = Operation.APPEND
                self._write(work)
            entity_operation = False
            operation = Operation.SYNC
            commit_started = self._clock.now_monotonic_us()
            commit_may_have_run = True
            self._transactions.commit(self._database.connection)
        except (
            sqlite3.Error,
            OSError,
            TypeError,
            ValueError,
            OverflowError,
            AttributeError,
            PersistenceIdentityCollision,
            StorageUnavailable,
        ) as error:
            self._increment(batch_transaction_failures=1)
            failure = classify_global_failure(error)
            candidate = (
                classify_entity_failure(error, operation)
                if entity_operation and not was_unknown
                else None
            )
            failure, rolled_back = self._rollback_failure(
                failure, item_failure=candidate is not None
            )
            if candidate is not None and rolled_back:
                preceding = self._suspected.get(index)
                self._suspected[index] = candidate
                self._isolating = True
                self._needs_validation = True
                if isolated and preceding == candidate:
                    if self._lease.entries[index].spec.kind is Kind.CLOCK_OBSERVATION:
                        return self._fail_global(
                            DatabaseFailure(State.UNAVAILABLE_INCOMPATIBLE_SCHEMA),
                            unknown=False,
                            retain=True,
                        )
                    try:
                        self._freeze_quarantine(index, candidate)
                    except (
                        QuarantineEvidenceError,
                        ValueError,
                        TypeError,
                        OverflowError,
                    ):
                        self._increment(durable_quarantine_failures=1)
                        return self._fail_global(
                            DatabaseFailure(State.UNAVAILABLE_INCOMPATIBLE_SCHEMA),
                            unknown=False,
                            retain=True,
                        )
                elif isolated and preceding is not None:
                    # A changing failure has not reproduced. Pace further
                    # investigation without labeling it poison or spinning.
                    return self._fail_global(
                        DatabaseFailure(State.UNAVAILABLE_IO),
                        unknown=False,
                        retain=True,
                    )
                return PersistenceAttemptResult(
                    OrdinaryBatchCommitOutcome.NOT_COMMITTED
                )
            unknown = was_unknown or commit_may_have_run
            if unknown:
                self._unknown_indices = indices
            return self._fail_global(
                failure,
                unknown=unknown,
                retain=self._isolating
                or isinstance(error, PersistenceIdentityCollision),
            )
        return self._finish(
            indices,
            quarantine=False,
            duration=self._clock.now_monotonic_us() - commit_started,
        )

    def close(self) -> None:
        """Close SQLite without acknowledging pending volatile work."""
        self._database.close()
