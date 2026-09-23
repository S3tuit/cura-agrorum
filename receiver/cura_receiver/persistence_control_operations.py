"""Caller-driven SQLite control operations on the persistence owner's handle."""

from __future__ import annotations

import hashlib
import sqlite3
from threading import current_thread

from .communicator_state_persistence import (
    CommunicatorStatePolicy,
    classify_communicator_state_rows,
    validate_communicator_state,
)
from .generated.receiver_entities_generated import communicator_state_v1_parameters
from .generated.receiver_enums_generated import (
    DATABASE_SCHEMA_VERSION,
    DiagnosticOperation as Op,
)
from .persistence_control_execution import ControlCommand, ControlDeadlineExceeded
from .persistence_control_values import (
    CommunicatorStateCommitDisposition as StateDisposition,
    CommunicatorStateCommitFailureKind as StateFailure,
    CommunicatorStateCommitResult as StateResult,
    CommunicatorStateCondition as Condition,
    CommunicatorStateLoadResult as LoadResult,
    CommunicatorStateLoadStatus as LoadStatus,
    ReceiverCleanStopV1,
    ReceiverCleanStopCommitDisposition as StopDisposition,
    ReceiverCleanStopCommitFailureKind as StopFailure,
    ReceiverCleanStopCommitResult as StopResult,
)
from .persistence_failures import classify_global_failure
from .receiver_configuration import PersistenceControlInterfaceViolation as Violation
from .sqlite_database import (
    ReceiverDatabase,
    StorageUnavailable,
    SQLITE_BUSY_TIMEOUT_MS,
)
from .sqlite_repository import SqliteRepository
from .sqlite_transactions import SqliteTransactions


class PersistenceControlOperations:
    """Explicit transaction owner; no mailbox, policy mutation or automatic recovery."""

    def __init__(
        self,
        database: ReceiverDatabase,
        *,
        instance,
        queue,
        clock,
        policy: CommunicatorStatePolicy,
        transactions=None,
        minimum_free_bytes=0,
    ):
        if type(database) is not ReceiverDatabase:
            raise TypeError("control operations require a validated ReceiverDatabase")
        self.database = database
        self.instance = instance
        self.queue = queue
        self.clock = clock
        self.policy = policy
        self.transactions = (
            transactions if transactions is not None else SqliteTransactions()
        )
        self.minimum_free_bytes = minimum_free_bytes
        self.owner = current_thread()
        self.last_failure = None

    def _ready(self, command):
        command.start()
        command.check()
        if self.database.connection.in_transaction:
            raise RuntimeError("control dispatch requires a safe transaction boundary")
        failure = self.database.inspect_storage(
            minimum_free_bytes=self.minimum_free_bytes
        )
        if failure is not None:
            raise StorageUnavailable(failure)
        command.check(self.database.connection)

    def _repository(self):
        return SqliteRepository(self.database.connection)

    def _restore_wait(self):
        try:
            self.database.connection.execute(
                f"PRAGMA busy_timeout = {SQLITE_BUSY_TIMEOUT_MS}"
            )
        except (sqlite3.Error, OSError) as error:
            if self.last_failure is None:
                self._database_failure(error)

    def _database_failure(self, error):
        failure = classify_global_failure(error)
        try:
            self.transactions.rollback(self.database.connection)
        except (sqlite3.Error, OSError):
            # Preserve the primary classification and discard a possibly open transaction.
            try:
                self.database.close()
            except (sqlite3.Error, OSError):
                pass
        self.last_failure = failure
        return dict(
            sqlite_primary_code=failure.sqlite_primary_code,
            sqlite_extended_code=failure.sqlite_extended_code,
            os_errno=failure.os_errno,
        )

    def load_state(self, command: ControlCommand):
        self.last_failure = None
        if current_thread() is not self.owner:
            return LoadResult(
                LoadStatus.INTERFACE_VIOLATION,
                Op.READ,
                interface_violation=Violation.WRONG_CALLER,
            )
        try:
            self._ready(command)
            repository = self._repository()
            raw = repository.read_communicator_state_rows()
            command.check(self.database.connection)
            result = classify_communicator_state_rows(raw, repository, self.policy)
            command.check()
            return result
        except ControlDeadlineExceeded:
            return LoadResult(LoadStatus.DEADLINE_EXCEEDED, Op.READ)
        except (sqlite3.Error, OSError, StorageUnavailable) as error:
            return LoadResult(
                LoadStatus.DATABASE_ERROR, Op.READ, **self._database_failure(error)
            )
        finally:
            self._restore_wait()

    def _reject_stop(self, violation):
        return StopResult(
            StopDisposition.NOT_COMMITTED,
            StopFailure.INTERFACE_VIOLATION,
            Op.CLEANUP,
            interface_violation=violation,
        )

    def commit_clean_stop(self, marker, command: ControlCommand):
        self.last_failure = None
        if current_thread() is not self.owner:
            return self._reject_stop(Violation.WRONG_CALLER)
        commit_may_have_run = False
        try:
            self._ready(command)
            if type(marker) is not ReceiverCleanStopV1:
                return self._reject_stop(Violation.INVALID_ARGUMENT)
            try:
                marker.__post_init__()
            except (TypeError, ValueError):
                return self._reject_stop(Violation.INVALID_ARGUMENT)
            if (
                marker.receiver_instance_id != self.instance.receiver_instance_id
                or marker.stopped_at_monotonic_us
                < self.instance.started_at_monotonic_us
                or not self.queue.snapshot().closed_and_drained
            ):
                return self._reject_stop(Violation.CLEAN_STOP_PRECONDITION)
            command.check(self.database.connection)
            self.transactions.begin(self.database.connection)
            repository = self._repository()
            current = repository.find_receiver_instance(marker.receiver_instance_id)
            if current is None or current[3] != self.instance.started_at_monotonic_us:
                self.transactions.rollback(self.database.connection)
                return self._reject_stop(Violation.CLEAN_STOP_PRECONDITION)
            intended = (
                marker.stopped_at_monotonic_us,
                marker.communicator_state_generation,
            )
            if current[4:] != (None, None):
                self.transactions.rollback(self.database.connection)
                command.check()
                if current[4:] == intended:
                    return StopResult(
                        StopDisposition.ALREADY_COMMITTED, StopFailure.NONE, Op.NONE
                    )
                return self._reject_stop(Violation.CLEAN_STOP_CONFLICT)
            command.check(self.database.connection)
            installed = classify_communicator_state_rows(
                repository.read_communicator_state_rows(), repository, self.policy
            )
            generation = (
                installed.state.generation
                if installed.status is LoadStatus.LOADED
                else 0
            )
            if generation != marker.communicator_state_generation:
                self.transactions.rollback(self.database.connection)
                return self._reject_stop(Violation.CLEAN_STOP_PRECONDITION)
            command.check(self.database.connection)
            self.database.connection.execute(
                "UPDATE receiver_instances SET clean_stopped_at_monotonic_us = ?, "
                "clean_stop_state_generation = ? WHERE receiver_instance_id = ?",
                (*intended, marker.receiver_instance_id),
            )
            command.before_commit(self.database.connection)
            commit_may_have_run = True
            self.transactions.commit(self.database.connection)
            return StopResult(StopDisposition.COMMITTED, StopFailure.NONE, Op.NONE)
        except ControlDeadlineExceeded:
            try:
                self.transactions.rollback(self.database.connection)
            except (sqlite3.Error, OSError) as error:
                self._database_failure(error)
            return StopResult(
                StopDisposition.NOT_COMMITTED, StopFailure.DEADLINE_EXCEEDED, Op.CLEANUP
            )
        except (sqlite3.Error, OSError, StorageUnavailable) as error:
            return StopResult(
                StopDisposition.OUTCOME_UNKNOWN
                if commit_may_have_run
                else StopDisposition.NOT_COMMITTED,
                StopFailure.DATABASE_ERROR,
                Op.CLEANUP,
                **self._database_failure(error),
            )
        finally:
            self._restore_wait()

    def _reject_state(self, violation):
        return StateResult(
            StateDisposition.NOT_INSTALLED,
            StateFailure.INTERFACE_VIOLATION,
            Op.WRITE,
            interface_violation=violation,
        )

    def _validate_recovery(self, state, condition):
        if state.generation != 1:
            return False
        charges = tuple(bucket.charged_airtime_us for bucket in state.buckets)
        if condition in (Condition.UNSUPPORTED_VERSION, Condition.POLICY_MISMATCH):
            # The continuous no-TX wait is a caller invariant, independent of UTC.
            return not any(charges)
        q, r = divmod(state.tx_airtime_budget_us, state.bucket_charge_limit_us)
        newest = ((r,) if r else ()) + (state.bucket_charge_limit_us,) * q
        return charges == (0,) * (len(charges) - len(newest)) + newest

    def _archive(self, raw, command):
        observed_at = self.clock.now_monotonic_us()
        for row in raw:
            command.check(self.database.connection)
            archived = self.database.connection.execute(
                "INSERT INTO quarantined_communicator_states "
                "(observed_singleton_id, observed_state_format_version, observed_generation, "
                "observed_state_blob, observed_state_sha256, calculated_blob_sha256, "
                "preserved_by_receiver_instance_id, preserved_at_monotonic_us, database_schema_version) "
                "SELECT singleton_id, state_format_version, generation, state_blob, state_sha256, "
                "?, ?, ?, ? FROM communicator_state WHERE rowid = ?",
                (
                    hashlib.sha256(row.values[3]).digest()
                    if type(row.values[3]) is bytes
                    else None,
                    self.instance.receiver_instance_id,
                    observed_at,
                    DATABASE_SCHEMA_VERSION,
                    row.row_id,
                ),
            )
            if archived.rowcount != 1:
                raise sqlite3.IntegrityError(
                    "observed communicator state row was not archived"
                )

    def commit_state(self, state, command: ControlCommand):
        self.last_failure = None
        if current_thread() is not self.owner:
            return self._reject_state(Violation.WRONG_CALLER)
        commit_may_have_run = False
        try:
            self._ready(command)
            repository = self._repository()
            try:
                blob = validate_communicator_state(state, repository, self.policy)
            except (TypeError, ValueError, OverflowError):
                return self._reject_state(Violation.INVALID_STATE)
            command.check(self.database.connection)
            self.transactions.begin(self.database.connection)
            command.check(self.database.connection)
            raw = repository.read_communicator_state_rows()
            installed = classify_communicator_state_rows(raw, repository, self.policy)
            if installed.status is LoadStatus.LOADED:
                generation = installed.state.generation
                if state.generation == generation:
                    self.transactions.rollback(self.database.connection)
                    command.check()
                    if blob == raw[0].values[3]:
                        return StateResult(
                            StateDisposition.ALREADY_COMMITTED,
                            StateFailure.NONE,
                            Op.NONE,
                        )
                    return self._reject_state(Violation.GENERATION_CONTENT_CONFLICT)
                if state.generation != generation + 1:
                    self.transactions.rollback(self.database.connection)
                    return self._reject_state(
                        Violation.STALE_GENERATION
                        if state.generation < generation
                        else Violation.GENERATION_GAP
                    )
            else:
                if not self._validate_recovery(state, installed.state_condition):
                    self.transactions.rollback(self.database.connection)
                    if installed.state_condition in (
                        Condition.UNSUPPORTED_VERSION,
                        Condition.POLICY_MISMATCH,
                    ):
                        return StateResult(
                            StateDisposition.NOT_INSTALLED,
                            StateFailure.STATE_UNAVAILABLE,
                            Op.WRITE,
                            state_condition=installed.state_condition,
                        )
                    return self._reject_state(Violation.INVALID_STATE)
                self._archive(raw, command)
            command.check(self.database.connection)
            self.database.connection.execute("DELETE FROM communicator_state")
            command.check(self.database.connection)
            self.database.connection.execute(
                "INSERT INTO communicator_state VALUES (?, ?, ?, ?, ?)",
                communicator_state_v1_parameters(state),
            )
            command.before_commit(self.database.connection)
            commit_may_have_run = True
            self.transactions.commit(self.database.connection)
            return StateResult(StateDisposition.COMMITTED, StateFailure.NONE, Op.NONE)
        except ControlDeadlineExceeded:
            try:
                self.transactions.rollback(self.database.connection)
            except (sqlite3.Error, OSError) as error:
                self._database_failure(error)
            return StateResult(
                StateDisposition.NOT_INSTALLED, StateFailure.DEADLINE_EXCEEDED, Op.WRITE
            )
        except (sqlite3.Error, OSError, StorageUnavailable) as error:
            return StateResult(
                StateDisposition.OUTCOME_UNKNOWN
                if commit_may_have_run
                else StateDisposition.NOT_INSTALLED,
                StateFailure.DATABASE_ERROR,
                Op.WRITE,
                **self._database_failure(error),
            )
        finally:
            self._restore_wait()
