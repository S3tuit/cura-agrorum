"""Persistence prerequisites for one process instance, without a worker or replay."""

from __future__ import annotations

import sqlite3
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from uuid import RFC_4122, UUID, uuid4

from .generated.receiver_enums_generated import PersistenceAdmissionState
from .ports.clocks import MonotonicClock
from .persistence_startup import StartupStage
from .receiver_configuration import (
    ReceiverConfigurationLoadResult,
    ReceiverConfigurationLoadStatus,
    ReceiverConfigurationReader,
)
from .sqlite_database import (
    DatabaseFailure,
    ReceiverDatabase,
    database_failure,
    open_receiver_database,
)


@dataclass(frozen=True, slots=True)
class ReceiverInstanceStart:
    receiver_instance_id: bytes
    started_at_monotonic_us: int

    def __post_init__(self) -> None:
        if (
            type(self.receiver_instance_id) is not bytes
            or len(self.receiver_instance_id) != 16
        ):
            raise ValueError("receiver_instance_id must be exactly 16 UUIDv4 bytes")
        identity = UUID(bytes=self.receiver_instance_id)
        if identity.version != 4 or identity.variant != RFC_4122:
            raise ValueError("receiver_instance_id must be UUIDv4")
        if (
            type(self.started_at_monotonic_us) is not int
            or not 0 <= self.started_at_monotonic_us <= (1 << 63) - 1
        ):
            raise ValueError("start monotonic time must be an integer in 0..INT64_MAX")


def create_receiver_instance(clock: MonotonicClock) -> ReceiverInstanceStart:
    """Call once at process start, before handing the immutable value to owners."""

    identity = uuid4().bytes
    return ReceiverInstanceStart(identity, clock.now_monotonic_us())


class ReceiverInstanceStartDisposition(Enum):
    STARTED = auto()
    NOT_STARTED = auto()
    OUTCOME_UNKNOWN = auto()


@dataclass(frozen=True, slots=True)
class ReceiverInstanceStartResult:
    disposition: ReceiverInstanceStartDisposition
    instance_ordinal: int | None = None
    failure: DatabaseFailure | None = None


def insert_receiver_instance_start(
    connection: sqlite3.Connection,
    instance: ReceiverInstanceStart,
    linux_boot_id: bytes,
    *,
    failure_observer=None,
) -> ReceiverInstanceStartResult:
    """Commit exactly one start row using its own transaction, never retry it.

    The caller must use a validated production connection and must not start
    ordinary admission before this returns STARTED. An uncertain result needs
    later reconciliation; it is not permission to reinsert or to admit work.
    After a failed start the caller must close/discard the connection; rollback
    is best effort and may leave a transaction open if cleanup also fails.
    """

    if type(instance) is not ReceiverInstanceStart:
        raise TypeError("instance must be a ReceiverInstanceStart")
    if type(linux_boot_id) is not bytes or len(linux_boot_id) != 16:
        raise ValueError("linux_boot_id must contain exactly 16 bytes")
    if connection.in_transaction:
        raise ValueError("receiver-instance startup requires its own transaction")
    commit_may_have_run = False
    try:
        connection.execute("BEGIN IMMEDIATE")
        cursor = connection.execute(
            "INSERT INTO receiver_instances "
            "(receiver_instance_id, linux_boot_id, started_at_monotonic_us) "
            "VALUES (?, ?, ?)",
            (
                instance.receiver_instance_id,
                linux_boot_id,
                instance.started_at_monotonic_us,
            ),
        )
        ordinal = cursor.lastrowid
        commit_may_have_run = True
        connection.execute("COMMIT")
        return ReceiverInstanceStartResult(
            ReceiverInstanceStartDisposition.STARTED, ordinal
        )
    except (sqlite3.Error, OSError) as exc:
        failure = database_failure(exc)
        if not commit_may_have_run and (
            isinstance(exc, sqlite3.IntegrityError)
            or failure.sqlite_primary_code
            in (sqlite3.SQLITE_ERROR, sqlite3.SQLITE_SCHEMA)
        ):
            failure = DatabaseFailure(
                PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA,
                failure.sqlite_primary_code,
                failure.sqlite_extended_code,
                failure.os_errno,
            )
        disposition = (
            ReceiverInstanceStartDisposition.OUTCOME_UNKNOWN
            if commit_may_have_run
            else ReceiverInstanceStartDisposition.NOT_STARTED
        )
        result = ReceiverInstanceStartResult(disposition, failure=failure)
        if failure_observer is not None:
            failure_observer(result)
        if connection.in_transaction:
            try:
                connection.execute("ROLLBACK")
            except (sqlite3.Error, OSError):
                # Cleanup cannot replace the classified startup failure. The
                # owner must discard this connection even if rollback failed.
                pass
        return result
    except BaseException:
        if failure_observer is not None:
            failure_observer(None)
        raise


@dataclass(frozen=True, slots=True)
class ReceiverStartupResult:
    configuration_load: ReceiverConfigurationLoadResult | None
    database_failure: DatabaseFailure | None = None
    instance_start: ReceiverInstanceStartResult | None = None
    database: ReceiverDatabase | None = field(default=None, repr=False)
    unexpected_failure: bool = False

    @property
    def started(self) -> bool:
        """Persistence startup completed; this does not assert radio/TX readiness."""
        return self.database is not None


def start_receiver_instance(
    instance: ReceiverInstanceStart,
    *,
    configuration_reader: ReceiverConfigurationReader,
    database_path: Path,
    minimum_free_bytes: int,
    stage_entered=None,
    failure_observer=None,
) -> ReceiverStartupResult:
    """Run the actual startup prerequisites on the persistence owner's thread.

    The successful caller owns the returned validated database handle. Configuration
    failure stops before database access; database/start failure closes the
    connection without checkpointing or changing admission. State policy,
    initial time observations and radio startup belong to later components.
    """

    if type(instance) is not ReceiverInstanceStart:
        raise TypeError("instance must be a ReceiverInstanceStart")
    configuration = None
    def enter(stage):
        if stage_entered is not None:
            stage_entered(stage)
    def failed(result):
        if failure_observer is not None:
            failure_observer(result)
    def open_failed(failure):
        failed(ReceiverStartupResult(configuration, database_failure=failure,
                                     unexpected_failure=failure is None))
    def instance_failed(result):
        failed(ReceiverStartupResult(configuration, instance_start=result,
                                     unexpected_failure=result is None))
    enter(StartupStage.CONFIGURATION_LOAD)
    try:
        configuration = configuration_reader.read()
    except BaseException:
        failed(ReceiverStartupResult(None, unexpected_failure=True))
        raise
    if configuration.status is not ReceiverConfigurationLoadStatus.LOADED:
        result = ReceiverStartupResult(configuration)
        failed(result)
        return result
    enter(StartupStage.DATABASE_OPEN_VALIDATION)
    opened = open_receiver_database(
        database_path,
        configuration.configuration.group_id,
        minimum_free_bytes=minimum_free_bytes,
        **({} if failure_observer is None else {"failure_observer": open_failed}),
    )
    if opened.database is None:
        return ReceiverStartupResult(configuration, database_failure=opened.failure)
    database = opened.database
    try:
        enter(StartupStage.INSTANCE_COMMIT)
        start = insert_receiver_instance_start(
            database.connection, instance, configuration.linux_boot_id,
            **({} if failure_observer is None else {"failure_observer": instance_failed}),
        )
        if start.disposition is ReceiverInstanceStartDisposition.STARTED:
            result = ReceiverStartupResult(
                configuration, instance_start=start, database=database
            )
            database = None
            return result
        return ReceiverStartupResult(configuration, instance_start=start)
    except BaseException:
        failed(ReceiverStartupResult(configuration, unexpected_failure=True))
        raise
    finally:
        if database is not None:
            try:
                database.close()
            except (sqlite3.Error, OSError):
                # Startup already failed. Retain its bounded result and evidence;
                # cleanup cannot turn it into success or an unstructured error.
                pass
