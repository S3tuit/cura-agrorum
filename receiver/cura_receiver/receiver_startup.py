"""Persistence prerequisites for one process instance, without a worker or replay."""

from __future__ import annotations

import sqlite3
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from uuid import RFC_4122, UUID, uuid4

from .generated.receiver_enums_generated import PersistenceAdmissionState
from .ports.clocks import MonotonicClock
from .receiver_configuration import (
    ReceiverConfigurationLoadResult,
    ReceiverConfigurationLoadStatus,
    ReceiverConfigurationReader,
)
from .sqlite_database import DatabaseFailure, database_failure, open_receiver_database


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
        if connection.in_transaction:
            try:
                connection.execute("ROLLBACK")
            except (sqlite3.Error, OSError):
                # Cleanup cannot replace the classified startup failure. The
                # owner must discard this connection even if rollback failed.
                pass
        disposition = (
            ReceiverInstanceStartDisposition.OUTCOME_UNKNOWN
            if commit_may_have_run
            else ReceiverInstanceStartDisposition.NOT_STARTED
        )
        return ReceiverInstanceStartResult(disposition, failure=failure)


@dataclass(frozen=True, slots=True)
class ReceiverStartupResult:
    configuration_load: ReceiverConfigurationLoadResult
    database_failure: DatabaseFailure | None = None
    instance_start: ReceiverInstanceStartResult | None = None
    connection: sqlite3.Connection | None = field(default=None, repr=False)

    @property
    def started(self) -> bool:
        """Persistence startup completed; this does not assert radio/TX readiness."""
        return self.connection is not None


def start_receiver_instance(
    instance: ReceiverInstanceStart,
    *,
    configuration_reader: ReceiverConfigurationReader,
    database_path: Path,
    minimum_free_bytes: int,
) -> ReceiverStartupResult:
    """Run the actual startup prerequisites on the persistence owner's thread.

    The successful caller owns the returned SQLite connection. Configuration
    failure stops before database access; database/start failure closes the
    connection without checkpointing or changing admission. State policy,
    initial time observations and radio startup belong to later components.
    """

    if type(instance) is not ReceiverInstanceStart:
        raise TypeError("instance must be a ReceiverInstanceStart")
    configuration = configuration_reader.read()
    if configuration.status is not ReceiverConfigurationLoadStatus.LOADED:
        return ReceiverStartupResult(configuration)
    opened = open_receiver_database(
        database_path,
        configuration.configuration.group_id,
        minimum_free_bytes=minimum_free_bytes,
    )
    if opened.connection is None:
        return ReceiverStartupResult(configuration, database_failure=opened.failure)
    connection = opened.connection
    try:
        start = insert_receiver_instance_start(
            connection, instance, configuration.linux_boot_id
        )
        if start.disposition is ReceiverInstanceStartDisposition.STARTED:
            result = ReceiverStartupResult(
                configuration, instance_start=start, connection=connection
            )
            connection = None
            return result
        return ReceiverStartupResult(configuration, instance_start=start)
    finally:
        if connection is not None:
            try:
                connection.close()
            except (sqlite3.Error, OSError):
                # Startup already failed. Retain its bounded result and evidence;
                # cleanup cannot turn it into success or an unstructured error.
                pass
