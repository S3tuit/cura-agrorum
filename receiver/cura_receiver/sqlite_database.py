"""Open an existing receiver database without initialization or recovery policy."""

from __future__ import annotations

import errno
import os
import sqlite3
import stat
from dataclasses import dataclass, field
from pathlib import Path

from .generated.receiver_enums_generated import (
    DATABASE_SCHEMA_FINGERPRINT,
    DATABASE_SCHEMA_VERSION,
    SQLITE_APPLICATION_ID,
    PersistenceAdmissionState,
)

SQLITE_BUSY_TIMEOUT_MS = 250


@dataclass(frozen=True, slots=True)
class DatabaseFailure:
    """Bounded startup failure evidence; never an exception or SQL error string."""

    admission_state: PersistenceAdmissionState
    sqlite_primary_code: int | None = None
    sqlite_extended_code: int | None = None
    os_errno: int | None = None


@dataclass(frozen=True, slots=True)
class DatabaseOpenResult:
    """A usable owner-thread connection is not yet permission for admission."""

    connection: sqlite3.Connection | None = field(default=None, repr=False)
    failure: DatabaseFailure | None = None


class _ValidationFailure(Exception):
    def __init__(self, state: PersistenceAdmissionState) -> None:
        self.state = state


def database_failure(error: sqlite3.Error | OSError) -> DatabaseFailure:
    """Classify global startup/SQL failures, without poison or retry policy."""

    extended = getattr(error, "sqlite_errorcode", None)
    if type(extended) is not int:
        extended = None
    primary = None if extended is None else extended & 0xFF
    os_errno = error.errno if isinstance(error, OSError) else None
    if primary == sqlite3.SQLITE_FULL or os_errno == errno.ENOSPC:
        state = PersistenceAdmissionState.UNAVAILABLE_DISK_FULL
    elif primary in (sqlite3.SQLITE_CORRUPT, sqlite3.SQLITE_NOTADB):
        state = PersistenceAdmissionState.UNAVAILABLE_CORRUPT
    else:
        state = PersistenceAdmissionState.UNAVAILABLE_IO
    return DatabaseFailure(state, primary, extended, os_errno)


def _connect(path: Path, *, mode: str) -> sqlite3.Connection:
    # URI quoting prevents '?' or '#' in a deployment path becoming URI options.
    connection = sqlite3.connect(
        path.as_uri() + f"?mode={mode}",
        uri=True,
        timeout=SQLITE_BUSY_TIMEOUT_MS / 1000,
        isolation_level=None,
    )
    try:
        # Closing a rejected WAL database must not checkpoint/delete its evidence.
        connection.setconfig(sqlite3.SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE, True)
        if not connection.getconfig(sqlite3.SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE):
            raise _ValidationFailure(PersistenceAdmissionState.UNAVAILABLE_IO)
        connection.execute("PRAGMA synchronous = FULL")
        connection.execute("PRAGMA foreign_keys = ON")
        connection.execute("PRAGMA wal_autocheckpoint = 0")
        if (
            connection.execute("PRAGMA synchronous").fetchone() != (2,)
            or connection.execute("PRAGMA foreign_keys").fetchone() != (1,)
            or connection.execute("PRAGMA busy_timeout").fetchone()
            != (SQLITE_BUSY_TIMEOUT_MS,)
            or connection.execute("PRAGMA wal_autocheckpoint").fetchone() != (0,)
        ):
            raise _ValidationFailure(PersistenceAdmissionState.UNAVAILABLE_IO)
        return connection
    except BaseException:
        connection.close()
        raise


def _validate_identity(connection: sqlite3.Connection, group_id: bytes) -> None:
    incompatible = PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    if connection.execute("PRAGMA application_id").fetchone() != (
        SQLITE_APPLICATION_ID,
    ):
        raise _ValidationFailure(incompatible)
    try:
        rows = connection.execute(
            "SELECT singleton_id, group_id, database_schema_version, "
            "database_schema_fingerprint FROM database_metadata"
        ).fetchmany(2)
    except sqlite3.Error as exc:
        # Missing/malformed metadata relations are compatibility failures. Never
        # reinterpret corruption, I/O or contention as a missing metadata table.
        if getattr(exc, "sqlite_errorcode", None) in (
            sqlite3.SQLITE_ERROR,
            sqlite3.SQLITE_SCHEMA,
        ):
            raise _ValidationFailure(incompatible) from None
        raise
    expected = (1, group_id, DATABASE_SCHEMA_VERSION, DATABASE_SCHEMA_FINGERPRINT)
    if (
        len(rows) != 1
        or rows[0] != expected
        or any(
            type(value) is not type(wanted) for value, wanted in zip(rows[0], expected)
        )
    ):
        raise _ValidationFailure(incompatible)


def _validate_integrity(connection: sqlite3.Connection) -> None:
    if connection.execute("PRAGMA integrity_check").fetchmany(2) != [("ok",)]:
        raise _ValidationFailure(PersistenceAdmissionState.UNAVAILABLE_CORRUPT)
    if connection.execute("PRAGMA foreign_key_check").fetchone() is not None:
        raise _ValidationFailure(PersistenceAdmissionState.UNAVAILABLE_CORRUPT)


def open_receiver_database(
    path: Path,
    group_id: bytes,
    *,
    minimum_free_bytes: int,
) -> DatabaseOpenResult:
    """Validate, then open without create; all returned I/O stays on this thread.

    Requires Python's SQLite ``setconfig`` support (Python 3.12 or newer) and
    NO_CKPT_ON_CLOSE. The caller owns close/checkpoint decisions and admission.
    Zero explicitly disables the caller's preventive free-space threshold.
    """

    if not isinstance(path, Path):
        raise TypeError("database path must be a Path")
    if type(group_id) is not bytes or len(group_id) != 8:
        raise ValueError("group_id must contain exactly 8 bytes")
    if type(minimum_free_bytes) is not int or minimum_free_bytes < 0:
        raise ValueError("minimum_free_bytes must be a non-negative integer")
    connection = None
    try:
        destination = path.resolve(strict=True)
        if not stat.S_ISREG(destination.stat().st_mode):
            raise OSError(errno.EINVAL, "database is not a regular file")
        filesystem = os.statvfs(destination.parent)
        if filesystem.f_flag & os.ST_RDONLY:
            raise OSError(errno.EROFS, "database filesystem is read-only")
        if not os.access(
            destination, os.R_OK | os.W_OK, effective_ids=True
        ) or not os.access(
            destination.parent,
            os.W_OK | os.X_OK,
            effective_ids=True,
        ):
            raise OSError(errno.EACCES, "database storage is not writable")
        if filesystem.f_bavail * filesystem.f_frsize < minimum_free_bytes:
            raise _ValidationFailure(PersistenceAdmissionState.UNAVAILABLE_LOW_SPACE)

        connection = _connect(destination, mode="ro")
        _validate_identity(connection, group_id)
        _validate_integrity(connection)
        connection.close()
        connection = None

        connection = _connect(destination, mode="rw")
        # Recheck the actual write connection; never trust a previous path open.
        _validate_identity(connection, group_id)
        _validate_integrity(connection)
        if connection.execute("PRAGMA journal_mode = WAL").fetchone() != ("wal",):
            raise _ValidationFailure(PersistenceAdmissionState.UNAVAILABLE_IO)
        if connection.execute("PRAGMA synchronous").fetchone() != (2,):
            raise _ValidationFailure(PersistenceAdmissionState.UNAVAILABLE_IO)
        result = DatabaseOpenResult(connection=connection)
        connection = None
        return result
    except _ValidationFailure as exc:
        failure = DatabaseFailure(exc.state)
    except (sqlite3.Error, OSError) as exc:
        failure = database_failure(exc)
    finally:
        # The successful result transfers connection ownership to the caller.
        if connection is not None:
            try:
                connection.close()
            except sqlite3.Error:
                pass  # Preserve the original validation failure; never delete files.
    return DatabaseOpenResult(failure=failure)
