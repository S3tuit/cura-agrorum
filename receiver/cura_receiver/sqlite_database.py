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
    """Bounded database failure evidence; never an exception or SQL error string."""

    admission_state: PersistenceAdmissionState
    sqlite_primary_code: int | None = None
    sqlite_extended_code: int | None = None
    os_errno: int | None = None


@dataclass(frozen=True, slots=True)
class DatabaseOpenResult:
    """A validated owner-thread handle is not yet permission for admission."""

    database: ReceiverDatabase | None = field(default=None, repr=False)
    failure: DatabaseFailure | None = None


class ReceiverDatabase:
    """Concrete storage ownership created only by open_receiver_database().

    The canonical path, group and file identity stay fixed for this handle's
    lifetime. Reopening the same file is supported; replacing history requires
    stopping the receiver and passing a new handle through startup.
    """

    __slots__ = ("_connection", "_path", "_group_id", "_file_identity")

    def __new__(cls):
        raise TypeError("use open_receiver_database() to obtain a validated handle")

    @property
    def connection(self) -> sqlite3.Connection:
        if self._connection is None:
            raise sqlite3.ProgrammingError("receiver database is closed")
        return self._connection

    @property
    def path(self) -> Path:
        return self._path

    @property
    def group_id(self) -> bytes:
        return self._group_id

    def inspect_storage(self, *, minimum_free_bytes: int) -> DatabaseFailure | None:
        try:
            metadata = self._path.stat()
            if (metadata.st_dev, metadata.st_ino) != self._file_identity:
                return DatabaseFailure(
                    PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
                )
        except OSError as error:
            return database_failure(error)
        return inspect_receiver_storage(
            self._path, minimum_free_bytes=minimum_free_bytes
        )

    def revalidate(self, *, minimum_free_bytes: int) -> DatabaseFailure | None:
        failure = self.inspect_storage(minimum_free_bytes=minimum_free_bytes)
        if failure is not None:
            return failure
        if self._connection is not None:
            failure = validate_receiver_connection(self._connection, self._group_id)
            if failure is None:
                return None
            if failure.admission_state is not PersistenceAdmissionState.UNAVAILABLE_IO:
                return failure
            self.close()
        opened = open_receiver_database(
            self._path, self._group_id, minimum_free_bytes=minimum_free_bytes
        )
        if opened.failure is not None:
            return opened.failure
        replacement = opened.database
        if replacement._file_identity != self._file_identity:
            replacement.close()
            return DatabaseFailure(
                PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
            )
        self._connection = replacement._connection
        replacement._connection = None
        return validate_receiver_connection(self.connection, self._group_id)

    def close(self) -> None:
        if self._connection is not None:
            try:
                self._connection.close()
            finally:
                self._connection = None


class _ValidationFailure(Exception):
    def __init__(self, state: PersistenceAdmissionState) -> None:
        self.state = state


def database_failure(error: sqlite3.Error | OSError) -> DatabaseFailure:
    """Classify global SQLite/host results; see sqlite.org/rescode.html.

    IOERR_DATA is a page-checksum failure and IOERR_CORRUPTFS indicates
    filesystem corruption. Their primary IOERR code must not hide corruption.
    """

    extended = getattr(error, "sqlite_errorcode", None)
    if type(extended) is not int or not -(1 << 31) <= extended < (1 << 31):
        extended = None
    primary = None if extended is None else extended & 0xFF
    os_errno = getattr(error, "errno", None)
    if type(os_errno) is not int or not -(1 << 31) <= os_errno < (1 << 31):
        os_errno = None
    if primary == sqlite3.SQLITE_FULL or os_errno == errno.ENOSPC:
        state = PersistenceAdmissionState.UNAVAILABLE_DISK_FULL
    elif primary in (sqlite3.SQLITE_CORRUPT, sqlite3.SQLITE_NOTADB) or extended in (
        sqlite3.SQLITE_IOERR_DATA,
        sqlite3.SQLITE_IOERR_CORRUPTFS,
    ):
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


def _validate_required_projections(connection: sqlite3.Connection) -> None:
    from .sqlite_repository import REQUIRED_TABLE_PROJECTIONS

    for table, columns in REQUIRED_TABLE_PROJECTIONS:
        try:
            connection.execute(f"SELECT {', '.join(columns)} FROM {table} LIMIT 0")
        except sqlite3.Error as error:
            if getattr(error, "sqlite_errorcode", None) in (
                sqlite3.SQLITE_ERROR,
                sqlite3.SQLITE_SCHEMA,
            ):
                raise _ValidationFailure(
                    PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
                ) from None
            raise


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
        metadata = destination.stat()
        file_identity = (metadata.st_dev, metadata.st_ino)
        if not stat.S_ISREG(metadata.st_mode):
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
        _validate_required_projections(connection)
        connection.close()
        connection = None

        connection = _connect(destination, mode="rw")
        # Recheck the actual write connection; never trust a previous path open.
        _validate_identity(connection, group_id)
        _validate_integrity(connection)
        _validate_required_projections(connection)
        if connection.execute("PRAGMA journal_mode = WAL").fetchone() != ("wal",):
            raise _ValidationFailure(PersistenceAdmissionState.UNAVAILABLE_IO)
        if connection.execute("PRAGMA synchronous").fetchone() != (2,):
            raise _ValidationFailure(PersistenceAdmissionState.UNAVAILABLE_IO)
        actual_path = next(
            Path(filename)
            for _, name, filename in connection.execute("PRAGMA database_list")
            if name == "main"
        )
        metadata = destination.stat()
        if (
            actual_path != destination
            or (metadata.st_dev, metadata.st_ino) != file_identity
        ):
            raise _ValidationFailure(
                PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
            )
        database = object.__new__(ReceiverDatabase)
        database._connection = connection
        database._path = destination
        database._group_id = group_id
        database._file_identity = file_identity
        result = DatabaseOpenResult(database=database)
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


def validate_receiver_connection(
    connection: sqlite3.Connection, group_id: bytes
) -> DatabaseFailure | None:
    """Revalidate an idle owner connection before retained work may recover."""
    if connection.in_transaction:
        raise ValueError("connection validation requires a safe transaction boundary")
    try:
        _validate_identity(connection, group_id)
        _validate_integrity(connection)
        _validate_required_projections(connection)
        if (
            connection.execute("PRAGMA journal_mode").fetchone() != ("wal",)
            or connection.execute("PRAGMA synchronous").fetchone() != (2,)
            or connection.execute("PRAGMA foreign_keys").fetchone() != (1,)
            or connection.execute("PRAGMA busy_timeout").fetchone()
            != (SQLITE_BUSY_TIMEOUT_MS,)
            or connection.execute("PRAGMA wal_autocheckpoint").fetchone() != (0,)
            or not connection.getconfig(sqlite3.SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE)
        ):
            raise _ValidationFailure(PersistenceAdmissionState.UNAVAILABLE_IO)
    except _ValidationFailure as error:
        return DatabaseFailure(error.state)
    except (sqlite3.Error, OSError) as error:
        return database_failure(error)
    return None


class StorageUnavailable(Exception):
    """Internal transport for already classified storage inspection evidence."""

    def __init__(self, failure: DatabaseFailure) -> None:
        super().__init__("storage unavailable")
        self.failure = failure


def inspect_receiver_storage(
    path: Path, *, minimum_free_bytes: int
) -> DatabaseFailure | None:
    """Check preventive capacity and access on the receiver's actual filesystem."""
    try:
        filesystem = os.statvfs(path.parent)
        if filesystem.f_flag & os.ST_RDONLY:
            raise OSError(errno.EROFS, "database filesystem is read-only")
        if not os.access(path, os.R_OK | os.W_OK, effective_ids=True) or not os.access(
            path.parent, os.W_OK | os.X_OK, effective_ids=True
        ):
            raise OSError(errno.EACCES, "database storage is not writable")
        if filesystem.f_bavail * filesystem.f_frsize < minimum_free_bytes:
            return DatabaseFailure(PersistenceAdmissionState.UNAVAILABLE_LOW_SPACE)
    except OSError as error:
        return database_failure(error)
    return None
