from __future__ import annotations

import errno
import os
import sqlite3
from pathlib import Path

import pytest

from cura_receiver import sqlite_database
from cura_receiver.database_initializer import initialize_database
from cura_receiver.generated.receiver_enums_generated import (
    DATABASE_SCHEMA_FINGERPRINT,
    DATABASE_SCHEMA_VERSION,
    SQLITE_APPLICATION_ID,
    PersistenceAdmissionState as State,
)
from cura_receiver.sqlite_database import database_failure, open_receiver_database

GROUP = bytes.fromhex("0102030405060708")


# Resolving an alias at open binds later inspection and reopen to that same file.
def test_handle_binds_alias_and_reopens_same_file(tmp_path):
    first = tmp_path / "first.db"
    second = tmp_path / "second.db"
    initialize_database(first, GROUP)
    initialize_database(second, GROUP)
    alias = tmp_path / "alias.db"
    alias.symlink_to(first.name)
    database = open_receiver_database(alias, GROUP, minimum_free_bytes=0).database
    original = database.connection
    try:
        assert database.path == first.resolve()
        assert database.group_id == GROUP
        alias.unlink()
        alias.symlink_to(second.name)
        assert database.inspect_storage(minimum_free_bytes=0) is None
        database.close()
        assert database.revalidate(minimum_free_bytes=0) is None
        assert database.connection is not original
        assert database.connection.execute("PRAGMA database_list").fetchone()[2] == str(
            first
        )
        with pytest.raises(AttributeError):
            database.path = second
    finally:
        database.close()


# A valid replacement must not become the storage of an already active handle.
def test_handle_rejects_replaced_database(tmp_path):
    first = tmp_path / "first.db"
    replacement = tmp_path / "replacement.db"
    initialize_database(first, GROUP)
    initialize_database(replacement, GROUP)
    database = open_receiver_database(first, GROUP, minimum_free_bytes=0).database
    database.close()
    first.rename(tmp_path / "original.db")
    replacement.rename(first)
    before = first.read_bytes()
    for failure in (
        database.inspect_storage(minimum_free_bytes=0),
        database.revalidate(minimum_free_bytes=0),
    ):
        assert failure.admission_state is State.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    assert first.read_bytes() == before
    assert (tmp_path / "original.db").exists()


def _database(tmp_path: Path) -> Path:
    path = tmp_path / "receiver ?# database.sqlite3"
    initialize_database(path, GROUP)
    return path


# A real production connection enforces all required settings without publishing admission.
def test_open_usable_database(tmp_path: Path) -> None:
    path = _database(tmp_path)
    result = open_receiver_database(path, GROUP, minimum_free_bytes=1)
    assert result.failure is None
    connection = result.database.connection
    assert connection is not None
    try:
        for pragma, expected in (
            ("journal_mode", "wal"),
            ("synchronous", 2),
            ("foreign_keys", 1),
            ("busy_timeout", 250),
            ("wal_autocheckpoint", 0),
        ):
            assert connection.execute(f"PRAGMA {pragma}").fetchone() == (expected,)
        assert connection.getconfig(sqlite3.SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE)
        assert connection.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
        assert connection.execute("PRAGMA foreign_key_check").fetchall() == []
        assert connection.execute("SELECT * FROM communicator_state").fetchall() == []
        assert connection.execute("SELECT * FROM receiver_instances").fetchall() == []
        assert not connection.in_transaction
    finally:
        connection.close()


# Real malformed metadata is rejected without converting its journal or rewriting its bytes.
@pytest.mark.parametrize(
    "case",
    (
        "application-id",
        "missing-table",
        "missing-column",
        "missing-row",
        "multiple-rows",
        "singleton",
        "group",
        "newer",
        "older",
        "gapped",
        "fingerprint",
        "type",
        "null",
    ),
)
def test_identity_rejection_preserves_database(tmp_path: Path, case: str) -> None:
    path = _database(tmp_path)
    connection = sqlite3.connect(path, isolation_level=None)
    if case == "application-id":
        connection.execute("PRAGMA application_id = 0")
    else:
        connection.execute("DROP TABLE database_metadata")
        if case != "missing-table":
            if case == "missing-column":
                connection.execute(
                    "CREATE TABLE database_metadata (singleton_id INTEGER)"
                )
            else:
                connection.execute(
                    "CREATE TABLE database_metadata (singleton_id, group_id, database_schema_version, database_schema_fingerprint)"
                )
                row = [1, GROUP, DATABASE_SCHEMA_VERSION, DATABASE_SCHEMA_FINGERPRINT]
                if case == "singleton":
                    row[0] = 2
                elif case == "group":
                    row[1] = bytes(8)
                elif case in {"newer", "older", "gapped"}:
                    row[2] += {"newer": 1, "older": -1, "gapped": 10}[case]
                elif case == "fingerprint":
                    row[3] = bytes(32)
                elif case == "type":
                    row[0] = 1.0
                elif case == "null":
                    row[3] = None
                if case != "missing-row":
                    connection.execute(
                        "INSERT INTO database_metadata VALUES (?, ?, ?, ?)", row
                    )
                if case == "multiple-rows":
                    connection.execute(
                        "INSERT INTO database_metadata VALUES (?, ?, ?, ?)", row
                    )
    connection.close()
    before = path.read_bytes()
    result = open_receiver_database(path, GROUP, minimum_free_bytes=0)
    assert result.database is None
    assert result.failure.admission_state is State.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    assert path.read_bytes() == before
    assert not Path(str(path) + "-wal").exists()
    assert not Path(str(path) + "-shm").exists()
    assert b"group_master_key" not in before


# Rejection of an existing WAL database preserves uncheckpointed evidence on close.
def test_rejected_wal_is_preserved(tmp_path: Path) -> None:
    path = _database(tmp_path)
    connection = sqlite3.connect(path, isolation_level=None)
    connection.setconfig(sqlite3.SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE, True)
    connection.execute("PRAGMA journal_mode = WAL")
    connection.execute(
        "INSERT INTO receiver_instances (receiver_instance_id, linux_boot_id, started_at_monotonic_us) VALUES (?, ?, ?)",
        (bytes(16), bytes(16), 10),
    )
    connection.close()
    wal = Path(str(path) + "-wal")
    shm = Path(str(path) + "-shm")
    before = (path.read_bytes(), wal.read_bytes())
    assert shm.exists()
    identities = [
        (metadata.st_dev, metadata.st_ino, metadata.st_size)
        for metadata in (artifact.stat() for artifact in (path, wal, shm))
    ]
    result = open_receiver_database(path, bytes(8), minimum_free_bytes=0)
    assert result.failure.admission_state is State.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    assert (path.read_bytes(), wal.read_bytes()) == before
    assert shm.exists()
    assert [
        (metadata.st_dev, metadata.st_ino, metadata.st_size)
        for metadata in (artifact.stat() for artifact in (path, wal, shm))
    ] == identities


# Non-SQLite bytes and real page corruption follow the corrupt-artifact path unchanged.
@pytest.mark.parametrize("case", ("not-a-database", "page"))
def test_real_corruption_is_preserved(tmp_path: Path, case: str) -> None:
    path = _database(tmp_path)
    if case == "not-a-database":
        path.write_bytes(b"corrupt receiver evidence")
    else:
        connection = sqlite3.connect(path)
        page = connection.execute(
            "SELECT rootpage FROM sqlite_schema WHERE name='receiver_instances'"
        ).fetchone()[0]
        page_size = connection.execute("PRAGMA page_size").fetchone()[0]
        connection.close()
        with path.open("r+b") as stream:
            stream.seek((page - 1) * page_size)
            stream.write(b"\xff" * 32)
    before = path.read_bytes()
    result = open_receiver_database(path, GROUP, minimum_free_bytes=0)
    assert result.database is None
    assert result.failure.admission_state is State.UNAVAILABLE_CORRUPT
    assert path.read_bytes() == before


# Missing paths and non-file storage never cause implicit runtime database creation.
@pytest.mark.parametrize("case", ("missing", "missing-parent", "directory"))
def test_unusable_database_path(tmp_path: Path, case: str) -> None:
    path = tmp_path if case == "directory" else tmp_path / "missing"
    if case == "missing-parent":
        path /= "receiver.db"
    result = open_receiver_database(path, GROUP, minimum_free_bytes=0)
    assert result.database is None
    assert result.failure.admission_state is State.UNAVAILABLE_IO
    assert list(tmp_path.iterdir()) == []


# The explicit preventive threshold rejects before any SQLite mode changes.
def test_low_space_threshold(tmp_path: Path) -> None:
    path = _database(tmp_path)
    before = path.read_bytes()
    result = open_receiver_database(path, GROUP, minimum_free_bytes=1 << 100)
    assert result.failure.admission_state is State.UNAVAILABLE_LOW_SPACE
    assert path.read_bytes() == before


# An inaccessible required storage path is reported without calling SQLite.
def test_access_denied_before_open(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = _database(tmp_path)
    monkeypatch.setattr(sqlite_database.os, "access", lambda *args, **kwargs: False)
    result = open_receiver_database(path, GROUP, minimum_free_bytes=0)
    assert result.failure.admission_state is State.UNAVAILABLE_IO
    assert result.failure.os_errno == errno.EACCES


# Actual SQLite refusal/readback of a required pragma prevents a usable result.
@pytest.mark.parametrize("pragma", ("journal_mode", "synchronous", "foreign_keys"))
def test_pragma_refusal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, pragma: str
) -> None:
    path = _database(tmp_path)
    real_connect = sqlite3.connect

    class RefusingConnection(sqlite3.Connection):
        def execute(self, sql: str, *args: object, **kwargs: object):
            replacement = {
                "journal_mode": (
                    "PRAGMA journal_mode = WAL",
                    "PRAGMA journal_mode = DELETE",
                ),
                "synchronous": (
                    "PRAGMA synchronous = FULL",
                    "PRAGMA synchronous = NORMAL",
                ),
                "foreign_keys": (
                    "PRAGMA foreign_keys = ON",
                    "PRAGMA foreign_keys = OFF",
                ),
            }[pragma]
            if sql == replacement[0]:
                sql = replacement[1]
            return super().execute(sql, *args, **kwargs)

    def connect(*args: object, **kwargs: object):
        return real_connect(*args, factory=RefusingConnection, **kwargs)

    monkeypatch.setattr(sqlite_database.sqlite3, "connect", connect)
    result = open_receiver_database(path, GROUP, minimum_free_bytes=0)
    assert result.database is None
    assert result.failure.admission_state is State.UNAVAILABLE_IO


# A real competing exclusive lock fails closed with the original bounded-wait SQLite code.
def test_database_lock_failure(tmp_path: Path) -> None:
    path = _database(tmp_path)
    other = sqlite3.connect(path, isolation_level=None)
    other.execute("BEGIN EXCLUSIVE")
    try:
        result = open_receiver_database(path, GROUP, minimum_free_bytes=0)
        assert result.failure.admission_state is State.UNAVAILABLE_IO
        assert result.failure.sqlite_primary_code == sqlite3.SQLITE_BUSY
    finally:
        other.execute("ROLLBACK")
        other.close()


# Every binding-exposed result preserves full/corrupt paths, including corruption-specific IOERR extensions.
@pytest.mark.parametrize(
    "code",
    sorted(
        {
            value
            for name, value in vars(sqlite3).items()
            if name.startswith("SQLITE_")
            and type(value) is int
            and (
                name.startswith(
                    (
                        "SQLITE_IOERR",
                        "SQLITE_BUSY",
                        "SQLITE_LOCKED",
                        "SQLITE_CORRUPT",
                        "SQLITE_CANTOPEN",
                        "SQLITE_READONLY",
                        "SQLITE_CONSTRAINT",
                        "SQLITE_ERROR",
                        "SQLITE_ABORT",
                        "SQLITE_AUTH",
                        "SQLITE_NOTICE",
                        "SQLITE_WARNING",
                        "SQLITE_OK",
                        "SQLITE_ROW",
                        "SQLITE_DONE",
                    )
                )
                or 0 <= value <= 28
            )
        }
    ),
)
def test_startup_sqlite_error_classification(code: int) -> None:
    error = sqlite3.OperationalError("must not enter result")
    error.sqlite_errorcode = code
    failure = database_failure(error)
    expected = (
        State.UNAVAILABLE_DISK_FULL
        if code & 255 == sqlite3.SQLITE_FULL
        else (
            State.UNAVAILABLE_CORRUPT
            if code & 255 in (sqlite3.SQLITE_CORRUPT, sqlite3.SQLITE_NOTADB)
            or code in (sqlite3.SQLITE_IOERR_DATA, sqlite3.SQLITE_IOERR_CORRUPTFS)
            else State.UNAVAILABLE_IO
        )
    )
    assert failure.admission_state is expected
    assert failure.sqlite_primary_code == code & 255
    assert failure.sqlite_extended_code == code
    assert "must not enter result" not in repr(failure)


# Host capacity failure is distinct from permissions and unrecognized failures.
@pytest.mark.parametrize(
    ("error", "expected"),
    (
        (OSError(errno.ENOSPC, "full"), State.UNAVAILABLE_DISK_FULL),
        (OSError(errno.EROFS, "readonly"), State.UNAVAILABLE_IO),
        (OSError(errno.EIO, "I/O"), State.UNAVAILABLE_IO),
        (sqlite3.OperationalError("unknown"), State.UNAVAILABLE_IO),
    ),
)
def test_startup_os_and_unknown_failure(error: Exception, expected: State) -> None:
    failure = database_failure(error)
    assert failure.admission_state is expected
    assert failure.os_errno == (error.errno if isinstance(error, OSError) else None)


# Corrupt startup retains artifacts and database/WAL bytes; SQLite may update SHM bookkeeping.
def test_corrupt_database_keeps_wal_and_shared_memory(tmp_path: Path) -> None:
    path = _database(tmp_path)
    connection = sqlite3.connect(path, isolation_level=None)
    connection.setconfig(sqlite3.SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE, True)
    connection.execute("PRAGMA journal_mode = WAL")
    connection.execute(
        "INSERT INTO receiver_instances (receiver_instance_id, linux_boot_id, started_at_monotonic_us) VALUES (?, ?, ?)",
        (bytes(16), bytes(16), 1),
    )
    connection.close()
    with path.open("r+b") as stream:
        stream.write(b"corrupt header!!")
    wal = Path(str(path) + "-wal")
    shm = Path(str(path) + "-shm")
    before = (path.read_bytes(), wal.read_bytes())
    identities = [
        (metadata.st_dev, metadata.st_ino, metadata.st_size)
        for metadata in (artifact.stat() for artifact in (path, wal, shm))
    ]
    result = open_receiver_database(path, GROUP, minimum_free_bytes=0)
    assert result.failure.admission_state is State.UNAVAILABLE_CORRUPT
    assert (path.read_bytes(), wal.read_bytes()) == before
    assert [
        (metadata.st_dev, metadata.st_ino, metadata.st_size)
        for metadata in (artifact.stat() for artifact in (path, wal, shm))
    ] == identities
