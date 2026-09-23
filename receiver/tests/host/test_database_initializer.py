from __future__ import annotations

import hashlib
import sqlite3
from pathlib import Path

import pytest

from cura_receiver import database_initializer
from cura_receiver.database_initializer import (
    DatabaseInitializationError,
    DatabaseInstallationUncertainError,
    initialize_database,
    reconcile_database_installation,
)
from cura_receiver.generated.receiver_enums_generated import (
    DATABASE_SCHEMA_FINGERPRINT,
    DATABASE_SCHEMA_VERSION,
    SQLITE_APPLICATION_ID,
)


RECEIVER_ROOT = Path(__file__).resolve().parents[2]
SCHEMA = RECEIVER_ROOT / "db" / "schema.sql"


# Installs the exact packaged schema, metadata, catalogues, and immutability rules.
def test_initializer_installs_exact_packaged_schema_and_metadata(
    tmp_path: Path,
) -> None:
    database = tmp_path / "receiver.sqlite3"
    group_id = bytes.fromhex("0011223344556677")

    result = initialize_database(database, group_id)

    assert result.database_path == database
    assert result.cleanup_complete
    assert result.cleanup_pending_path is None
    assert database.is_file()
    assert not list(tmp_path.glob(".receiver.sqlite3.*.initializing"))
    connection = sqlite3.connect(database)
    connection.execute("PRAGMA foreign_keys = ON")
    assert connection.execute("PRAGMA application_id").fetchone() == (
        SQLITE_APPLICATION_ID,
    )
    assert connection.execute("PRAGMA foreign_keys").fetchone() == (1,)
    assert connection.execute(
        "SELECT singleton_id, group_id, database_schema_version, "
        "database_schema_fingerprint FROM database_metadata"
    ).fetchall() == [
        (1, group_id, DATABASE_SCHEMA_VERSION, DATABASE_SCHEMA_FINGERPRINT)
    ]
    assert connection.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
    assert connection.execute("PRAGMA foreign_key_check").fetchall() == []
    tables = {
        row[0]
        for row in connection.execute(
            "SELECT name FROM sqlite_schema "
            "WHERE type = 'table' AND name NOT LIKE 'sqlite_%'"
        )
    }
    assert {
        row[1]: row[5]
        for row in connection.execute("PRAGMA table_list")
        if row[1] in tables
    } == {table: 1 for table in tables}
    assert connection.execute(
        "SELECT id, code FROM rtc_health_codes ORDER BY id"
    ).fetchall() == [(1, "PRESENT"), (2, "MISSING"), (3, "INVALID")]
    assert {
        row[1] for row in connection.execute("PRAGMA table_info(database_metadata)")
    } == {
        "singleton_id",
        "group_id",
        "database_schema_version",
        "database_schema_fingerprint",
    }
    for statement in (
        "INSERT INTO rtc_health_codes VALUES (4, 'OTHER')",
        "UPDATE rtc_health_codes SET code = 'OTHER' WHERE id = 1",
        "DELETE FROM rtc_health_codes WHERE id = 1",
        "INSERT INTO database_metadata VALUES (1, X'0000000000000000', 1, zeroblob(32))",
        "UPDATE database_metadata SET database_schema_version = 1",
        "DELETE FROM database_metadata",
    ):
        with pytest.raises(sqlite3.IntegrityError):
            connection.execute(statement)
    connection.close()


# Rejects every noncanonical receiver-group identity before creating a file.
@pytest.mark.parametrize("group_id", (b"", bytes(7), bytes(9), bytearray(8)))
def test_initializer_rejects_invalid_group_identity(
    tmp_path: Path,
    group_id: object,
) -> None:
    database = tmp_path / "receiver.sqlite3"
    with pytest.raises(ValueError, match="group_id"):
        initialize_database(database, group_id)  # type: ignore[arg-type]
    assert not database.exists()


# Rejects schema bytes whose digest differs from the generated contract.
def test_initializer_rejects_schema_bytes_that_do_not_match_generated_hash(
    tmp_path: Path,
) -> None:
    tampered_schema = tmp_path / "schema.sql"
    tampered_schema.write_bytes(SCHEMA.read_bytes() + b"\n-- tampered\n")
    database = tmp_path / "receiver.sqlite3"

    with pytest.raises(DatabaseInitializationError, match="fingerprint"):
        initialize_database(database, bytes(8), schema_path=tampered_schema)

    assert not database.exists()
    assert not list(tmp_path.glob(".receiver.sqlite3.*.initializing"))


# Preserves an existing destination byte for byte.
def test_initializer_never_overwrites_an_existing_destination(
    tmp_path: Path,
) -> None:
    database = tmp_path / "receiver.sqlite3"
    database.write_bytes(b"existing receiver evidence")
    before = hashlib.sha256(database.read_bytes()).digest()

    with pytest.raises(FileExistsError):
        initialize_database(database, bytes(8))

    assert hashlib.sha256(database.read_bytes()).digest() == before


# Removes initialization artifacts after a definite SQLite failure.
def test_initializer_cleans_temporary_artifacts_after_sqlite_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    database = tmp_path / "receiver.sqlite3"

    def fail_connect(*args: object, **kwargs: object) -> sqlite3.Connection:
        raise sqlite3.OperationalError("injected open failure")

    monkeypatch.setattr(database_initializer.sqlite3, "connect", fail_connect)
    with pytest.raises(DatabaseInitializationError, match="initialization failed"):
        initialize_database(database, bytes(8))

    assert not database.exists()
    assert not list(tmp_path.glob(".receiver.sqlite3.*.initializing"))


# Preserves and exactly reconciles an installation with uncertain durability.
def test_initializer_preserves_and_reconciles_uncertain_installation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    database = tmp_path / "receiver.sqlite3"
    group_id = bytes.fromhex("0011223344556677")
    synchronize_directory = database_initializer._synchronize_directory

    def fail_directory_sync(_directory: Path) -> None:
        raise OSError("injected directory sync failure")

    monkeypatch.setattr(
        database_initializer,
        "_synchronize_directory",
        fail_directory_sync,
    )
    with pytest.raises(DatabaseInstallationUncertainError) as captured:
        initialize_database(database, group_id)

    uncertain = captured.value
    assert uncertain.database_path == database
    assert database.is_file()
    assert uncertain.temporary_path.is_file()
    assert database.samefile(uncertain.temporary_path)

    with pytest.raises(DatabaseInitializationError, match="metadata"):
        reconcile_database_installation(uncertain, bytes([1]) * 8)
    assert database.samefile(uncertain.temporary_path)

    monkeypatch.setattr(
        database_initializer,
        "_synchronize_directory",
        synchronize_directory,
    )
    result = reconcile_database_installation(uncertain, group_id)
    assert result.database_path == database
    assert result.cleanup_complete
    assert database.is_file()
    assert not uncertain.temporary_path.exists()


# Reports a linked temporary artifact that could not be removed.
def test_initializer_reports_temporary_unlink_as_cleanup_pending(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    database = tmp_path / "receiver.sqlite3"
    path_unlink = Path.unlink

    def fail_temporary_unlink(path: Path, *args: object, **kwargs: object) -> None:
        if path.name.endswith(".initializing"):
            raise OSError("injected temporary unlink failure")
        path_unlink(path, *args, **kwargs)

    monkeypatch.setattr(Path, "unlink", fail_temporary_unlink)
    result = initialize_database(database, bytes(8))

    assert database.is_file()
    assert not result.cleanup_complete
    assert result.cleanup_pending_path is not None
    assert result.cleanup_pending_path.is_file()
    assert database.samefile(result.cleanup_pending_path)


# Rejects reconciliation after the preserved artifact identity changes.
def test_reconciliation_rejects_a_replaced_temporary_artifact(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    database = tmp_path / "receiver.sqlite3"

    def fail_directory_sync(_directory: Path) -> None:
        raise OSError("injected directory sync failure")

    monkeypatch.setattr(
        database_initializer,
        "_synchronize_directory",
        fail_directory_sync,
    )
    with pytest.raises(DatabaseInstallationUncertainError) as captured:
        initialize_database(database, bytes(8))

    uncertain = captured.value
    uncertain.temporary_path.unlink()
    uncertain.temporary_path.write_bytes(b"replacement")
    with pytest.raises(DatabaseInitializationError, match="identity changed"):
        reconcile_database_installation(uncertain, bytes(8))

    assert database.is_file()


# Keeps a durable installation successful while reporting cleanup sync failure.
def test_initializer_reports_cleanup_sync_failure_without_failing_installation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    database = tmp_path / "receiver.sqlite3"
    synchronize_directory = database_initializer._synchronize_directory
    call_count = 0

    def fail_second_directory_sync(directory: Path) -> None:
        nonlocal call_count
        call_count += 1
        if call_count == 2:
            raise OSError("injected cleanup sync failure")
        synchronize_directory(directory)

    monkeypatch.setattr(
        database_initializer,
        "_synchronize_directory",
        fail_second_directory_sync,
    )
    result = initialize_database(database, bytes(8))

    assert call_count == 2
    assert database.is_file()
    assert not result.cleanup_complete
    assert result.cleanup_pending_path is not None
    assert not result.cleanup_pending_path.exists()
