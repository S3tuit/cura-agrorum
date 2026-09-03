from __future__ import annotations

import hashlib
import sqlite3
from pathlib import Path

import pytest

from receiver.cura_receiver import database_initializer
from receiver.cura_receiver.database_initializer import (
    DatabaseInitializationError,
    initialize_database,
)
from receiver.cura_receiver.generated.receiver_enums_generated import (
    DATABASE_SCHEMA_FINGERPRINT,
    DATABASE_SCHEMA_VERSION,
    SQLITE_APPLICATION_ID,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA = REPO_ROOT / "receiver" / "db" / "schema.sql"


def test_initializer_installs_exact_packaged_schema_and_metadata(
    tmp_path: Path,
) -> None:
    database = tmp_path / "receiver.sqlite3"
    group_id = bytes.fromhex("0011223344556677")

    initialize_database(database, group_id)

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


@pytest.mark.parametrize("group_id", (b"", bytes(7), bytes(9), bytearray(8)))
def test_initializer_rejects_invalid_group_identity(
    tmp_path: Path,
    group_id: object,
) -> None:
    database = tmp_path / "receiver.sqlite3"
    with pytest.raises(ValueError, match="group_id"):
        initialize_database(database, group_id)  # type: ignore[arg-type]
    assert not database.exists()


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


def test_initializer_never_overwrites_an_existing_destination(
    tmp_path: Path,
) -> None:
    database = tmp_path / "receiver.sqlite3"
    database.write_bytes(b"existing receiver evidence")
    before = hashlib.sha256(database.read_bytes()).digest()

    with pytest.raises(FileExistsError):
        initialize_database(database, bytes(8))

    assert hashlib.sha256(database.read_bytes()).digest() == before


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
