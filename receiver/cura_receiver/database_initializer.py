"""Create a fresh receiver database from the exact packaged schema."""

from __future__ import annotations

import hashlib
import os
import sqlite3
import tempfile
from pathlib import Path

from .generated.receiver_enums_generated import (
    DATABASE_SCHEMA_FINGERPRINT,
    DATABASE_SCHEMA_VERSION,
    SQLITE_APPLICATION_ID,
)


PACKAGED_SCHEMA_PATH = Path(__file__).resolve().parents[1] / "db" / "schema.sql"


class DatabaseInitializationError(RuntimeError):
    """The requested fresh database could not be safely initialized."""


def initialize_database(
    database_path: str | os.PathLike[str],
    group_id: bytes,
    *,
    schema_path: str | os.PathLike[str] = PACKAGED_SCHEMA_PATH,
) -> None:
    """Create and atomically install one new receiver database.

    The destination must not already exist. ``schema_path`` is injectable for
    verification tests, but its exact bytes must always match the generated
    schema fingerprint before SQLite sees them.
    """

    if type(group_id) is not bytes or len(group_id) != 8:
        raise ValueError("group_id must contain exactly 8 bytes")

    destination = Path(database_path)
    parent = destination.parent
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(f"receiver database already exists: {destination}")
    if not parent.is_dir():
        raise FileNotFoundError(f"receiver database parent does not exist: {parent}")

    try:
        schema_bytes = Path(schema_path).read_bytes()
    except OSError as exc:
        raise DatabaseInitializationError("could not read packaged receiver schema") from exc
    if hashlib.sha256(schema_bytes).digest() != DATABASE_SCHEMA_FINGERPRINT:
        raise DatabaseInitializationError(
            "packaged receiver schema does not match the generated fingerprint"
        )
    try:
        schema_sql = schema_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise DatabaseInitializationError("packaged receiver schema is not UTF-8") from exc

    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.",
        suffix=".initializing",
        dir=parent,
    )
    os.close(file_descriptor)
    temporary_path = Path(temporary_name)
    connection: sqlite3.Connection | None = None
    installed = False
    try:
        connection = sqlite3.connect(temporary_path, isolation_level=None)
        connection.execute("PRAGMA foreign_keys = ON")
        if connection.execute("PRAGMA foreign_keys").fetchone() != (1,):
            raise DatabaseInitializationError("SQLite refused foreign-key enforcement")
        connection.execute("PRAGMA synchronous = FULL")
        connection.executescript(
            "BEGIN IMMEDIATE;\n"
            f"PRAGMA application_id = {SQLITE_APPLICATION_ID};\n"
            + schema_sql
        )
        connection.execute(
            "INSERT INTO database_metadata "
            "(singleton_id, group_id, database_schema_version, "
            "database_schema_fingerprint) VALUES (?, ?, ?, ?)",
            (
                1,
                group_id,
                DATABASE_SCHEMA_VERSION,
                DATABASE_SCHEMA_FINGERPRINT,
            ),
        )
        if connection.execute("PRAGMA application_id").fetchone() != (
            SQLITE_APPLICATION_ID,
        ):
            raise DatabaseInitializationError("SQLite application ID was not installed")
        metadata = connection.execute(
            "SELECT singleton_id, group_id, database_schema_version, "
            "database_schema_fingerprint FROM database_metadata"
        ).fetchall()
        if metadata != [
            (
                1,
                group_id,
                DATABASE_SCHEMA_VERSION,
                DATABASE_SCHEMA_FINGERPRINT,
            )
        ]:
            raise DatabaseInitializationError("database metadata was not installed exactly")
        if connection.execute("PRAGMA foreign_key_check").fetchall():
            raise DatabaseInitializationError("fresh database has foreign-key violations")
        if connection.execute("PRAGMA integrity_check").fetchall() != [("ok",)]:
            raise DatabaseInitializationError("fresh database failed its integrity check")
        connection.execute("COMMIT")
        connection.close()
        connection = None

        try:
            os.link(temporary_path, destination)
        except FileExistsError:
            raise FileExistsError(
                f"receiver database already exists: {destination}"
            ) from None
        installed = True
        temporary_path.unlink()
        directory_descriptor = os.open(parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    except (DatabaseInitializationError, FileExistsError):
        raise
    except (OSError, sqlite3.Error) as exc:
        raise DatabaseInitializationError("receiver database initialization failed") from exc
    finally:
        if connection is not None:
            try:
                if connection.in_transaction:
                    connection.execute("ROLLBACK")
            except sqlite3.Error:
                pass
            connection.close()
        if not installed:
            for artifact in (
                temporary_path,
                Path(str(temporary_path) + "-journal"),
                Path(str(temporary_path) + "-wal"),
                Path(str(temporary_path) + "-shm"),
            ):
                try:
                    artifact.unlink()
                except FileNotFoundError:
                    pass
