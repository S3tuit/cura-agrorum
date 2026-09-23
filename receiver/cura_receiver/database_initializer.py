"""Create a fresh receiver database from the exact packaged schema."""

from __future__ import annotations

import hashlib
import os
import sqlite3
import stat
import tempfile
from dataclasses import dataclass
from pathlib import Path

from .generated.receiver_enums_generated import (
    DATABASE_SCHEMA_FINGERPRINT,
    DATABASE_SCHEMA_VERSION,
    SQLITE_APPLICATION_ID,
)


PACKAGED_SCHEMA_PATH = Path(__file__).resolve().parents[1] / "db" / "schema.sql"


class DatabaseInitializationError(RuntimeError):
    """The requested fresh database could not be safely initialized."""


class DatabaseInstallationUncertainError(DatabaseInitializationError):
    """The destination link exists, but its directory durability is unknown."""

    def __init__(
        self,
        database_path: Path,
        temporary_path: Path,
        file_identity: tuple[int, int],
    ) -> None:
        super().__init__(
            "receiver database installation is uncertain; preserve and reconcile "
            f"{database_path} with {temporary_path}"
        )
        self.database_path = database_path
        self.temporary_path = temporary_path
        self.file_identity = file_identity


@dataclass(frozen=True, slots=True)
class DatabaseInitializationResult:
    """The durable installation result and any unconfirmed temporary cleanup."""

    database_path: Path
    cleanup_pending_path: Path | None = None

    @property
    def cleanup_complete(self) -> bool:
        return self.cleanup_pending_path is None


def initialize_database(
    database_path: str | os.PathLike[str],
    group_id: bytes,
    *,
    schema_path: str | os.PathLike[str] = PACKAGED_SCHEMA_PATH,
) -> DatabaseInitializationResult:
    """Create and atomically install one new receiver database.

    The destination must not already exist. ``schema_path`` is injectable for
    verification tests, but its exact bytes must always match the generated
    schema fingerprint before SQLite sees them. The result distinguishes clean
    installation from non-fatal temporary-name cleanup still pending.
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
    linked = False
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

        temporary_stat = temporary_path.stat(follow_symlinks=False)
        try:
            os.link(temporary_path, destination)
        except FileExistsError:
            raise FileExistsError(
                f"receiver database already exists: {destination}"
            ) from None
        linked = True
        try:
            _synchronize_directory(parent)
        except OSError as exc:
            raise DatabaseInstallationUncertainError(
                destination,
                temporary_path,
                (temporary_stat.st_dev, temporary_stat.st_ino),
            ) from exc
        return _finish_installed_database(destination, temporary_path, parent)
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
        if not linked:
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


def reconcile_database_installation(
    uncertain: DatabaseInstallationUncertainError,
    group_id: bytes,
) -> DatabaseInitializationResult:
    """Validate and durably finish one installation reported as uncertain."""

    if type(group_id) is not bytes or len(group_id) != 8:
        raise ValueError("group_id must contain exactly 8 bytes")
    destination = uncertain.database_path
    temporary_path = uncertain.temporary_path
    if destination.parent.resolve(strict=True) != temporary_path.parent.resolve(
        strict=True
    ):
        raise DatabaseInitializationError(
            "uncertain installation paths do not share one directory"
        )
    for path in (destination, temporary_path):
        try:
            path_stat = path.stat(follow_symlinks=False)
        except OSError as exc:
            raise DatabaseInitializationError(
                "uncertain installation artifact is unavailable"
            ) from exc
        if stat.S_ISLNK(path_stat.st_mode) or not stat.S_ISREG(path_stat.st_mode):
            raise DatabaseInitializationError(
                "uncertain installation artifact is not a regular file"
            )
        if (path_stat.st_dev, path_stat.st_ino) != uncertain.file_identity:
            raise DatabaseInitializationError(
                "uncertain installation artifact identity changed"
            )

    _validate_database_for_reconciliation(destination, group_id)
    try:
        _synchronize_directory(destination.parent)
    except OSError as exc:
        raise DatabaseInstallationUncertainError(
            destination,
            temporary_path,
            uncertain.file_identity,
        ) from exc
    return _finish_installed_database(
        destination,
        temporary_path,
        destination.parent,
    )


def _validate_database_for_reconciliation(
    database_path: Path,
    group_id: bytes,
) -> None:
    connection: sqlite3.Connection | None = None
    try:
        database_uri = database_path.resolve(strict=True).as_uri() + "?mode=ro"
        connection = sqlite3.connect(database_uri, uri=True, isolation_level=None)
        connection.execute("PRAGMA foreign_keys = ON")
        if connection.execute("PRAGMA foreign_keys").fetchone() != (1,):
            raise DatabaseInitializationError("SQLite refused foreign-key enforcement")
        if connection.execute("PRAGMA application_id").fetchone() != (
            SQLITE_APPLICATION_ID,
        ):
            raise DatabaseInitializationError(
                "uncertain database has the wrong SQLite application ID"
            )
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
            raise DatabaseInitializationError(
                "uncertain database metadata does not match the requested installation"
            )
        if connection.execute("PRAGMA foreign_key_check").fetchall():
            raise DatabaseInitializationError(
                "uncertain database has foreign-key violations"
            )
        if connection.execute("PRAGMA integrity_check").fetchall() != [("ok",)]:
            raise DatabaseInitializationError(
                "uncertain database failed its integrity check"
            )
    except DatabaseInitializationError:
        raise
    except (OSError, sqlite3.Error) as exc:
        raise DatabaseInitializationError(
            "could not validate uncertain receiver database installation"
        ) from exc
    finally:
        if connection is not None:
            connection.close()


def _finish_installed_database(
    destination: Path,
    temporary_path: Path,
    parent: Path,
) -> DatabaseInitializationResult:
    try:
        temporary_path.unlink()
    except FileNotFoundError:
        pass
    except OSError:
        return DatabaseInitializationResult(destination, temporary_path)
    try:
        _synchronize_directory(parent)
    except OSError:
        return DatabaseInitializationResult(destination, temporary_path)
    return DatabaseInitializationResult(destination)


def _synchronize_directory(directory: Path) -> None:
    directory_descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory_descriptor)
    finally:
        try:
            os.close(directory_descriptor)
        except OSError:
            # A successful directory fsync already established the durability
            # contract. A close error must not recast it as installation failure.
            pass
