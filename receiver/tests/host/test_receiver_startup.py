from __future__ import annotations

import json
import os
import select
import sqlite3
import subprocess
import sys
from pathlib import Path
from uuid import UUID

import pytest

from cura_receiver import receiver_startup, sqlite_database
from cura_receiver.database_initializer import initialize_database
from cura_receiver.generated.receiver_enums_generated import PersistenceAdmissionState
from cura_receiver.receiver_configuration import (
    ReceiverConfigurationReader,
    ReceiverConfigurationLoadStatus,
)
from cura_receiver.receiver_startup import (
    ReceiverInstanceStart,
    ReceiverInstanceStartDisposition as Disposition,
    create_receiver_instance,
    insert_receiver_instance_start,
    start_receiver_instance,
)
from cura_receiver.sqlite_database import open_receiver_database
from tests.support.fakes.os_clock import FakeOsClock

GROUP = bytes.fromhex("0102030405060708")
INSTANCE = bytes.fromhex("00112233445546778899aabbccddeeff")
BOOT = bytes.fromhex("11223344556677889900aabbccddeeff")


def _connection(tmp_path: Path) -> tuple[Path, sqlite3.Connection]:
    path = tmp_path / "receiver.db"
    initialize_database(path, GROUP)
    result = open_receiver_database(path, GROUP, minimum_free_bytes=0)
    assert result.failure is None
    return path, result.connection


def _reader(tmp_path: Path) -> ReceiverConfigurationReader:
    config = tmp_path / "test-group.json"
    config.write_text(
        json.dumps(
            {
                "format_version": 1,
                "group_id": GROUP.hex(),
                "group_master_key": "00" * 32,
                "active_node_ids": [],
                "retired_node_ids": [],
            }
        ),
        encoding="utf-8",
    )
    config.chmod(0o600)
    boot_path = tmp_path / "test-boot-id"
    boot_path.write_text(str(UUID(bytes=BOOT)) + "\n", encoding="ascii")
    return ReceiverConfigurationReader(config, boot_id_path=boot_path)


# Instance identity is fresh UUIDv4 while its creation timestamp uses the injected clock.
def test_create_new_instance() -> None:
    clock = FakeOsClock(monotonic_us=123)
    first = create_receiver_instance(clock)
    clock.advance_elapsed_us(7)
    second = create_receiver_instance(clock)
    assert first.receiver_instance_id != second.receiver_instance_id
    assert (
        UUID(bytes=first.receiver_instance_id).version
        == UUID(bytes=second.receiver_instance_id).version
        == 4
    )
    assert (first.started_at_monotonic_us, second.started_at_monotonic_us) == (123, 130)


# Durable ordinals order clean, unclean and simulated reboot starts without a false marker.
def test_durable_start_order_and_boot_mapping(tmp_path: Path) -> None:
    path, connection = _connection(tmp_path)
    try:
        instances = [
            create_receiver_instance(FakeOsClock(monotonic_us=t)) for t in (10, 20, 1)
        ]
        boots = [BOOT, BOOT, bytes(16)]
        for ordinal, (instance, boot) in enumerate(zip(instances, boots), 1):
            result = insert_receiver_instance_start(connection, instance, boot)
            assert result.disposition is Disposition.STARTED
            assert result.instance_ordinal == ordinal
            assert result.failure is None
        with sqlite3.connect(path) as observer:
            rows = observer.execute(
                "SELECT * FROM receiver_instances ORDER BY instance_ordinal"
            ).fetchall()
        assert rows == [
            (
                i,
                instance.receiver_instance_id,
                boot,
                instance.started_at_monotonic_us,
                None,
                None,
            )
            for i, (instance, boot) in enumerate(zip(instances, boots), 1)
        ]
        assert not connection.in_transaction
    finally:
        connection.close()


# Reusing even an identical start identity is a collision, never a second insertion or replay.
def test_start_collision_preserves_original(tmp_path: Path) -> None:
    path, connection = _connection(tmp_path)
    instance = ReceiverInstanceStart(INSTANCE, 10)
    try:
        assert (
            insert_receiver_instance_start(connection, instance, BOOT).disposition
            is Disposition.STARTED
        )
        result = insert_receiver_instance_start(connection, instance, BOOT)
        assert result.disposition is Disposition.NOT_STARTED
        assert (
            result.failure.admission_state
            is PersistenceAdmissionState.UNAVAILABLE_INCOMPATIBLE_SCHEMA
        )
        assert result.instance_ordinal is None
        assert connection.execute("SELECT * FROM receiver_instances").fetchall() == [
            (1, INSTANCE, BOOT, 10, None, None)
        ]
    finally:
        connection.close()


# Invalid public identities and SQLite-bound start times fail before any SQL operation.
@pytest.mark.parametrize(
    ("identity", "timestamp"),
    ((bytes(16), 1), (INSTANCE, True), (INSTANCE, -1), (INSTANCE, 1 << 63), (b"", 1)),
)
def test_invalid_instance_start(identity: bytes, timestamp: object) -> None:
    with pytest.raises(ValueError):
        ReceiverInstanceStart(identity, timestamp)


# The raw Linux boot identity must retain its exact 16-byte representation.
@pytest.mark.parametrize(
    "boot", (b"", bytes(15), bytearray(16), "11223344-5566-7788-9900-aabbccddeeff")
)
def test_invalid_boot_prevents_start(tmp_path: Path, boot: object) -> None:
    _, connection = _connection(tmp_path)
    try:
        with pytest.raises(ValueError):
            insert_receiver_instance_start(
                connection, ReceiverInstanceStart(INSTANCE, 0), boot
            )
        assert connection.execute("SELECT * FROM receiver_instances").fetchall() == []
    finally:
        connection.close()


# Start cannot accidentally join, commit or roll back somebody else's transaction.
def test_start_refuses_existing_transaction(tmp_path: Path) -> None:
    _, connection = _connection(tmp_path)
    try:
        connection.execute("BEGIN")
        with pytest.raises(ValueError, match="own transaction"):
            insert_receiver_instance_start(
                connection, ReceiverInstanceStart(INSTANCE, 0), BOOT
            )
        assert connection.in_transaction
    finally:
        connection.close()


# Failures around the real commit boundary distinguish definite absence from uncertainty.
@pytest.mark.parametrize(
    ("boundary", "expected", "durable_rows"),
    (
        ("insert", Disposition.NOT_STARTED, 0),
        ("before-commit", Disposition.OUTCOME_UNKNOWN, 0),
        ("after-commit", Disposition.OUTCOME_UNKNOWN, 1),
    ),
)
def test_start_commit_certainty(
    tmp_path: Path, boundary: str, expected: Disposition, durable_rows: int
) -> None:
    path = tmp_path / "receiver.db"
    initialize_database(path, GROUP)

    class FailingConnection(sqlite3.Connection):
        def execute(self, sql: str, *args: object, **kwargs: object):
            if (
                boundary == "insert"
                and sql.startswith("INSERT INTO receiver_instances")
            ) or (boundary == "before-commit" and sql == "COMMIT"):
                raise sqlite3.OperationalError("injected startup failure")
            cursor = super().execute(sql, *args, **kwargs)
            if boundary == "after-commit" and sql == "COMMIT":
                raise sqlite3.OperationalError("injected lost commit result")
            return cursor

    connection = sqlite3.connect(path, isolation_level=None, factory=FailingConnection)
    try:
        result = insert_receiver_instance_start(
            connection, ReceiverInstanceStart(INSTANCE, 10), BOOT
        )
        assert result.disposition is expected
        assert result.instance_ordinal is None
        assert not connection.in_transaction
        observer = sqlite3.connect(path)
        try:
            assert observer.execute(
                "SELECT count(*) FROM receiver_instances"
            ).fetchone() == (durable_rows,)
        finally:
            observer.close()
    finally:
        connection.close()


# The startup composition returns a usable connection only after the start row is durable.
def test_startup_composition(tmp_path: Path) -> None:
    path = tmp_path / "receiver.db"
    initialize_database(path, GROUP)
    result = start_receiver_instance(
        ReceiverInstanceStart(INSTANCE, 17),
        configuration_reader=_reader(tmp_path),
        database_path=path,
        minimum_free_bytes=0,
    )
    assert result.started
    try:
        assert (
            result.configuration_load.status is ReceiverConfigurationLoadStatus.LOADED
        )
        assert result.instance_start.disposition is Disposition.STARTED
        assert result.instance_start.instance_ordinal == 1
        observer = sqlite3.connect(path)
        try:
            assert observer.execute("SELECT * FROM receiver_instances").fetchall() == [
                (1, INSTANCE, BOOT, 17, None, None)
            ]
        finally:
            observer.close()
    finally:
        result.connection.close()


# Configuration failure prevents database access; missing database prevents any start operation.
@pytest.mark.parametrize("failure_stage", ("configuration", "database"))
def test_startup_failure_order(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, failure_stage: str
) -> None:
    reader = _reader(tmp_path)
    if failure_stage == "configuration":
        (tmp_path / "test-group.json").chmod(0o644)

    def forbidden(*args: object, **kwargs: object) -> None:
        pytest.fail("startup continued past a failed prerequisite")

    monkeypatch.setattr(receiver_startup, "insert_receiver_instance_start", forbidden)
    if failure_stage == "configuration":
        monkeypatch.setattr(receiver_startup, "open_receiver_database", forbidden)
    path = tmp_path / "missing.db"
    result = start_receiver_instance(
        ReceiverInstanceStart(INSTANCE, 0),
        configuration_reader=reader,
        database_path=path,
        minimum_free_bytes=0,
    )
    assert not result.started
    assert result.instance_start is None
    assert not path.exists()


# Killing after the named durable-start boundary preserves the row without cleanup hooks.
def test_unclean_process_exit_preserves_start(tmp_path: Path) -> None:
    path = tmp_path / "receiver.db"
    initialize_database(path, GROUP)
    receiver_root = Path(__file__).resolve().parents[2]
    protocol_root = receiver_root.parent / "protocol/protocol-v2-lora/python"
    child_code = """
import sys
from pathlib import Path
from cura_receiver.receiver_startup import ReceiverInstanceStart, insert_receiver_instance_start, ReceiverInstanceStartDisposition
from cura_receiver.sqlite_database import open_receiver_database
opened = open_receiver_database(Path(sys.argv[1]), bytes.fromhex('0102030405060708'), minimum_free_bytes=0)
assert opened.failure is None
result = insert_receiver_instance_start(opened.connection, ReceiverInstanceStart(bytes.fromhex('00112233445546778899aabbccddeeff'), 12), bytes(16))
assert result.disposition is ReceiverInstanceStartDisposition.STARTED
print('durable-start', flush=True)
sys.stdin.buffer.read(1)
"""
    child = subprocess.Popen(
        [sys.executable, "-c", child_code, str(path)],
        env={
            **os.environ,
            "PYTHONPATH": os.pathsep.join((str(receiver_root), str(protocol_root))),
        },
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        ready, _, _ = select.select([child.stdout], [], [], 5)
        assert ready, "child failed to reach durable-start boundary"
        assert child.stdout.readline() == b"durable-start\n"
        child.kill()
        child.wait(timeout=5)
        assert child.returncode < 0
    finally:
        if child.poll() is None:
            child.kill()
        child.communicate(timeout=5)
    assert Path(str(path) + "-wal").exists()
    assert Path(str(path) + "-shm").exists()
    opened = open_receiver_database(path, GROUP, minimum_free_bytes=0)
    assert opened.failure is None
    try:
        assert opened.connection.execute(
            "SELECT * FROM receiver_instances"
        ).fetchall() == [(1, INSTANCE, bytes(16), 12, None, None)]
    finally:
        opened.connection.close()


# The last SQLite-compatible monotonic start value is durable without conversion to REAL.
def test_maximum_start_monotonic_value(tmp_path: Path) -> None:
    _, connection = _connection(tmp_path)
    try:
        result = insert_receiver_instance_start(
            connection, ReceiverInstanceStart(INSTANCE, (1 << 63) - 1), BOOT
        )
        assert result.disposition is Disposition.STARTED
        assert connection.execute(
            "SELECT started_at_monotonic_us, typeof(started_at_monotonic_us) FROM receiver_instances"
        ).fetchone() == ((1 << 63) - 1, "integer")
    finally:
        connection.close()


# A later close error cannot replace a definite, bounded startup failure with an exception.
def test_failed_start_preserves_result_when_close_raises(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "receiver.db"
    initialize_database(path, GROUP)
    real_connect = sqlite3.connect

    class FailingConnection(sqlite3.Connection):
        failed_start = False

        def execute(self, sql: str, *args: object, **kwargs: object):
            if sql.startswith("INSERT INTO receiver_instances"):
                self.failed_start = True
                error = sqlite3.OperationalError("injected capacity failure")
                error.sqlite_errorcode = sqlite3.SQLITE_FULL
                raise error
            return super().execute(sql, *args, **kwargs)

        def close(self) -> None:
            super().close()
            if self.failed_start:
                raise sqlite3.OperationalError("injected close failure")

    def connect(*args: object, **kwargs: object):
        return real_connect(*args, factory=FailingConnection, **kwargs)

    monkeypatch.setattr(sqlite_database.sqlite3, "connect", connect)
    result = start_receiver_instance(
        ReceiverInstanceStart(INSTANCE, 12),
        configuration_reader=_reader(tmp_path),
        database_path=path,
        minimum_free_bytes=0,
    )
    assert not result.started
    assert result.instance_start.disposition is Disposition.NOT_STARTED
    assert (
        result.instance_start.failure.admission_state
        is PersistenceAdmissionState.UNAVAILABLE_DISK_FULL
    )
    assert result.instance_start.failure.sqlite_primary_code == sqlite3.SQLITE_FULL
    assert "injected" not in repr(result)
    observer = real_connect(path)
    try:
        assert observer.execute("SELECT * FROM receiver_instances").fetchall() == []
    finally:
        observer.close()
