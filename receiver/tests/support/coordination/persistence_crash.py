from __future__ import annotations

import json
import os
import pickle
import selectors
import shutil
import signal
import subprocess
import sys
from pathlib import Path

from cura_receiver.database_initializer import initialize_database
from cura_receiver.ordinary_persistence import OrdinaryPersistence
from cura_receiver.persist_queue import PersistQueue, PersistQueueBatchLease
from cura_receiver.persist_queue_entities import MEASUREMENT_PROFILE_V1_SPEC
from cura_receiver.receiver_startup import (
    ReceiverInstanceStart,
    insert_receiver_instance_start,
)
from cura_receiver.sqlite_database import open_receiver_database
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import GROUP, INSTANCE, _measurement
from tests.support.fakes.os_clock import FakeOsClock

BOUNDARIES = (
    "before_transaction",
    "after_profile_write",
    "after_reading_write",
    "before_commit",
    "after_commit",
    "before_acknowledgement",
    "after_acknowledgement",
)
DURABLE_BOUNDARIES = frozenset(BOUNDARIES[4:])


def _owner(path, *, transactions=None):
    database = open_receiver_database(path, GROUP, minimum_free_bytes=0).database
    connection = database.connection
    assert connection is not None
    queue = PersistQueue()
    owner = OrdinaryPersistence(
        database,
        queue,
        instance=ReceiverInstanceStart(INSTANCE, 0),
        clock=FakeOsClock(monotonic_us=100),
        transactions=transactions,
    )
    owner.enable_admission()
    return connection, queue, owner


def _child(path: Path, boundary: str, intended_path: Path):
    # This file is written only by this test's parent, never read by production.
    with intended_path.open("rb") as source:
        intended = pickle.load(source)
    queue = None

    def arrive(name):
        if name == boundary:
            print(
                json.dumps(
                    {"boundary": name, "published": queue.snapshot().published_entities}
                ),
                flush=True,
            )
            # Explicit parent release/termination, no timer chooses the boundary.
            sys.stdin.buffer.read(1)
            raise RuntimeError("crash child unexpectedly released")

    class CrashTransactions(SqliteTransactions):
        def begin(self, connection):
            arrive("before_transaction")
            super().begin(connection)

        def commit(self, connection):
            arrive("before_commit")
            super().commit(connection)
            arrive("after_commit")

    connection, queue, owner = _owner(path, transactions=CrashTransactions())
    connection.create_function("crash_boundary", 1, arrive)
    for table, name in (
        ("message_profiles", "after_profile_write"),
        ("reading_messages", "after_reading_write"),
    ):
        connection.execute(
            f"CREATE TEMP TRIGGER crash_{table} AFTER INSERT ON {table} BEGIN SELECT crash_boundary('{name}'); END"
        )
    original_acknowledge = PersistQueueBatchLease.acknowledge_durable

    def acknowledge(self, *, completed_entities):
        arrive("before_acknowledgement")
        original_acknowledge(self, completed_entities=completed_entities)
        arrive("after_acknowledgement")

    PersistQueueBatchLease.acknowledge_durable = acknowledge
    reservation = queue.try_reserve_one(MEASUREMENT_PROFILE_V1_SPEC)
    reservation.reservation.publish(intended)
    owner.attempt(max_entities=1)
    raise RuntimeError(f"child did not reach {boundary}")


def _kill_at(path: Path, boundary: str, intended):
    intended_path = path.parent / "test-held-intended.pickle"
    with intended_path.open("wb") as destination:
        pickle.dump(intended, destination)
    receiver_root = Path(__file__).resolve().parents[3]
    protocol_root = receiver_root.parent / "protocol/protocol-v2-lora/python"
    stderr_path = path.parent / "child-stderr.txt"
    with stderr_path.open("wb") as stderr:
        child = subprocess.Popen(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                str(path),
                boundary,
                str(intended_path),
            ],
            env={
                **os.environ,
                "PYTHONPATH": os.pathsep.join((str(receiver_root), str(protocol_root))),
            },
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=stderr,
        )
        try:
            with selectors.DefaultSelector() as readiness:
                readiness.register(child.stdout, selectors.EVENT_READ)
                assert readiness.select(
                    timeout=20
                ), f"child readiness timeout: {stderr_path}"
                line = child.stdout.readline()
            assert line, f"child exited before readiness: {stderr_path.read_text()}"
            observed = json.loads(line)
            assert observed == {
                "boundary": boundary,
                "published": 0 if boundary == "after_acknowledgement" else 1,
            }
            child.kill()
            assert child.wait(timeout=10) == -signal.SIGKILL
        finally:
            if child.poll() is None:
                child.kill()
                child.wait(timeout=10)
            child.stdin.close()
            child.stdout.close()
    assert stderr_path.read_text() == ""
    # Preserve exact files before SQLite performs startup WAL recovery/bookkeeping.
    evidence = path.parent / "crash-evidence"
    evidence.mkdir()
    for artifact in (
        path,
        path.with_name(path.name + "-wal"),
        path.with_name(path.name + "-shm"),
    ):
        assert artifact.exists(), artifact
        shutil.copy2(artifact, evidence / artifact.name)
    (evidence / "boundary.json").write_text(json.dumps(observed) + "\n")
    return evidence


def exercise_pair_crash(tmp_path, boundary):
    path = tmp_path / "receiver.db"
    initialize_database(path, GROUP)
    opened = open_receiver_database(path, GROUP, minimum_free_bytes=0)
    insert_receiver_instance_start(
        opened.database.connection, ReceiverInstanceStart(INSTANCE, 0), b"b" * 16
    )
    opened.database.close()
    intended = _measurement()
    _kill_at(path, boundary, intended)
    connection, queue, owner = _owner(path)
    try:
        expected_count = int(boundary in DURABLE_BOUNDARIES)
        assert connection.execute(
            "SELECT count(*) FROM message_profiles"
        ).fetchone() == (expected_count,)
        assert connection.execute(
            "SELECT count(*) FROM reading_messages"
        ).fetchone() == (expected_count,)
        # The replacement process has no old RAM queue. This is test-driven replay,
        # not a claim that production can recover a volatile queue after SIGKILL.
        assert queue.snapshot().published_entities == 0
        before = tuple(
            connection.execute(f"SELECT * FROM {table}").fetchall()
            for table in ("message_profiles", "reading_messages")
        )
        queue.try_reserve_one(MEASUREMENT_PROFILE_V1_SPEC).reservation.publish(intended)
        assert owner.attempt(max_entities=1).acknowledged_entities == 1
        assert queue.snapshot().published_entities == 0
        after = tuple(
            connection.execute(f"SELECT * FROM {table}").fetchall()
            for table in ("message_profiles", "reading_messages")
        )
        assert [len(rows) for rows in after] == [1, 1]
        if expected_count:
            assert after == before
        assert connection.execute(
            "SELECT persistence_classification_id FROM message_profiles"
        ).fetchone() == (1,)
        assert connection.execute(
            "SELECT node_id,message_id,sample_id,is_canonical_for_sample,reading_body FROM reading_messages"
        ).fetchone() == (b"n" * 8, 100, 200, 1, intended.candidate.reading_body)
        assert connection.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
        assert connection.execute("PRAGMA foreign_key_check").fetchall() == []
    finally:
        owner.close()


if __name__ == "__main__":
    _child(Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3]))
