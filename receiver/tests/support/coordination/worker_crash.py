import json
import os
from pathlib import Path
import selectors
import shutil
import signal
import sqlite3
import subprocess
import sys

from cura_receiver.persist_queue import PersistQueueBatchLease
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC, ProfileOnlyUnitV1
from cura_receiver.persistence_control_values import ReceiverCleanStopV1
from cura_receiver.persistence_worker import PersistenceWorker
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.sqlite_database import ReceiverDatabase
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import INSTANCE, _profile
from tests.support.builders.persistence_control import synthetic
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
from tests.support.fakes.os_clock import FakeOsClock


BOUNDARIES = (
    "startup_before_insert",
    "startup_before_commit",
    "startup_after_commit",
    "state_before_begin",
    "state_after_write",
    "state_after_archive",
    "state_before_commit",
    "state_after_commit",
    "marker_before_commit",
    "marker_after_commit",
    "before_checkpoint",
    "after_checkpoint",
    "before_close",
    "ordinary_before_ack",
    "ordinary_after_ack",
)


def child(root, boundary):
    import cura_receiver.receiver_startup as startup

    def arrive(name):
        if name == boundary:
            print(json.dumps({"boundary": name}), flush=True)
            sys.stdin.buffer.read(1)
            raise RuntimeError("parent unexpectedly released crash child")

    original_start = startup.insert_receiver_instance_start

    def insert(db, *args, **kwargs):
        arrive("startup_before_insert")
        db.set_trace_callback(
            lambda sql: arrive("startup_before_commit") if sql == "COMMIT" else None
        )
        result = original_start(db, *args, **kwargs)
        db.set_trace_callback(None)
        arrive("startup_after_commit")
        return result

    startup.insert_receiver_instance_start = insert

    class CrashTransactions(SqliteTransactions):
        def begin(self, db):
            if boundary.startswith("state_"):
                arrive("state_before_begin")
                db.create_function("arrive", 1, arrive)
                for table, name in [
                    ("communicator_state", "state_after_write"),
                    ("quarantined_communicator_states", "state_after_archive"),
                ]:
                    db.execute(
                        f"CREATE TEMP TRIGGER IF NOT EXISTS gate_{table} AFTER INSERT ON {table} BEGIN SELECT arrive('{name}'); END"
                    )
            super().begin(db)

        def commit(self, db):
            prefix = "state" if boundary.startswith("state_") else "marker"
            arrive(prefix + "_before_commit")
            super().commit(db)
            arrive(prefix + "_after_commit")

        def checkpoint(self, db):
            arrive("before_checkpoint")
            result = super().checkpoint(db)
            arrive("after_checkpoint")
            return result

    original_close = ReceiverDatabase.close

    def close(db):
        arrive("before_close")
        original_close(db)

    ReceiverDatabase.close = close
    original_ack = PersistQueueBatchLease.acknowledge_durable

    def acknowledge(lease, *, completed_entities):
        arrive("ordinary_before_ack")
        original_ack(lease, completed_entities=completed_entities)
        arrive("ordinary_after_ack")

    PersistQueueBatchLease.acknowledge_durable = acknowledge
    owner = PersistenceWorker(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=root / "worker.db",
        configuration_path=root / "test-group.json",
        boot_id_path=root / "boot-id",
        clock=FakeOsClock(monotonic_us=100),
        transactions=CrashTransactions(),
        wake_threshold_entities=1,
    )
    owner.start()
    assert owner.wait_started(deadline_monotonic_us=3_600_000_100)
    if boundary.startswith("state_"):
        owner.control.commit_communicator_state(
            synthetic(), deadline_monotonic_us=3_600_000_100
        )
    elif boundary.startswith("ordinary_"):
        owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
            ProfileOnlyUnitV1(_profile())
        )
    else:
        owner.queue.close()
        owner.control.commit_receiver_clean_stop(
            ReceiverCleanStopV1(INSTANCE, 100, 0), deadline_monotonic_us=3_600_000_100
        )
        owner.request_stop(deadline_monotonic_us=3_600_000_100)
    owner.join(3600)  # Parent enforces a 20-second readiness budget and exact SIGKILL.
    raise RuntimeError("crash boundary was not reached")


def kill_at(root, boundary):
    receiver_root = Path(__file__).resolve().parents[3]
    protocol_root = receiver_root.parent / "protocol/protocol-v2-lora/python"
    stderr_path = root / "worker-child-stderr.txt"
    with stderr_path.open("wb") as errors:
        process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "tests.support.coordination.worker_crash",
                str(root),
                boundary,
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=errors,
            env={
                **os.environ,
                "PYTHONPATH": os.pathsep.join((str(receiver_root), str(protocol_root))),
            },
        )
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(process.stdout, selectors.EVENT_READ)
                assert selector.select(20), f"worker child timeout: {stderr_path}"
                line = process.stdout.readline()
            assert line, stderr_path.read_text()
            assert json.loads(line) == {"boundary": boundary}
            process.kill()
            assert process.wait(10) == -signal.SIGKILL
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(10)
            process.stdin.close()
            process.stdout.close()
    assert stderr_path.read_text() == ""
    evidence = root / "worker-crash-evidence"
    evidence.mkdir()
    for suffix in ("", "-wal", "-shm"):
        path = root / ("worker.db" + suffix)
        if path.exists():
            shutil.copy2(path, evidence / path.name)
    (evidence / "boundary.json").write_text(json.dumps({"boundary": boundary}) + "\n")
    return evidence


# SIGKILL at actual worker boundaries preserves atomic control effects and starts a new empty process queue.
def exercise_worker_crash(worker_files, boundary):
    path, config, boot = worker_files
    root = path.parent
    if boundary == "state_after_archive":
        with sqlite3.connect(path) as db:
            db.execute("INSERT INTO communicator_state VALUES (1,1,1,x'00',x'00')")
    evidence = kill_at(root, boundary)
    assert (evidence / "worker.db").exists()
    restarted_id = bytes.fromhex("00112233445546778899aabbccddee01")
    owner = CheckedPersistenceWorker(
        instance=ReceiverInstanceStart(restarted_id, 200),
        database_path=path,
        configuration_path=config,
        boot_id_path=boot,
        clock=FakeOsClock(monotonic_us=300),
    )
    owner.start()
    try:
        started = owner.wait_started(deadline_monotonic_us=5_000_300)
        assert started.database_failure is None
        assert owner.queue.snapshot().published_entities == 0
        with sqlite3.connect(path) as db:
            instances = db.execute(
                "SELECT receiver_instance_id, linux_boot_id, clean_stopped_at_monotonic_us, clean_stop_state_generation FROM receiver_instances ORDER BY instance_ordinal"
            ).fetchall()
            old_start = boundary not in (
                "startup_before_insert",
                "startup_before_commit",
            )
            assert len(instances) == 1 + old_start
            assert instances[-1][0] == restarted_id and instances[-1][2:] == (
                None,
                None,
            )
            if old_start:
                assert (
                    instances[0][0] == INSTANCE and instances[0][1] == instances[-1][1]
                )
                clean = boundary in (
                    "marker_after_commit",
                    "before_checkpoint",
                    "after_checkpoint",
                    "before_close",
                )
                assert instances[0][2:] == ((100, 0) if clean else (None, None))
            expected_state_rows = int(
                boundary in ("state_after_commit", "state_after_archive")
            )
            assert db.execute("SELECT count(*) FROM communicator_state").fetchone() == (
                expected_state_rows,
            )
            assert db.execute(
                "SELECT count(*) FROM quarantined_communicator_states"
            ).fetchone() == (0,)
            assert db.execute("SELECT count(*) FROM message_profiles").fetchone() == (
                int(boundary.startswith("ordinary_")),
            )
            assert db.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
            assert db.execute("PRAGMA foreign_key_check").fetchall() == []
        if boundary == "state_after_commit":
            assert (
                owner.control.load_communicator_state(
                    deadline_monotonic_us=5_000_300
                ).state
                == synthetic()
            )
    finally:
        owner.finish_test()


if __name__ == "__main__":
    child(Path(sys.argv[1]), sys.argv[2])
