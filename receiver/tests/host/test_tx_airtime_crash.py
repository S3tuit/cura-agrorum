"""Actual process termination at the airtime/SQLite recovery and usage-save boundaries."""

import json
import os
from pathlib import Path
import selectors
import shutil
import signal
import sqlite3
import subprocess
import sys
import uuid

import pytest

from cura_receiver.airtime_ledger import AirtimeCorrelation
from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.generated.receiver_entities_generated import (
    communicator_state_v2_parameters,
)
from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth,
    SystemTimeQuality,
)
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.sqlite_transactions import SqliteTransactions
from cura_receiver.time_observations import TrustedTimeSample
from cura_receiver.tx_airtime import TxAirtimePolicy, AirtimeReason as R, TxCertainty
from tests.support.builders.persistence_control import state
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
from tests.support.fakes.os_clock import FakeOsClock

BOUNDARIES = (
    "recovery_before_commit",
    "recovery_after_commit",
    "recovery_acknowledged",
    "tentative_spend",
    "definite_reclaim",
    "save_before_commit",
    "save_after_commit",
)


def component(root, elapsed, *, transactions=None):
    clock = FakeOsClock(monotonic_us=elapsed + 100)
    worker = CheckedPersistenceWorker(
        instance=ReceiverInstanceStart(uuid.uuid4().bytes, elapsed),
        database_path=root / "worker.db",
        configuration_path=root / "test-group.json",
        boot_id_path=root / "boot-id",
        clock=clock,
        transactions=transactions,
    )
    worker.start()
    started = worker.wait_started(deadline_monotonic_us=elapsed + 5_000_100)
    assert started is not None and started.database_failure is None
    owner = CommunicatorStateOwner.from_load(
        control=worker.control, loaded=started.state_load
    )
    policy = TxAirtimePolicy(state_owner=owner, clock=clock)
    policy.update_time(
        AirtimeCorrelation(
            TrustedTimeSample(
                elapsed + 100, elapsed, 1, SystemTimeQuality.NETWORK_SYNCED, 1
            ),
            1,
            elapsed + 10_000_000_000,
        ),
        rtc_health=RtcHealth.PRESENT,
    )
    return policy, worker, clock


def child(root, boundary, elapsed):
    def arrive(name):
        if name == boundary:
            print(json.dumps({"boundary": name}), flush=True)
            sys.stdin.buffer.read(1)
            raise RuntimeError("parent unexpectedly released a crash boundary")

    class BoundaryTransactions(SqliteTransactions):
        phase = None

        def commit(self, connection):
            arrive(str(self.phase) + "_before_commit")
            super().commit(connection)
            arrive(str(self.phase) + "_after_commit")

    transactions = BoundaryTransactions()
    policy, worker, clock = component(root, elapsed, transactions=transactions)
    try:
        assert policy.available_charge_us == 0
        transactions.phase = "recovery"
        result = policy.maintain(deadline_monotonic_us=elapsed + 5_000_100)
        if boundary == "recovery_denied":
            assert policy.available_charge_us == 0
            arrive("recovery_denied")
        assert result.reason is R.STATE_READY
        arrive("recovery_acknowledged")
        token = policy.try_spend().token
        assert token is not None
        arrive("tentative_spend")
        if boundary == "definite_reclaim":
            policy.report_tx(token, TxCertainty.NOT_STARTED)
            arrive("definite_reclaim")
        transactions.phase = "save"
        policy.save( deadline_monotonic_us=elapsed + 5_000_100)
        raise RuntimeError("selected child boundary was not reached")
    finally:
        worker.finish_test()


def kill_at(root, boundary, *, elapsed=0, episode="one"):
    receiver = Path(__file__).resolve().parents[2]
    error_path = root / f"{episode}-stderr.txt"
    with error_path.open("wb") as errors:
        process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "tests.host.test_tx_airtime_crash",
                str(root),
                boundary,
                str(elapsed),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=errors,
            env={
                **os.environ,
                "PYTHONPATH": os.pathsep.join(
                    (
                        str(receiver),
                        str(receiver.parent / "protocol/protocol-v2-lora/python"),
                    )
                ),
            },
        )
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(process.stdout, selectors.EVENT_READ)
                assert selector.select(20), f"child readiness timeout: {error_path}"
                line = process.stdout.readline()
            assert line, error_path.read_text()
            assert json.loads(line) == {"boundary": boundary}
            process.kill()
            assert process.wait(10) == -signal.SIGKILL
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(10)
            process.stdin.close()
            process.stdout.close()
    assert error_path.read_text() == ""
    evidence = root / f"{episode}-crash-evidence"
    evidence.mkdir()
    for suffix in ("", "-wal", "-shm"):
        path = root / ("worker.db" + suffix)
        if path.exists():
            shutil.copy2(path, evidence / path.name)
    (evidence / "boundary.json").write_text(
        json.dumps({"boundary": boundary, "elapsed_us": elapsed}) + "\n"
    )
    return evidence


def seed(path):
    with sqlite3.connect(path) as connection:
        connection.execute(
            "INSERT INTO communicator_state VALUES (?,?,?,?,?)",
            communicator_state_v2_parameters(state()),
        )


# SIGKILL before/after actual commit and volatile certainty updates leaves conservative durable charge.
@pytest.mark.parametrize("boundary", BOUNDARIES)
def test_airtime_process_kill_at_durable_boundaries(worker_files, boundary):
    database, _, _ = worker_files
    root = database.parent
    seed(database)
    evidence = kill_at(root, boundary)
    assert (evidence / "worker.db").exists()
    replacement, worker, clock = component(root, 1)
    try:
        assert replacement.available_charge_us == 0
        assert (
            replacement.recover(deadline_monotonic_us=5_000_101).reason is R.STATE_READY
        )
        expected = (2_000_000 if boundary == "recovery_before_commit" else
                    6_000_000 if boundary == "save_after_commit" else 4_000_000)
        assert replacement.total_used == expected
        assert replacement.state.generation == (2 if boundary == "recovery_before_commit" else
                                                4 if boundary == "save_after_commit" else 3)
        assert replacement.maintain(deadline_monotonic_us=5_000_101).reason is R.ALLOWED
        assert replacement.available_charge_us == 2_000_000
        with sqlite3.connect(database) as connection:
            instances = connection.execute(
                "SELECT receiver_instance_id, linux_boot_id FROM receiver_instances ORDER BY instance_ordinal"
            ).fetchall()
            assert len(instances) == 2 and instances[0][0] != instances[1][0]
            assert instances[0][1] == instances[1][1]
            assert connection.execute("PRAGMA integrity_check").fetchone() == ("ok",)
            assert connection.execute("PRAGMA foreign_key_check").fetchall() == []
    finally:
        worker.finish_test()


# Every confirmed no-TX recovery consumes an entry until the full array blocks ACK.
def test_repeated_airtime_process_crashes_accumulate_conservatively(worker_files):
    database, _, _ = worker_files
    root = database.parent
    seed(database)
    for episode in range(18):
        kill_at(root, "recovery_acknowledged", elapsed=episode*10_000_000, episode=str(episode))
    replacement, worker, clock = component(root, 180_000_000)
    try:
        assert replacement.recover(deadline_monotonic_us=185_000_100).reason is R.STATE_READY
        assert replacement.total_used == 36_000_000
        assert replacement.available_charge_us == 0
        assert replacement.state.generation == 20
    finally:
        worker.finish_test()


@pytest.mark.parametrize("boundary", BOUNDARIES)
def test_commissioning_process_kill_keeps_token_and_state_atomic(tmp_path, boundary):
    from tests.support.coordination.persistence_worker import prepare_worker_files
    database, _, _ = prepare_worker_files(tmp_path, known_empty_airtime=True)
    kill_at(tmp_path, boundary)
    before_commit = boundary == "recovery_before_commit"
    with sqlite3.connect(database) as connection:
        assert connection.execute('SELECT count(*) FROM airtime_commissioning').fetchone() == (
            1 if before_commit else 0,)
        assert connection.execute('SELECT count(*) FROM communicator_state').fetchone() == (
            0 if before_commit else 1,)
    replacement, worker, clock = component(tmp_path, 1)
    try:
        assert replacement.available_charge_us == 0
        assert replacement.recover(deadline_monotonic_us=5_000_101).reason is R.STATE_READY
        assert replacement.total_used == (
            0 if before_commit else 4_000_000 if boundary == "save_after_commit" else 2_000_000)
        assert replacement.state.generation == (
            1 if before_commit else 3 if boundary == "save_after_commit" else 2)
        with sqlite3.connect(database) as connection:
            assert connection.execute('SELECT * FROM airtime_commissioning').fetchall() == []
            assert connection.execute('PRAGMA integrity_check').fetchone() == ('ok',)
    finally:
        worker.finish_test()


def test_commissioning_is_not_repeated_across_process_crashes(tmp_path):
    from tests.support.coordination.persistence_worker import prepare_worker_files
    prepare_worker_files(tmp_path, known_empty_airtime=True)
    for episode in range(4):
        kill_at(tmp_path, "recovery_acknowledged", elapsed=episode * 10_000_000,
                episode=str(episode))
    replacement, worker, clock = component(tmp_path, 40_000_000)
    try:
        assert replacement.recover(deadline_monotonic_us=45_000_100).reason is R.STATE_READY
        assert replacement.state.generation == 5
        assert replacement.total_used == 8_000_000
    finally:
        worker.finish_test()


if __name__ == "__main__":
    child(Path(sys.argv[1]), sys.argv[2], int(sys.argv[3]))
