from __future__ import annotations

import json
import os
import shutil
import sqlite3
import subprocess
import tempfile
import time
from pathlib import Path

import pytest
from cura_receiver.database_initializer import initialize_database
from cura_receiver.generated.receiver_enums_generated import (
    AdmissionResult,
    PersistenceAdmissionState as State,
)
from cura_receiver.ordinary_persistence import OrdinaryPersistence
from cura_receiver.persist_queue import PersistQueue
from cura_receiver.persist_queue_entities import (
    CLOCK_OBSERVATION_V1_SPEC,
    DIAGNOSTIC_V1_SPEC,
    MEASUREMENT_PROFILE_V1_SPEC,
    PROFILE_ONLY_V1_SPEC,
    RECEIVER_HEALTH_REQUEST_V1_SPEC,
    ProfileOnlyUnitV1,
)
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_host_observations import LinuxHostObservations
from cura_receiver.receiver_startup import (
    ReceiverInstanceStart,
    insert_receiver_instance_start,
)
from cura_receiver.sqlite_database import open_receiver_database
from tests.support.builders.persistence import (
    GROUP,
    INSTANCE,
    _diagnostic,
    _health_request,
    _measurement,
    _observation,
    _profile,
)
from tests.support.coordination.persistence_crash import exercise_pair_crash
from tests.hardware.conftest import _validated_destructive_test_root

pytestmark = pytest.mark.hardware


def _create(path, *, minimum_free_bytes=0, host_observations=None):
    initialize_database(path, GROUP)
    connection = open_receiver_database(path, GROUP, minimum_free_bytes=0).connection
    instance = ReceiverInstanceStart(INSTANCE, 0)
    insert_receiver_instance_start(connection, instance, b"b" * 16)
    queue = PersistQueue()
    owner = OrdinaryPersistence(
        connection,
        queue,
        instance=instance,
        database_path=path,
        group_id=GROUP,
        clock=LinuxOsClock(),
        minimum_free_bytes=minimum_free_bytes,
        host_observations=host_observations,
    )
    owner.enable_admission()
    return connection, queue, owner


def _publish(queue, entity, spec):
    reservation = queue.try_reserve_one(spec)
    assert reservation.status is AdmissionResult.RESERVED
    reservation.reservation.publish(entity)


def _wait_due(owner):
    deadline = owner.retry_deadline_monotonic_us
    assert deadline is not None
    remaining = deadline - LinuxOsClock().now_monotonic_us()
    if remaining > 0:
        time.sleep(remaining / 1_000_000)


def _sudo(*arguments):
    # Optional credential is supplied only in the invoking process environment;
    # never write it to source, artifact, stdout or the subprocess command line.
    password = os.environ.get("CURA_RECEIVER_TEST_SUDO_PASSWORD")
    result = subprocess.run(
        ["sudo", "-S" if password else "-n", "--", *arguments],
        input=None if password is None else password + "\n",
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert (
        result.returncode == 0
    ), f"isolated storage fixture command failed: {result.stderr}"


@pytest.fixture
def bounded_storage(request):
    root = _validated_destructive_test_root(
        request.config.getoption("receiver_test_root")
    )
    fixture = Path(tempfile.mkdtemp(prefix="ordinary-storage-", dir=root))
    mount = fixture / "mount"
    mount.mkdir()
    mounted = False
    try:
        _sudo(
            "mount",
            "-t",
            "tmpfs",
            "-o",
            f"size=4m,mode=0700,uid={os.getuid()},gid={os.getgid()}",
            "cura-receiver-test",
            str(mount),
        )
        mounted = True
        assert mount.stat().st_dev != fixture.stat().st_dev
        assert (
            os.statvfs(mount).f_blocks * os.statvfs(mount).f_frsize <= 4 * 1024 * 1024
        )
        yield mount
    finally:
        if mounted:
            # Retain bounded artifacts even for a failed assertion before unmount.
            try:
                shutil.copytree(mount, fixture / "artifacts", dirs_exist_ok=True)
            finally:
                _sudo("umount", str(mount))
                assert mount.stat().st_dev == fixture.stat().st_dev


# Real mixed entities on deployed storage retain exact identities, frames and health samples at several batch sizes.
@pytest.mark.parametrize("batch_size", [1, 7, 32])
def test_target_mixed_ordinary_workload(tmp_path, batch_size):
    path = tmp_path / "receiver.db"
    observations = []

    class RecordedLinuxHost(LinuxHostObservations):
        def sample(self):
            observed = super().sample()
            observations.append(observed)
            return observed

    connection, queue, owner = _create(path, host_observations=RecordedLinuxHost(path))
    expected = {}
    try:
        for sequence in range(1, 25):
            pair = _measurement(
                sequence=sequence * 2, message=100 + sequence, sample=200 + sequence
            )
            profile = ProfileOnlyUnitV1(_profile(sequence=sequence * 2 + 1))
            units = (
                (_observation(sequence=sequence), CLOCK_OBSERVATION_V1_SPEC),
                (pair, MEASUREMENT_PROFILE_V1_SPEC),
                (profile, PROFILE_ONLY_V1_SPEC),
                (_diagnostic(sequence=sequence), DIAGNOSTIC_V1_SPEC),
                (_health_request(sequence=sequence), RECEIVER_HEALTH_REQUEST_V1_SPEC),
            )
            for entity, spec in units:
                _publish(queue, entity, spec)
            expected[pair.profile.occurrence_sequence] = pair.profile.received_frame
            expected[profile.profile.occurrence_sequence] = bytes(255)
        acknowledged = 0
        while queue.snapshot().published_entities:
            result = owner.attempt(max_entities=batch_size)
            assert result.failure is None
            acknowledged += result.acknowledged_entities
        assert acknowledged == 120
        assert (
            dict(
                connection.execute(
                    "SELECT occurrence_sequence,received_frame FROM message_profiles"
                )
            )
            == expected
        )
        assert connection.execute(
            "SELECT observation_sequence,sampled_at_monotonic_us FROM clock_observations ORDER BY observation_sequence"
        ).fetchall() == [(n, 10) for n in range(1, 25)]
        assert connection.execute(
            "SELECT message_id,sample_id,is_canonical_for_sample,soil_0_mv FROM reading_messages ORDER BY message_id"
        ).fetchall() == [(100 + n, 200 + n, 1, 1000) for n in range(1, 25)]
        assert connection.execute(
            "SELECT diagnostic_sequence,context FROM diagnostics ORDER BY diagnostic_sequence"
        ).fetchall() == [
            (n, bytes.fromhex("00000101") + bytes(124)) for n in range(1, 25)
        ]
        health = connection.execute(
            "SELECT communicator_sampled_at_monotonic_us,linux_load_1m_milli,cpu_temperature_milli_c,memory_available_bytes,sqlite_filesystem_available_bytes,sqlite_database_size_bytes,sqlite_wal_size_bytes FROM receiver_health ORDER BY health_sequence"
        ).fetchall()
        assert health == [
            (
                15,
                o.linux_load_1m_milli,
                o.cpu_temperature_milli_c,
                o.memory_available_bytes,
                o.sqlite_filesystem_available_bytes,
                o.sqlite_database_size_bytes,
                o.sqlite_wal_size_bytes,
            )
            for o in observations
        ]
        tables = (
            "clock_observations",
            "message_profiles",
            "reading_messages",
            "diagnostics",
            "receiver_health",
        )
        before = {
            t: connection.execute(f"SELECT * FROM {t}").fetchall() for t in tables
        }
        checkpoint = owner.checkpoint()
        assert checkpoint.failure is None
        assert checkpoint.checkpointed_frames == checkpoint.wal_frames
        assert {
            t: connection.execute(f"SELECT * FROM {t}").fetchall() for t in tables
        } == before
        assert connection.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
        assert connection.execute("PRAGMA foreign_key_check").fetchall() == []
        (tmp_path / "workload.json").write_text(
            json.dumps(
                {
                    "batch_size": batch_size,
                    "acknowledged": acknowledged,
                    "checkpoint_duration_us": checkpoint.duration_us,
                    "wal_frames": checkpoint.wal_frames,
                    "sqlite": sqlite3.sqlite_version,
                }
            )
            + "\n"
        )
    finally:
        owner.close()


# Deployed SQLite survives process termination at real write, commit and acknowledgement boundaries.
@pytest.mark.parametrize(
    "boundary",
    ["after_profile_write", "before_commit", "after_commit", "after_acknowledgement"],
)
def test_target_ordinary_process_kill(tmp_path, boundary):
    exercise_pair_crash(tmp_path, boundary)


# A dedicated bounded filesystem produces real LOW_SPACE and FULL, preserves pending work and later recovers.
@pytest.mark.destructive
def test_target_bounded_full_recovery(bounded_storage):
    path = bounded_storage / "receiver.db"
    filler = bounded_storage / "fixture-filler"
    filler.write_bytes(bytes(1024 * 1024))
    connection, queue, owner = _create(path, minimum_free_bytes=4 * 1024 * 1024)
    try:
        _publish(queue, ProfileOnlyUnitV1(_profile()), PROFILE_ONLY_V1_SPEC)
        assert (
            owner.attempt(max_entities=64).failure.admission_state
            is State.UNAVAILABLE_LOW_SPACE
        )
        assert queue.snapshot().published_entities == 1
        owner._minimum_free_bytes = 0
        _wait_due(owner)
        assert owner.attempt(max_entities=64).acknowledged_entities == 1
        committed = 1
        failure = None
        for batch in range(128):
            for offset in range(64):
                sequence = 2 + batch * 64 + offset
                _publish(
                    queue,
                    ProfileOnlyUnitV1(_profile(sequence=sequence)),
                    PROFILE_ONLY_V1_SPEC,
                )
            attempt = owner.attempt(max_entities=64)
            if attempt.failure is not None:
                failure = attempt.failure
                break
            committed += 64
        assert failure is not None, "bounded fixture never produced SQLite FULL"
        assert failure.admission_state is State.UNAVAILABLE_DISK_FULL
        assert queue.snapshot().published_entities == 64
        assert (
            queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).status
            is AdmissionResult.PERSISTENCE_UNAVAILABLE
        )
        assert connection.execute(
            "SELECT count(*) FROM message_profiles"
        ).fetchone() == (committed,)
        assert owner.attempt(max_entities=64) is None
        filler.unlink()
        _wait_due(owner)
        result = owner.attempt(max_entities=64)
        assert result.acknowledged_entities == 64
        assert queue.snapshot().admission_snapshot.state is State.AVAILABLE
        assert connection.execute(
            "SELECT count(*) FROM message_profiles"
        ).fetchone() == (committed + 64,)
        assert connection.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
    finally:
        owner.close()


# Changes to only the fixture's permissions or mount retain work through repeated paced I/O failures.
@pytest.mark.destructive
@pytest.mark.parametrize("mode", ["permissions", "readonly_mount"])
def test_target_access_recovery(bounded_storage, mode):
    assert os.geteuid() != 0, "access test requires the unprivileged receiver account"
    path = bounded_storage / "receiver.db"
    connection, queue, owner = _create(path)
    _publish(queue, _observation(), CLOCK_OBSERVATION_V1_SPEC)
    try:
        try:
            if mode == "permissions":
                path.chmod(0)
            else:
                # Linux requires writable handles closed before a read-only remount.
                owner.close()
                _sudo("mount", "-o", "remount,ro", str(bounded_storage))
            for _ in range(2):
                result = owner.attempt(max_entities=1)
                assert result.failure.admission_state is State.UNAVAILABLE_IO
                assert queue.snapshot().published_entities == 1
                assert owner.attempt(max_entities=1) is None
                _wait_due(owner)
        finally:
            if mode == "permissions":
                path.chmod(0o600)
            else:
                _sudo("mount", "-o", "remount,rw", str(bounded_storage))
        assert owner.attempt(max_entities=1).acknowledged_entities == 1
        assert queue.snapshot().admission_snapshot.state is State.AVAILABLE
        observer = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        try:
            assert observer.execute("SELECT * FROM clock_observations").fetchall() == [
                (INSTANCE, 1, 0, 10, None, 0, 0, 1)
            ]
        finally:
            observer.close()
    finally:
        owner.close()


# Real corrupt copied files stay preserved and recovery needs an explicit maintenance handoff.
def test_target_corrupt_artifacts_and_maintenance(tmp_path):
    active = tmp_path / "active"
    active.mkdir()
    path = active / "receiver.db"
    connection, queue, owner = _create(path)
    _publish(queue, _observation(), CLOCK_OBSERVATION_V1_SPEC)
    try:
        path.chmod(0)
        try:
            assert (
                owner.attempt(max_entities=1).failure.admission_state
                is State.UNAVAILABLE_IO
            )
        finally:
            path.chmod(0o600)
        owner.close()
        backup = tmp_path / "backup"
        shutil.copytree(active, backup)
        with path.open("r+b") as destination:
            destination.write(b"not a SQLite db!")
        original = {
            p.name: (p.stat().st_ino, p.stat().st_size, p.read_bytes())
            for p in active.iterdir()
        }
        _wait_due(owner)
        assert (
            owner.attempt(max_entities=1).failure.admission_state
            is State.UNAVAILABLE_CORRUPT
        )
        for p in active.iterdir():
            inode, size, data = original[p.name]
            assert (p.stat().st_ino, p.stat().st_size) == (inode, size)
            if not p.name.endswith("-shm"):
                assert p.read_bytes() == data
        assert owner.attempt(max_entities=1) is None
        active.rename(tmp_path / "corrupt-evidence")
        shutil.copytree(backup, active)
        assert owner.attempt(max_entities=1) is None
        owner.request_operator_recovery()
        assert owner.attempt(max_entities=1).acknowledged_entities == 1
        assert queue.snapshot().admission_snapshot.state is State.AVAILABLE
    finally:
        owner.close()
