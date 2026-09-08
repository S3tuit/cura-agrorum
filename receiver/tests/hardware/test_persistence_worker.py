"""Target storage/scheduling evidence for the worker with a bounded test caller.

Control deadlines are 5 s; an additional 0.5 s allows caller scheduling after
completion. These are asymmetric acceptance bounds, not hard real-time claims.
The slow case runs 30 s with one CPU/storage load process and a fixed seed.
"""

from dataclasses import replace
import json
import platform
import random
import selectors
import sqlite3
import subprocess
import sys
from threading import Event
import time
import traceback

import pytest

from cura_receiver.generated.receiver_enums_generated import AdmissionResult
from cura_receiver.persist_queue_entities import (
    CLOCK_OBSERVATION_V1_SPEC,
    DIAGNOSTIC_V1_SPEC,
    MEASUREMENT_PROFILE_V1_SPEC,
    PROFILE_ONLY_V1_SPEC,
    RECEIVER_HEALTH_REQUEST_V1_SPEC,
    ProfileOnlyUnitV1,
)
from cura_receiver.persistence_control_values import (
    CommunicatorStateLoadStatus as L,
    ReceiverCleanStopV1,
    ReceiverCleanStopCommitDisposition as D,
)
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import (
    INSTANCE,
    _profile,
    _measurement,
    _observation,
    _diagnostic,
    _health_request,
)
from tests.support.builders.persistence_control import synthetic
from tests.support.coordination.persistence_worker import (
    CheckedPersistenceWorker,
    prepare_worker_files,
)
from tests.support.coordination.threads import (
    start_checked_threads,
    join_checked_threads,
)
from tests.support.coordination.worker_crash import exercise_worker_crash

pytestmark = pytest.mark.hardware


def make_worker(root, cls=CheckedPersistenceWorker, **kwargs):
    path, config, boot = prepare_worker_files(root)
    owner = cls(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=path,
        configuration_path=config,
        boot_id_path=boot,
        clock=LinuxOsClock(),
        **kwargs,
    )
    owner.start()
    assert owner.wait_started(deadline_monotonic_us=deadline(owner, 10))
    return owner, path


def deadline(owner, seconds=5):
    return owner._clock.now_monotonic_us() + int(seconds * 1_000_000)


def publish(owner, sequence):
    reservation = owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert reservation.status is AdmissionResult.RESERVED
    reservation.reservation.publish(ProfileOnlyUnitV1(_profile(sequence=sequence)))


def evidence(root, **values):
    values.update(
        python=sys.version,
        sqlite=sqlite3.sqlite_version,
        platform=platform.platform(),
        control_deadline_us=5_000_000,
        caller_scheduling_tolerance_us=500_000,
    )
    values["thread_stacks"] = {
        str(key): traceback.format_stack(frame)
        for key, frame in sys._current_frames().items()
    }
    (root / "worker-evidence.json").write_text(json.dumps(values, indent=2) + "\n")


# Idle, in-flight commit and paced retry all service controls at the next safe boundary on target.
@pytest.mark.parametrize("phase", ["idle", "commit", "backoff"])
def test_target_control_latency(tmp_path, phase):
    arrived, release, submitted = Event(), Event(), Event()
    trace, samples = [], []

    class Backend(SqliteTransactions):
        first = True

        def begin(self, db):
            if self.first and phase == "backoff":
                self.first = False
                raise OSError(5, "target bounded retry fault")
            super().begin(db)

        def commit(self, db):
            if self.first and phase == "commit":
                self.first = False
                arrived.set()
                assert release.wait(10)
            super().commit(db)

    class Observed(CheckedPersistenceWorker):
        def _wait_for_work(self, timeout):
            if (
                phase == "backoff"
                and self._ordinary.retry_deadline_monotonic_us is not None
            ):
                arrived.set()
                assert release.wait(10)
            super()._wait_for_work(timeout)

        def _dispatch_work(self, action):
            trace.append(action)
            super()._dispatch_work(action)

        def _dispatch_control(self, command):
            trace.append("control")
            super()._dispatch_control(command)

    owner, _ = make_worker(
        tmp_path, Observed, transactions=Backend(), wake_threshold_entities=1
    )
    original_wait = owner.control._wait_for_completion

    def completion(command, remaining):
        submitted.set()
        return original_wait(command, remaining)

    owner.control._wait_for_completion = completion
    try:
        if phase != "idle":
            publish(owner, 1)
            assert arrived.wait(10)

        def caller():
            started = time.perf_counter_ns()
            loaded = owner.control.load_communicator_state(
                deadline_monotonic_us=deadline(owner)
            )
            samples.append((time.perf_counter_ns() - started) // 1000)
            assert loaded.status is L.STATE_UNAVAILABLE

        threads = start_checked_threads([("target-control-caller", caller)])
        assert submitted.wait(10)
        release.set()
        join_checked_threads(threads, timeout_seconds=10)
        assert 0 <= samples[0] <= 5_500_000
        assert trace[:2] == (
            ["control"] if phase == "idle" else ["ordinary", "control"]
        )
    finally:
        release.set()
        owner.finish_test()
        evidence(tmp_path, phase=phase, control_latency_us=samples, trace=trace)


# WAL threshold dispatch uses real target SQLite; a blocked checkpoint leaves publication and control submission independent.
def test_target_threshold_checkpoint_stall(tmp_path):
    arrived, release, submitted, drained = Event(), Event(), Event(), Event()
    trace, wal_sizes, checkpoint_us = [], [], []

    class Backend(SqliteTransactions):
        first = True

        def checkpoint(self, db):
            if self.first:
                self.first = False
                arrived.set()
                assert release.wait(10)
            start = time.perf_counter_ns()
            result = super().checkpoint(db)
            checkpoint_us.append((time.perf_counter_ns() - start) // 1000)
            return result

    class Observed(CheckedPersistenceWorker):
        def _dispatch_work(self, action):
            trace.append(action)
            if action == "checkpoint":
                wal_sizes.append(self._wal_bytes())
            super()._dispatch_work(action)
            if self.queue.snapshot().published_entities == 0:
                drained.set()

        def _dispatch_control(self, command):
            trace.append("control")
            super()._dispatch_control(command)

    owner, path = make_worker(
        tmp_path,
        Observed,
        transactions=Backend(),
        wake_threshold_entities=1,
        batch_limit_entities=8,
        checkpoint_threshold_bytes=32768,
    )
    original_wait = owner.control._wait_for_completion

    def completion(command, remaining):
        submitted.set()
        return original_wait(command, remaining)

    owner.control._wait_for_completion = completion
    try:
        for sequence in range(1, 33):
            publish(owner, sequence)
        assert arrived.wait(10)
        trace.clear()
        drained.clear()
        start = time.perf_counter_ns()
        for sequence in range(33, 97):
            publish(owner, sequence)
        publication_us = (time.perf_counter_ns() - start) // 1000
        assert owner.queue.snapshot().published_entities >= 64
        threads = start_checked_threads(
            [
                (
                    "target-checkpoint-control",
                    lambda: owner.control.load_communicator_state(
                        deadline_monotonic_us=deadline(owner)
                    ),
                )
            ]
        )
        assert submitted.wait(10)
        assert trace == []
        release.set()
        join_checked_threads(threads, timeout_seconds=10)
        assert drained.wait(10)
        assert trace[0] == "control"
        assert wal_sizes[0] >= 32768
        assert owner.queue.snapshot().published_entities == 0
        with sqlite3.connect(path) as db:
            assert db.execute("SELECT count(*) FROM message_profiles").fetchone() == (
                96,
            )
    finally:
        release.set()
        owner.finish_test()
        evidence(
            tmp_path,
            trace=trace,
            wal_sizes_bytes=wal_sizes,
            checkpoint_us=checkpoint_us,
            publication_64_entities_us=locals().get("publication_us"),
        )


# The deployed worker restarts after representative startup, state, marker and acknowledgement SIGKILL boundaries.
@pytest.mark.parametrize(
    "boundary",
    [
        "startup_after_commit",
        "state_after_archive",
        "state_after_commit",
        "marker_before_commit",
        "marker_after_commit",
        "ordinary_before_ack",
        "before_close",
    ],
)
def test_target_worker_process_kill(tmp_path, boundary):
    exercise_worker_crash(prepare_worker_files(tmp_path), boundary)


_LOAD = """import hashlib, os, sys, time
f = open(sys.argv[1], 'w+b', buffering=0)
data = b'x' * 65536
end = time.monotonic() + 50
print('ready', flush=True)
while time.monotonic() < end:
    for _ in range(100): hashlib.sha256(data).digest()
    f.seek(0); f.write(data); os.fsync(f.fileno())
f.close()
"""


# A repeatable 30-second mixed-work soak under CPU/storage load preserves progress, control generations and clean restart.
@pytest.mark.slow
def test_target_worker_mixed_race_soak(tmp_path):
    seed = 805_031
    randomizer = random.Random(seed)
    drained = Event()
    latencies, commits_us, checkpoint_us = [], [], []
    accepted = rejected = sequence = 0
    load = subprocess.Popen(
        [sys.executable, "-c", _LOAD, str(tmp_path / "bounded-load.bin")],
        stdout=subprocess.PIPE,
    )

    class Backend(SqliteTransactions):
        begins = 0
        fault_count = 0
        armed = True

        def begin(self, db):
            self.begins += 1
            if self.armed and self.begins % 43 == 0:
                self.fault_count += 1
                raise OSError(5, "bounded target soak fault")
            super().begin(db)

        def commit(self, db):
            start = time.perf_counter_ns()
            super().commit(db)
            commits_us.append((time.perf_counter_ns() - start) // 1000)

        def checkpoint(self, db):
            start = time.perf_counter_ns()
            result = super().checkpoint(db)
            checkpoint_us.append((time.perf_counter_ns() - start) // 1000)
            return result

    class Observed(CheckedPersistenceWorker):
        def _dispatch_work(self, action):
            super()._dispatch_work(action)
            if self.queue.snapshot().closed_and_drained:
                drained.set()

    backend = Backend()
    owner = None
    try:
        with selectors.DefaultSelector() as selector:
            selector.register(load.stdout, selectors.EVENT_READ)
            assert selector.select(10)
            assert load.stdout.readline() == b"ready\n"
        owner, path = make_worker(
            tmp_path,
            Observed,
            transactions=backend,
            wake_threshold_entities=4,
            batch_limit_entities=8,
            flush_interval_us=100_000,
            checkpoint_threshold_bytes=131072,
        )
        end = time.monotonic() + 30
        generation = 0
        kinds_seen = set()
        while time.monotonic() < end:
            sequence += 1
            choice = randomizer.randrange(5)
            kinds_seen.add(choice)
            if choice == 0:
                spec, entity = (
                    CLOCK_OBSERVATION_V1_SPEC,
                    _observation(sequence=sequence),
                )
            elif choice == 1:
                spec, entity = (
                    MEASUREMENT_PROFILE_V1_SPEC,
                    _measurement(sequence=sequence, message=sequence, sample=sequence),
                )
            elif choice == 2:
                spec, entity = (
                    PROFILE_ONLY_V1_SPEC,
                    ProfileOnlyUnitV1(_profile(sequence=sequence)),
                )
            elif choice == 3:
                spec, entity = DIAGNOSTIC_V1_SPEC, _diagnostic(sequence=sequence)
            else:
                spec, entity = (
                    RECEIVER_HEALTH_REQUEST_V1_SPEC,
                    _health_request(sequence=sequence),
                )
            reserved = owner.queue.try_reserve_one(spec)
            if reserved.status is AdmissionResult.RESERVED:
                reserved.reservation.publish(entity)
                accepted += 1
            else:
                rejected += 1
            start = time.perf_counter_ns()
            loaded = owner.control.load_communicator_state(
                deadline_monotonic_us=deadline(owner)
            )
            assert loaded.status in (L.LOADED, L.STATE_UNAVAILABLE)
            generation = loaded.state.generation if loaded.state else 0
            result = owner.control.commit_communicator_state(
                replace(synthetic(), generation=generation + 1),
                deadline_monotonic_us=deadline(owner),
            )
            assert result.disposition.name in ("COMMITTED", "NOT_INSTALLED")
            latencies.append((time.perf_counter_ns() - start) // 1000)
        backend.armed = False
        owner.queue.close()
        if not owner.queue.snapshot().closed_and_drained:
            assert drained.wait(10)
        loaded = owner.control.load_communicator_state(
            deadline_monotonic_us=deadline(owner)
        )
        assert loaded.status is L.LOADED
        generation = loaded.state.generation
        marker = ReceiverCleanStopV1(
            INSTANCE, owner._clock.now_monotonic_us(), generation
        )
        assert (
            owner.control.commit_receiver_clean_stop(
                marker, deadline_monotonic_us=deadline(owner)
            ).disposition
            is D.COMMITTED
        )
        owner.request_stop(deadline_monotonic_us=deadline(owner, 10))
        owner.join(15)
        assert not owner.is_alive()
        assert accepted >= 50 and generation >= 10 and backend.fault_count > 0
        assert kinds_seen == set(range(5))
        assert (
            max(latencies) <= 10_500_000
        )  # Two serial 5-second control deadlines plus caller tolerance.
        assert owner._ordinary.counters.batch_entities_committed == accepted
        with sqlite3.connect(path) as db:
            assert db.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
            assert db.execute("PRAGMA foreign_key_check").fetchall() == []
            assert db.execute(
                "SELECT clean_stop_state_generation FROM receiver_instances"
            ).fetchone() == (generation,)
        restarted = CheckedPersistenceWorker(
            instance=ReceiverInstanceStart(
                bytes.fromhex("00112233445546778899aabbccddee02"),
                owner._clock.now_monotonic_us(),
            ),
            database_path=path,
            configuration_path=tmp_path / "test-group.json",
            boot_id_path=tmp_path / "boot-id",
            clock=LinuxOsClock(),
        )
        restarted.start()
        try:
            assert (
                restarted.wait_started(
                    deadline_monotonic_us=deadline(restarted, 10)
                ).database_failure
                is None
            )
            assert (
                restarted.control.load_communicator_state(
                    deadline_monotonic_us=deadline(restarted)
                ).state.generation
                == generation
            )
            assert restarted.queue.snapshot().published_entities == 0
        finally:
            restarted.finish_test()
    finally:
        load.terminate()
        try:
            load.wait(10)
        except subprocess.TimeoutExpired:
            load.kill()
            load.wait(10)
        load.stdout.close()
        try:
            if owner is not None:
                owner.finish_test()
        finally:
            evidence(
                tmp_path,
                seed=seed,
                duration_seconds=30,
                accepted=accepted,
                rejected=rejected,
                iterations=sequence,
                control_pair_latency_us=latencies,
                commit_us=commits_us,
                checkpoint_us=checkpoint_us,
                injected_failures=backend.fault_count,
                load="one process: SHA256 CPU loop and fsync of a dedicated 64 KiB file",
            )
