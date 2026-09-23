import sqlite3
from queue import Queue
from threading import Event, current_thread

import pytest

from cura_receiver.generated.receiver_enums_generated import (
    PersistenceAdmissionState as State,
)
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC, ProfileOnlyUnitV1
from cura_receiver.persistence_control_values import (
    ReceiverCleanStopV1,
    ReceiverCleanStopCommitDisposition as D,
    ReceiverCleanStopCommitFailureKind as F,
)
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.sqlite_database import ReceiverDatabase
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import INSTANCE, _profile
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
from tests.support.coordination.threads import (
    start_checked_threads,
    join_checked_threads,
)
from tests.support.fakes.os_clock import FakeOsClock


def worker(paths, cls=CheckedPersistenceWorker, **kwargs):
    path, config, boot = paths
    return cls(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=path,
        configuration_path=config,
        boot_id_path=boot,
        clock=FakeOsClock(monotonic_us=100),
        **kwargs,
    )


def start(owner):
    owner.start()
    assert owner.wait_started(deadline_monotonic_us=5_000_100)


def marker_rows(path):
    with sqlite3.connect(path) as db:
        return db.execute(
            "SELECT clean_stopped_at_monotonic_us, clean_stop_state_generation FROM receiver_instances"
        ).fetchall()


# Only the caller's durable marker establishes clean stop; later checkpoint/close failures preserve it.
@pytest.mark.parametrize("failure", ["none", "checkpoint", "close"])
def test_final_checkpoint_close_order(worker_files, monkeypatch, failure):
    trace = []

    class Final(SqliteTransactions):
        def checkpoint(self, db):
            assert current_thread() is owner
            assert owner.queue.snapshot().closed_and_drained
            assert db.execute(
                "SELECT clean_stopped_at_monotonic_us FROM receiver_instances"
            ).fetchone() == (100,)
            trace.append("checkpoint")
            if failure == "checkpoint":
                raise OSError(5, "final checkpoint fault")
            return super().checkpoint(db)

    original_close = ReceiverDatabase.close

    def close(db):
        assert current_thread() is owner
        trace.append("close")
        original_close(db)
        if failure == "close":
            raise OSError(5, "final close fault")

    monkeypatch.setattr(ReceiverDatabase, "close", close)
    owner = worker(worker_files, transactions=Final())
    start(owner)
    try:
        owner.queue.close()
        result = owner.control.commit_receiver_clean_stop(
            ReceiverCleanStopV1(INSTANCE, 100, 0), deadline_monotonic_us=5_000_100
        )
        assert result.disposition is D.COMMITTED
        owner.request_stop(deadline_monotonic_us=1_000_100)
        owner.join(5)
        assert not owner.is_alive()
        assert trace == ["checkpoint", "close"]
        assert marker_rows(worker_files[0]) == [(100, 0)]
        assert owner._recovery.counters.wal_checkpoint_failures == (
            failure == "checkpoint"
        )
    finally:
        if failure == "close":
            with pytest.raises(OSError, match="final close fault"):
                owner.finish_test()
        else:
            owner.finish_test()


# An unresolved reservation can publish or cancel after closure; final storage work waits for it.
@pytest.mark.parametrize("resolution", ["publish", "cancel", "expire"])
def test_shutdown_outstanding_reservation(worker_files, resolution):
    waits = Queue()

    class Observed(CheckedPersistenceWorker):
        def _wait_for_work(self, timeout):
            waits.put(timeout)
            assert self._wake.wait(5)

    owner = worker(worker_files, Observed)
    start(owner)
    try:
        assert waits.get(timeout=5) is None
        reservation = owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation
        owner.request_stop(deadline_monotonic_us=1_000_100)
        assert waits.get(timeout=5) == 1.0
        assert owner.queue.snapshot().reserved_entities == 1
        assert not owner.queue.snapshot().closed_and_drained
        if resolution == "publish":
            reservation.publish(ProfileOnlyUnitV1(_profile()))
        elif resolution == "cancel":
            reservation.cancel()
        else:
            owner._clock.advance_elapsed_us(1_000_000)
            owner._wake.set()
        owner.join(5)
        assert not owner.is_alive()
        assert owner.queue.snapshot().closed_and_drained == (resolution != "expire")
        assert marker_rows(worker_files[0]) == [(None, None)]
        with sqlite3.connect(worker_files[0]) as db:
            assert db.execute("SELECT count(*) FROM message_profiles").fetchone() == (
                int(resolution == "publish"),
            )
        assert owner._recovery.counters.wal_checkpoint_attempts == int(
            resolution != "expire"
        )
    finally:
        owner.finish_test()


# Failed marker COMMIT remains unknown and cannot be upgraded to clean by final checkpointing.
@pytest.mark.parametrize("committed", [False, True])
def test_unknown_marker_then_checkpoint_recovery(worker_files, committed):
    waits = Queue()

    class Unknown(SqliteTransactions):
        def commit(self, db):
            if committed:
                super().commit(db)
            raise OSError(5, "marker commit reply lost")

    class Observed(CheckedPersistenceWorker):
        def _wait_for_work(self, timeout):
            waits.put(timeout)
            assert self._wake.wait(5)

    owner = worker(worker_files, Observed, transactions=Unknown())
    start(owner)
    try:
        waits.get(timeout=5)
        owner.queue.close()
        result = owner.control.commit_receiver_clean_stop(
            ReceiverCleanStopV1(INSTANCE, 100, 0), deadline_monotonic_us=5_000_100
        )
        assert (
            result.disposition is D.OUTCOME_UNKNOWN
            and result.failure_kind is F.DATABASE_ERROR
        )
        # Queue closure may itself have produced an idle observation before submission.
        while waits.get(timeout=5) is None:
            pass
        owner.request_stop(deadline_monotonic_us=1_000_100)
        assert waits.get(timeout=5) == 0.250925
        assert owner._recovery.counters.wal_checkpoint_attempts == 0
        owner._clock.advance_elapsed_us(250925)
        owner._wake.set()
        owner.join(5)
        assert not owner.is_alive()
        assert owner._recovery.counters.wal_checkpoint_successes == 1
        assert marker_rows(worker_files[0]) == (
            [(100, 0)] if committed else [(None, None)]
        )
    finally:
        owner.finish_test()


# Startup I/O may finish after the stop request, but a closed queue never admits new work.
def test_stop_during_startup(worker_files, monkeypatch):
    import cura_receiver.receiver_startup as startup

    arrived, release = Event(), Event()
    original = startup.open_receiver_database

    def open_db(*args, **kwargs):
        arrived.set()
        assert release.wait(5)
        return original(*args, **kwargs)

    monkeypatch.setattr(startup, "open_receiver_database", open_db)
    owner = worker(worker_files)
    owner.start()
    try:
        assert arrived.wait(5)
        owner.request_stop(deadline_monotonic_us=0)
        assert owner.queue.snapshot().closed
        assert (
            owner.queue.snapshot().admission_snapshot.state
            is State.UNAVAILABLE_STARTING
        )
        release.set()
        owner.join(5)
        assert not owner.is_alive()
        assert marker_rows(worker_files[0]) == [(None, None)]
    finally:
        release.set()
        owner.finish_test()


# An expired stop budget cannot preempt an open commit; only the confirmed durable prefix is removed.
@pytest.mark.parametrize("phase", ["begin", "commit", "rollback"])
def test_stop_open_ordinary_transaction(worker_files, phase):
    arrived, release = Event(), Event()

    class Gated(SqliteTransactions):
        def begin(self, db):
            super().begin(db)
            if phase == "begin":
                arrived.set()
                assert release.wait(5)

        def commit(self, db):
            if phase == "commit":
                arrived.set()
                assert release.wait(5)
            if phase == "rollback":
                raise OSError(5, "unknown commit")
            super().commit(db)

        def rollback(self, db):
            arrived.set()
            assert release.wait(5)
            super().rollback(db)

    owner = worker(worker_files, transactions=Gated(), wake_threshold_entities=1)
    start(owner)
    owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
        ProfileOnlyUnitV1(_profile())
    )
    try:
        assert arrived.wait(5)
        owner.request_stop(deadline_monotonic_us=0)
        assert (
            owner.is_alive()
        )  # Named open-operation barrier, not a timing assumption.
        release.set()
        owner.join(5)
        assert not owner.is_alive()
        assert owner.queue.snapshot().published_entities == int(phase == "rollback")
        assert owner._recovery.counters.wal_checkpoint_attempts == 0
        assert marker_rows(worker_files[0]) == [(None, None)]
    finally:
        release.set()
        owner.finish_test()


# Stopping in backoff retains unknown work; stopping during reconciliation waits for exact completion.
@pytest.mark.parametrize("phase", ["backoff", "reconciliation"])
def test_stop_unknown_ordinary_work(worker_files, phase):
    waits = Queue()
    arrived, release = Event(), Event()

    class UnknownOnce(SqliteTransactions):
        commits = 0

        def begin(self, db):
            super().begin(db)
            if self.commits:
                arrived.set()
                assert release.wait(5)

        def commit(self, db):
            super().commit(db)
            self.commits += 1
            if self.commits == 1:
                raise OSError(5, "ordinary commit reply lost")

    class Observed(CheckedPersistenceWorker):
        def _wait_for_work(self, timeout):
            waits.put(timeout)
            assert self._wake.wait(5)

    owner = worker(
        worker_files, Observed, transactions=UnknownOnce(), wake_threshold_entities=1
    )
    start(owner)
    try:
        assert waits.get(timeout=5) is None
        owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
            ProfileOnlyUnitV1(_profile())
        )
        assert waits.get(timeout=5) == 0.250925
        assert owner.queue.snapshot().claimed_entities == 1
        if phase == "reconciliation":
            owner._clock.advance_elapsed_us(250925)
            owner._wake.set()
            assert arrived.wait(5)
        owner.request_stop(deadline_monotonic_us=0)
        release.set()
        owner.join(5)
        assert not owner.is_alive()
        assert owner.queue.snapshot().published_entities == int(phase == "backoff")
        with sqlite3.connect(worker_files[0]) as db:
            assert db.execute("SELECT count(*) FROM message_profiles").fetchone() == (
                1,
            )
        assert marker_rows(worker_files[0]) == [(None, None)]
    finally:
        release.set()
        owner.finish_test()


# Final stop closes later submissions while an already running control reaches its own safe boundary.
@pytest.mark.parametrize("phase", ["precommit", "commit"])
def test_stop_during_control_operation(worker_files, phase):
    from cura_receiver.persistence_control_values import (
        CommunicatorStateCommitDisposition as SD,
    )
    from tests.support.builders.persistence_control import synthetic

    arrived, release = Event(), Event()
    results = []

    class Gated(SqliteTransactions):
        def begin(self, db):
            super().begin(db)
            if phase == "precommit":
                arrived.set()
                assert release.wait(5)

        def commit(self, db):
            if phase == "commit":
                arrived.set()
                assert release.wait(5)
            super().commit(db)

    owner = worker(worker_files, transactions=Gated())
    start(owner)
    callers = start_checked_threads(
        [
            (
                "stopping-control-caller",
                lambda: results.append(
                    owner.control.commit_communicator_state(
                        synthetic(), deadline_monotonic_us=5_000_100
                    )
                ),
            )
        ]
    )
    try:
        assert arrived.wait(5)
        owner.request_stop(deadline_monotonic_us=0)
        assert results == []
        release.set()
        join_checked_threads(callers)
        owner.join(5)
        assert not owner.is_alive()
        assert results[0].disposition is SD.COMMITTED
        assert marker_rows(worker_files[0]) == [(None, None)]
    finally:
        release.set()
        owner.finish_test()
