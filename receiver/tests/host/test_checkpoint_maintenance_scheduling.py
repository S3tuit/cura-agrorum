"""Real owner-thread SQLite coverage, using manual time and named boundaries."""

from dataclasses import replace
from queue import Queue
import sqlite3
from threading import Event, current_thread

import pytest

from cura_receiver.generated.receiver_enums_generated import (
    AdmissionResult,
    PersistenceAdmissionState as State,
)
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC, ProfileOnlyUnitV1
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as Commit,
    ReceiverCleanStopV1,
    ReceiverCleanStopCommitDisposition as Stop,
)
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import INSTANCE, _profile
from tests.support.builders.persistence_control import synthetic
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
from tests.support.coordination.threads import start_checked_threads, join_checked_threads
from tests.support.fakes.os_clock import FakeOsClock


class ManualWorker(CheckedPersistenceWorker):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.waits = Queue()
        self.trace = []

    def _wait_for_work(self, timeout):
        self.waits.put(timeout)
        assert self._wake.wait(5), "manual-clock boundary was not released"

    def _dispatch_work(self, action):
        self.trace.append((action, self.queue.snapshot().published_entities))
        super()._dispatch_work(action)

    def _dispatch_control(self, command):
        self.trace.append(("control", self.queue.snapshot().published_entities))
        super()._dispatch_control(command)

    def wait(self):
        return self.waits.get(timeout=5)

    def advance(self, elapsed_us):
        self._clock.advance_elapsed_us(elapsed_us)
        self._wake.set()
        return self.wait()


@pytest.fixture
def create(worker_files):
    owners = []

    def build(cls=ManualWorker, **kwargs):
        path, config, boot = worker_files
        owner = cls(
            instance=ReceiverInstanceStart(INSTANCE, 0),
            database_path=path,
            configuration_path=config,
            boot_id_path=boot,
            clock=FakeOsClock(monotonic_us=100),
            wake_threshold_entities=kwargs.pop("wake_threshold_entities", 1),
            **kwargs,
        )
        owners.append(owner)
        owner.start()
        assert owner.wait_started(deadline_monotonic_us=5_000_100).database_failure is None
        assert owner.wait() == owner._checkpoint.interval_us / 1_000_000
        return owner

    yield build
    for owner in owners:
        owner.finish_test()


def publish(owner, sequence=1, *, poison=False):
    profile = _profile(sequence=sequence)
    if poison:
        profile = replace(profile, busy_wait_count=-1)
    reservation = owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert reservation.status is AdmissionResult.RESERVED
    reservation.reservation.publish(ProfileOnlyUnitV1(profile))
    return owner.wait()


def finish_startup_checkpoint(owner):
    assert owner.advance(owner._checkpoint.interval_us) is None
    assert not owner._checkpoint.pending
    assert owner._recovery.counters.wal_checkpoint_attempts == 1


# Inherited large WAL allocation remains large after copying, but provides no wake eligibility.
def test_completed_large_wal_has_no_idle_checkpoint_or_deadline(create, worker_files):
    path = worker_files[0]
    inherited = sqlite3.connect(path, isolation_level=None)
    inherited.setconfig(sqlite3.SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE, True)
    inherited.execute("PRAGMA journal_mode=WAL")
    inherited.execute("PRAGMA wal_autocheckpoint=0")
    # The raw state envelope deliberately accepts malformed application state;
    # this creates inherited WAL without adding a non-production schema table.
    inherited.execute(
        "INSERT INTO communicator_state VALUES (1, 1, 1, zeroblob(?), NULL)",
        (5 * 1024 * 1024,),
    )
    owner = create()
    try:
        wal = path.with_name(path.name + "-wal")
        assert wal.stat().st_size > 4 * 1024 * 1024
        finish_startup_checkpoint(owner)
        assert wal.stat().st_size > 4 * 1024 * 1024
        # Retained allocation can grow without any new SQLite transaction.
        with wal.open("r+b") as allocation:
            allocation.truncate(wal.stat().st_size + 1024 * 1024)
        for elapsed in (5_000_000, 60_000_000, 3_600_000_000):
            assert owner.advance(elapsed) is None
            assert owner._checkpoint.deadline_monotonic_us is None
            assert owner._recovery.counters.wal_checkpoint_attempts == 1
        assert owner.queue.snapshot().admission_snapshot.state is State.AVAILABLE
    finally:
        inherited.close()


# First small-WAL commit arms its own interval; subsequent writes cannot postpone it.
def test_small_wal_first_commit_deadline_independent_of_flush(create, worker_files):
    owner = create(flush_interval_us=90_000_000, checkpoint_interval_us=2_000_000)
    finish_startup_checkpoint(owner)
    assert publish(owner) == 2.0
    due = 4_000_100
    assert owner._checkpoint.deadline_monotonic_us == due
    assert worker_files[0].with_name("worker.db-wal").stat().st_size < 4 * 1024 * 1024
    assert owner.advance(1_000_000) == 1.0
    assert publish(owner, 2) == 1.0
    assert owner._checkpoint.deadline_monotonic_us == due
    assert owner.advance(999_999) == 0.000001
    assert owner._recovery.counters.wal_checkpoint_attempts == 1
    assert owner.advance(1) is None
    assert owner._recovery.counters.wal_checkpoint_attempts == 2


# Reads, exact idempotent controls and publication before flushing are not new WAL work.
def test_reads_idempotence_and_uncommitted_publication_do_not_arm(create):
    owner = create(wake_threshold_entities=64, flush_interval_us=90_000_000)
    finish_startup_checkpoint(owner)
    deadline = owner._clock.now_monotonic_us() + 5_000_000
    owner.control.load_communicator_state(deadline_monotonic_us=deadline)
    assert owner.wait() is None
    value = synthetic()
    assert owner.control.commit_communicator_state(
        value, deadline_monotonic_us=deadline
    ).disposition is Commit.COMMITTED
    assert owner.wait() == 5.0
    assert owner.advance(5_000_000) is None
    assert owner.control.commit_communicator_state(
        value, deadline_monotonic_us=owner._clock.now_monotonic_us() + 5_000_000
    ).disposition is Commit.ALREADY_COMMITTED
    assert owner.wait() is None
    reservation = owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    reservation.reservation.publish(ProfileOnlyUnitV1(_profile()))
    assert owner.wait() == 80.0  # ordinary flush has its separate deadline
    assert not owner._checkpoint.pending


# Both non-error maintenance and recovery can be partial without closing admission indefinitely.
@pytest.mark.parametrize("recovering", [False, True])
def test_reader_partial_retries_without_new_writes(create, worker_files, recovering):
    class Backend(SqliteTransactions):
        fail = False

        def checkpoint(self, db):
            if self.fail:
                self.fail = False
                raise OSError(5, "checkpoint storage fault")
            return super().checkpoint(db)

    backend = Backend()
    owner = create(transactions=backend)
    finish_startup_checkpoint(owner)
    reader = sqlite3.connect(worker_files[0], isolation_level=None)
    try:
        reader.execute("BEGIN")
        reader.execute("SELECT count(*) FROM message_profiles").fetchone()
        assert publish(owner) == 5.0
        if recovering:
            backend.fail = True
            assert owner.advance(5_000_000) == 0.250925
            assert owner.queue.snapshot().admission_snapshot.state is State.UNAVAILABLE_IO
            assert owner.advance(250925) == 5.0
        else:
            assert owner.advance(5_000_000) == 5.0
        assert owner._checkpoint.pending
        assert not owner._ordinary.checkpoint_pending
        assert owner._recovery.retry_deadline_monotonic_us is None
        assert owner.queue.snapshot().admission_snapshot.state is State.AVAILABLE
        attempts = owner._recovery.counters.wal_checkpoint_attempts
        assert owner.advance(4_999_999) == 0.000001
        assert owner._recovery.counters.wal_checkpoint_attempts == attempts
        reader.close()
        assert owner.advance(1) is None
        assert owner._recovery.counters.wal_checkpoint_attempts == attempts + 1
        assert not owner._checkpoint.pending
    finally:
        reader.close()


# Quarantine is a writer even when the malformed profile cannot commit an ordinary row.
def test_quarantine_commit_arms_maintenance(create, worker_files):
    owner = create()
    finish_startup_checkpoint(owner)
    assert publish(owner, poison=True) == 5.0
    assert owner._checkpoint.pending
    assert owner._recovery.counters.durable_quarantine_successes == 1
    with sqlite3.connect(worker_files[0]) as db:
        assert db.execute("SELECT count(*) FROM message_profiles").fetchone() == (0,)
        assert db.execute("SELECT count(*) FROM quarantined_entities").fetchone() == (1,)
    assert owner.advance(5_000_000) is None


# A definite failure before COMMIT retains FIFO recovery without claiming a possible write.
def test_precommit_rollback_does_not_arm_maintenance(create):
    class Backend(SqliteTransactions):
        def begin(self, db):
            super().begin(db)
            raise OSError(5, "failure before COMMIT")

    owner = create(transactions=Backend())
    finish_startup_checkpoint(owner)
    assert publish(owner) == 0.250925
    assert not owner._checkpoint.pending
    assert owner._checkpoint.deadline_monotonic_us is None


# A single real caller continuously resubmits controls while ordinary FIFO remains nonempty.
@pytest.mark.parametrize("cross_during_control", [False, True])
def test_due_checkpoint_gets_turn_under_ordinary_and_control_load(
    create, worker_files, cross_during_control
):
    release_idle, submitted = Event(), Queue()

    class Scheduled(ManualWorker):
        def _wait_for_work(self, timeout):
            self.waits.put(timeout)
            assert release_idle.wait(5)
            assert self._wake.wait(5)

        def _dispatch_control(self, command):
            assert not self._database.connection.in_transaction
            # F-001: after the first ordinary turn, a control bypasses another
            # selected ordinary turn one microsecond before maintenance is due.
            if cross_during_control and sum(kind == "control" for kind, _ in self.trace) == 1:
                self._clock.advance_elapsed_us(1)
            super()._dispatch_control(command)
            if sum(kind == "control" for kind, _ in self.trace) < 4:
                submitted.get(timeout=5)  # next request is queued at this boundary

        def _dispatch_work(self, action):
            assert not self._database.connection.in_transaction
            super()._dispatch_work(action)

    owner = create(Scheduled, batch_limit_entities=1)
    original_wait = owner.control._wait_for_completion

    def completion(command, remaining):
        submitted.put(None)
        return original_wait(command, remaining)

    owner.control._wait_for_completion = completion

    def caller():
        for sequence in range(1, 9):
            reservation = owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
            reservation.reservation.publish(ProfileOnlyUnitV1(_profile(sequence=sequence)))
        for _ in range(4):
            owner.control.load_communicator_state(deadline_monotonic_us=50_000_100)

    threads = start_checked_threads([("continuous-control-caller", caller)])
    try:
        submitted.get(timeout=5)
        owner._clock.advance_elapsed_us(5_000_000 - int(cross_during_control))
        release_idle.set()
        join_checked_threads(threads)
        owner.wait()
        expected = (
            ["control", "ordinary", "control", "ordinary", "control", "checkpoint", "control"]
            if cross_during_control else
            ["control", "ordinary", "control", "checkpoint", "control", "ordinary", "control"]
        )
        assert [kind for kind, _ in owner.trace[:7]] == expected
        assert owner.trace[5 if cross_during_control else 3] == (
            "checkpoint", 6 if cross_during_control else 7
        )
        assert owner._recovery.counters.wal_checkpoint_attempts == 1
        assert owner._checkpoint.deadline_monotonic_us == 10_000_100
        with sqlite3.connect(worker_files[0]) as db:
            assert db.execute("SELECT count(*) FROM message_profiles").fetchone() == (8,)
    finally:
        release_idle.set()


# The outer observation must survive operation overrides that do not invoke super().
@pytest.mark.parametrize("writer", ["ordinary", "quarantine", "state", "clean_stop"])
@pytest.mark.parametrize("committed", [False, True])
def test_unknown_commit_keeps_maintenance(create, writer, committed):
    class LostReply(SqliteTransactions):
        def commit(self, db):
            if committed:
                super().commit(db)
            raise OSError(5, "COMMIT result unavailable")

    owner = create(transactions=LostReply())
    finish_startup_checkpoint(owner)
    if writer in ("ordinary", "quarantine"):
        assert publish(owner, poison=writer == "quarantine") == 0.250925
    elif writer == "state":
        result = owner.control.commit_communicator_state(
            synthetic(), deadline_monotonic_us=owner._clock.now_monotonic_us() + 5_000_000
        )
        assert result.disposition is Commit.OUTCOME_UNKNOWN
        assert owner.wait() == 0.250925
    else:
        owner.queue.close()
        assert owner.wait() is None
        result = owner.control.commit_receiver_clean_stop(
            ReceiverCleanStopV1(INSTANCE, owner._clock.now_monotonic_us(), 0),
            deadline_monotonic_us=owner._clock.now_monotonic_us() + 5_000_000,
        )
        assert result.disposition is Stop.OUTCOME_UNKNOWN
        assert owner.wait() == 0.250925
    assert owner._checkpoint.pending
    assert owner._checkpoint.deadline_monotonic_us == owner._clock.now_monotonic_us() + 5_000_000
    assert owner._recovery.counters.wal_checkpoint_attempts == 1


# Reopening the same file must seed coverage even when the original commit was already copied.
def test_connection_reopen_seeds_unknown_coverage(create, monkeypatch):
    from cura_receiver.sqlite_database import ReceiverDatabase

    original = ReceiverDatabase.revalidate
    seed_observed = []

    def reopen(database, **kwargs):
        assert current_thread() is owner
        database.close()
        return original(database, **kwargs)

    class Backend(SqliteTransactions):
        def checkpoint(self, db):
            seed_observed.append(owner._checkpoint.pending)
            return super().checkpoint(db)

    class ClosedOnRead(ManualWorker):
        armed = False

        def _dispatch_control(self, command):
            if self.armed:
                self.armed = False
                self._database.close()
            super()._dispatch_control(command)

    owner = create(ClosedOnRead, transactions=Backend())
    finish_startup_checkpoint(owner)
    monkeypatch.setattr(ReceiverDatabase, "revalidate", reopen)
    owner.armed = True
    owner.control.load_communicator_state(
        deadline_monotonic_us=owner._clock.now_monotonic_us() + 5_000_000
    )
    assert owner.wait() == 0.250925
    assert not owner._checkpoint.pending
    assert owner.advance(250925) is None
    assert seed_observed == [True, True]
    assert owner.queue.snapshot().admission_snapshot.state is State.AVAILABLE


# Busy, malformed and negative/sentinel results all retain work and pace recovery.
@pytest.mark.parametrize(
    "result", [None, (), (0, 1), (0, 1, 1, 1), (False, 0, 0), (0, -1, -1),
               (0, 1, -1), (0, 1, 2), (0, "1", 1), (1, -1, -1)]
)
def test_unusable_checkpoint_results_fail_closed(create, result):
    class Backend(SqliteTransactions):
        def checkpoint(self, db):
            return result

    owner = create(transactions=Backend())
    assert owner.advance(5_000_000) == 0.250925
    assert owner._checkpoint.pending and owner._ordinary.checkpoint_pending
    assert owner.queue.snapshot().admission_snapshot.state is State.UNAVAILABLE_IO
    assert owner._recovery.counters.wal_checkpoint_successes == 0
    assert owner.advance(250924) == 0.000001
    assert owner._recovery.counters.wal_checkpoint_attempts == 1
    assert owner.advance(1) == 0.50185
    assert owner._recovery.counters.wal_checkpoint_attempts == 2


# A zero-frame result disarms coverage, just like a valid positive equal count.
def test_zero_frame_completion_disarms(create):
    class Backend(SqliteTransactions):
        def checkpoint(self, db):
            result = super().checkpoint(db)
            # Perform real copying before injecting the zero-frame result at
            # the exact operation boundary; stored rows must still survive.
            assert result[0] == 0
            return (0, 0, 0)

    owner = create(transactions=Backend())
    finish_startup_checkpoint(owner)
    assert owner.advance(30_000_000) is None


# A clean marker is a writer; one reader-limited final attempt cannot erase its durability.
def test_partial_final_checkpoint_retains_durable_clean_marker(create, worker_files):
    owner = create()
    finish_startup_checkpoint(owner)
    reader = sqlite3.connect(worker_files[0], isolation_level=None)
    try:
        reader.execute("BEGIN")
        reader.execute("SELECT * FROM receiver_instances").fetchall()
        owner.queue.close()
        result = owner.control.commit_receiver_clean_stop(
            ReceiverCleanStopV1(INSTANCE, owner._clock.now_monotonic_us(), 0),
            deadline_monotonic_us=owner._clock.now_monotonic_us() + 5_000_000,
        )
        assert result.disposition is Stop.COMMITTED
        owner.request_stop(deadline_monotonic_us=owner._clock.now_monotonic_us() + 1_000_000)
        owner.join(5)
        assert not owner.is_alive()
        assert owner._checkpoint.pending
        assert owner._recovery.counters.wal_checkpoint_attempts == 2
        with sqlite3.connect(worker_files[0]) as db:
            assert db.execute("SELECT clean_stopped_at_monotonic_us FROM receiver_instances").fetchone() == (5_000_100,)
    finally:
        reader.close()
