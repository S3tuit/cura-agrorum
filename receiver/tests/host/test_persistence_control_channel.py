import sqlite3
from queue import Queue
from threading import Event, TIMEOUT_MAX

import pytest

from cura_receiver import persistence_control_channel
from cura_receiver.persistence_control_execution import (
    ControlCommand,
    ControlRequest,
    ControlCommandKind as Kind,
)
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as D,
    CommunicatorStateCommitFailureKind as F,
    CommunicatorStateCondition as Condition,
    ReceiverCleanStopV1,
    ReceiverCleanStopCommitDisposition as SD,
)
from cura_receiver.persistence_worker import PersistenceWorker
from cura_receiver.receiver_configuration import (
    PersistenceControlInterfaceViolation as Violation,
    ReceiverConfigurationLoadStatus,
)
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.sqlite_transactions import SqliteTransactions
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC, ProfileOnlyUnitV1
from tests.support.builders.persistence import INSTANCE, _profile
from tests.support.builders.persistence_control import synthetic
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.coordination.threads import (
    start_checked_threads,
    join_checked_threads,
)
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker


def worker(paths, cls=PersistenceWorker, **kwargs):
    path, config, boot = paths
    return cls(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=path,
        configuration_path=config,
        boot_id_path=boot,
        clock=FakeOsClock(monotonic_us=100),
        **kwargs,
    )


def stop(owner):
    owner.request_stop(deadline_monotonic_us=0)
    owner.join(5)
    assert not owner.is_alive()


# The four public operations return their exact outcomes and remain available after queue closure.
def test_public_control_operations(worker_files):
    owner = worker(worker_files)
    owner.start()
    try:
        config = owner.control.load_receiver_configuration(
            deadline_monotonic_us=5_000_100
        )
        assert config.status is ReceiverConfigurationLoadStatus.LOADED
        assert "group_master_key" not in repr(config)
        assert (
            owner.control.load_communicator_state(
                deadline_monotonic_us=5_000_100
            ).state_condition
            is Condition.MISSING
        )
        assert (
            owner.control.commit_communicator_state(
                synthetic(), deadline_monotonic_us=5_000_100
            ).disposition
            is D.COMMITTED
        )
        owner.queue.close()
        assert (
            owner.control.commit_receiver_clean_stop(
                ReceiverCleanStopV1(INSTANCE, 100, 1), deadline_monotonic_us=5_000_100
            ).disposition
            is SD.COMMITTED
        )
        assert (
            owner.control.load_communicator_state(deadline_monotonic_us=5_000_100).state
            == synthetic()
        )
        owner.control.close()
        assert (
            owner.control.commit_communicator_state(
                synthetic(), deadline_monotonic_us=5_000_100
            ).failure_kind
            is F.CHANNEL_CLOSED
        )
    finally:
        stop(owner)


# The caller side binds to one thread and rejects invalid deadlines before submission or I/O.
def test_control_caller_and_deadline_validation(worker_files):
    owner = worker(worker_files)
    assert (
        owner.control.load_communicator_state(
            deadline_monotonic_us=True
        ).interface_violation
        is Violation.INVALID_DEADLINE
    )
    results = []
    threads = start_checked_threads(
        [
            (
                "wrong-control-caller",
                lambda: results.append(
                    owner.control.load_communicator_state(deadline_monotonic_us=1000)
                ),
            )
        ]
    )
    join_checked_threads(threads)
    assert results[0].interface_violation is Violation.WRONG_CALLER
    assert not owner._mailbox


# A pending control bypasses only one due ordinary attempt even when its caller immediately resubmits.
def test_control_ordinary_bounded_alternation(worker_files):
    release_idle, idle, submitted, trace = Event(), Event(), Queue(), []

    class Scheduled(PersistenceWorker):
        def _wait_for_work(self, timeout):
            idle.set()
            assert release_idle.wait(5)
            super()._wait_for_work(timeout)

        def _dispatch_control(self, command):
            trace.append("control")
            super()._dispatch_control(command)
            if trace.count("control") < 4:
                submitted.get(
                    timeout=5
                )  # Next call is queued before this safe boundary returns.

        def _dispatch_work(self, action):
            trace.append(action)
            super()._dispatch_work(action)

    owner = worker(
        worker_files, Scheduled, wake_threshold_entities=1, batch_limit_entities=1
    )
    original_wait = owner.control._wait_for_completion

    def completion(command, remaining):
        submitted.put(None)
        return original_wait(command, remaining)

    owner.control._wait_for_completion = completion
    owner.start()

    def caller():
        assert idle.wait(5)
        for sequence in range(1, 6):
            owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
                ProfileOnlyUnitV1(_profile(sequence=sequence))
            )
        for _ in range(4):
            assert (
                owner.control.load_communicator_state(
                    deadline_monotonic_us=5_000_100
                ).state_condition
                is Condition.MISSING
            )
        owner.queue.close()

    threads = start_checked_threads([("communicator-test-caller", caller)])
    try:
        submitted.get(timeout=5)
        release_idle.set()
        join_checked_threads(threads)
        # A final command completes only after both the previous request and one ordinary turn.
        assert trace[:7] == [
            "control",
            "ordinary",
            "control",
            "ordinary",
            "control",
            "ordinary",
            "control",
        ]
    finally:
        release_idle.set()
        stop(owner)


# Timeouts before and after COMMIT keep later loads behind the original command's terminal outcome.
@pytest.mark.parametrize(
    "phase", ["precommit", "before_commit_backend", "after_commit_backend"]
)
def test_deadline_and_serialized_reconciliation(worker_files, phase):
    arrived, release, second_submitted = Event(), Event(), Event()

    class Gated(SqliteTransactions):
        def begin(self, connection):
            super().begin(connection)
            if phase == "precommit":
                arrived.set()
                assert release.wait(5)

        def commit(self, connection):
            if phase == "before_commit_backend":
                arrived.set()
                assert release.wait(5)
            super().commit(connection)
            if phase == "after_commit_backend":
                arrived.set()
                assert release.wait(5)

    owner = worker(worker_files, transactions=Gated())
    original_wait = owner.control._wait_for_completion

    def completion(command, remaining):
        if command.request.kind is Kind.COMMIT_STATE:
            assert arrived.wait(5)
            owner._clock.advance_elapsed_us(1_000_000)
            return False  # Named deadline boundary; no host sleep chooses the race.
        second_submitted.set()
        return original_wait(command, remaining)

    owner.control._wait_for_completion = completion
    results = []
    owner.start()
    assert owner.wait_started(deadline_monotonic_us=5_000_100)

    def caller():
        results.append(
            owner.control.commit_communicator_state(
                synthetic(), deadline_monotonic_us=1_000_100
            )
        )
        results.append(
            owner.control.load_communicator_state(deadline_monotonic_us=6_000_100)
        )

    threads = start_checked_threads([("deadline-caller", caller)])
    try:
        assert second_submitted.wait(5)
        assert len(results) == 1
        assert results[0].failure_kind is F.DEADLINE_EXCEEDED
        assert results[0].disposition is (
            D.NOT_INSTALLED if phase == "precommit" else D.OUTCOME_UNKNOWN
        )
        release.set()
        join_checked_threads(threads)
        if phase == "precommit":
            assert results[1].state_condition is Condition.MISSING
        else:
            assert results[1].state == synthetic()
    finally:
        release.set()
        stop(owner)


# Control submission during an ordinary transaction cannot enter SQLite until that transaction finishes.
def test_no_ordinary_transaction_preemption(worker_files):
    arrived, release, submitted = Event(), Event(), Event()

    class Gated(SqliteTransactions):
        def begin(self, connection):
            super().begin(connection)
            arrived.set()
            assert release.wait(5)

    owner = worker(worker_files, transactions=Gated(), wake_threshold_entities=1)
    original_wait = owner.control._wait_for_completion

    def completion(command, remaining):
        submitted.set()
        return original_wait(command, remaining)

    owner.control._wait_for_completion = completion
    owner.start()
    assert owner.wait_started(deadline_monotonic_us=5_000_100)
    results = []

    def caller():
        owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
            ProfileOnlyUnitV1(_profile())
        )
        assert arrived.wait(5)
        results.append(
            owner.control.load_communicator_state(deadline_monotonic_us=5_000_100)
        )

    threads = start_checked_threads([("transaction-caller", caller)])
    try:
        assert submitted.wait(5)
        assert not results
        assert owner.queue.snapshot().claimed_entities == 1
        release.set()
        join_checked_threads(threads)
        assert results[0].state_condition is Condition.MISSING
        assert owner.queue.snapshot().published_entities == 0
    finally:
        release.set()
        stop(owner)


# A timed-out queued mutation is skipped before a later reconciliation load executes.
def test_queued_cancellation_keeps_serialization(worker_files):
    idle, release, second_submitted = Event(), Event(), Event()

    class HeldIdle(PersistenceWorker):
        def _wait_for_work(self, timeout):
            idle.set()
            assert release.wait(5)
            super()._wait_for_work(timeout)

    owner = worker(worker_files, HeldIdle)
    original_wait = owner.control._wait_for_completion

    def completion(command, remaining):
        if command.request.kind is Kind.COMMIT_STATE:
            owner._clock.advance_elapsed_us(1_000_000)
            return False
        second_submitted.set()
        return original_wait(command, remaining)

    owner.control._wait_for_completion = completion
    owner.start()
    assert idle.wait(5)
    results = []

    def caller():
        results.append(
            owner.control.commit_communicator_state(
                synthetic(), deadline_monotonic_us=1_000_100
            )
        )
        results.append(
            owner.control.load_communicator_state(deadline_monotonic_us=6_000_100)
        )

    threads = start_checked_threads([("queued-timeout-caller", caller)])
    try:
        assert second_submitted.wait(5)
        assert results[0].disposition is D.NOT_INSTALLED
        assert results[0].failure_kind is F.DEADLINE_EXCEEDED
        release.set()
        join_checked_threads(threads)
        assert results[1].state_condition is Condition.MISSING
    finally:
        release.set()
        stop(owner)


# A completion already installed under the command lock wins over a delayed timeout observer.
def test_done_result_wins_deadline_race(worker_files):
    owner = worker(worker_files)
    original_wait = owner.control._wait_for_completion

    def completion(command, remaining):
        assert original_wait(command, remaining)
        owner._clock.advance_elapsed_us(1_000_000)
        return False

    owner.control._wait_for_completion = completion
    owner.start()
    try:
        assert (
            owner.control.commit_communicator_state(
                synthetic(), deadline_monotonic_us=1_000_100
            ).disposition
            is D.COMMITTED
        )
    finally:
        stop(owner)


# Channel closure wakes the disk owner, which completes a queued command without executing it.
def test_channel_closure_of_queued_command(worker_files):
    idle, release, submitted = Event(), Event(), Event()

    class HeldIdle(PersistenceWorker):
        def _wait_for_work(self, timeout):
            idle.set()
            assert release.wait(5)
            super()._wait_for_work(timeout)

    owner = worker(worker_files, HeldIdle)
    original_wait = owner.control._wait_for_completion

    def completion(command, remaining):
        submitted.set()
        return original_wait(command, remaining)

    owner.control._wait_for_completion = completion
    owner.start()
    assert idle.wait(5)
    results = []
    threads = start_checked_threads(
        [
            (
                "closing-channel-caller",
                lambda: results.append(
                    owner.control.commit_communicator_state(
                        synthetic(), deadline_monotonic_us=5_000_100
                    )
                ),
            )
        ]
    )
    try:
        assert submitted.wait(5)
        owner.control.close()
        release.set()
        join_checked_threads(threads)
        assert (
            results[0].disposition is D.NOT_INSTALLED
            and results[0].failure_kind is F.CHANNEL_CLOSED
        )
        with sqlite3.connect(worker_files[0]) as observer:
            assert observer.execute("SELECT * FROM communicator_state").fetchall() == []
    finally:
        release.set()
        stop(owner)


# F-003: accepted deadlines around the platform wait ceiling and u64 maximum retain their result.
@pytest.mark.parametrize("kind", [Kind.COMMIT_STATE, Kind.CLEAN_STOP])
@pytest.mark.parametrize(
    "deadline",
    [int(TIMEOUT_MAX) * 1_000_000, (int(TIMEOUT_MAX) + 1) * 1_000_000, (1 << 64) - 1],
)
def test_large_deadline_during_ordinary_commit(worker_files, kind, deadline):
    arrived, release, waiting = Event(), Event(), Event()

    class Gated(SqliteTransactions):
        def commit(self, connection):
            arrived.set()
            assert release.wait(5), "ordinary commit was not released"
            super().commit(connection)

    owner = worker(
        worker_files,
        CheckedPersistenceWorker,
        transactions=Gated(),
        wake_threshold_entities=1,
    )
    original_wait = owner.control._wait_for_completion

    def completion(command, remaining):
        event_wait = command.completion.wait

        def checked_wait(timeout):
            waiting.set()
            assert 0 <= timeout <= TIMEOUT_MAX
            return event_wait(timeout)

        command.completion.wait = checked_wait
        return original_wait(command, remaining)

    owner.control._wait_for_completion = completion
    owner.start()
    results = []

    def caller():
        if kind is Kind.COMMIT_STATE:
            results.append(
                owner.control.commit_communicator_state(
                    synthetic(),
                    deadline_monotonic_us=deadline,
                )
            )
        else:
            owner.queue.close()
            results.append(
                owner.control.commit_receiver_clean_stop(
                    ReceiverCleanStopV1(INSTANCE, 100, 0),
                    deadline_monotonic_us=deadline,
                )
            )
        results.append(
            owner.control.load_communicator_state(deadline_monotonic_us=deadline)
        )

    try:
        assert owner.wait_started(deadline_monotonic_us=5_000_100)
        owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
            ProfileOnlyUnitV1(_profile())
        )
        assert arrived.wait(5)
        threads = start_checked_threads([("large-deadline-caller", caller)])
        assert waiting.wait(5)
        assert not results
        release.set()
        join_checked_threads(threads)
        if kind is Kind.COMMIT_STATE:
            assert results[0].disposition is D.COMMITTED
            assert results[1].state == synthetic()
        else:
            assert results[0].disposition is SD.COMMITTED
            assert results[1].state_condition is Condition.MISSING
            with sqlite3.connect(worker_files[0]) as observer:
                assert observer.execute(
                    "SELECT clean_stopped_at_monotonic_us, clean_stop_state_generation "
                    "FROM receiver_instances"
                ).fetchall() == [(100, 0)]
    finally:
        release.set()
        owner.finish_test()


# F-003: a platform-sized wait segment cannot expire the command before its absolute deadline.
@pytest.mark.parametrize("completed", [False, True])
def test_completion_wait_segments_preserve_absolute_deadline(
    worker_files,
    monkeypatch,
    completed,
):
    owner = worker(worker_files)
    command = ControlCommand(ControlRequest(Kind.LOAD_STATE, 2_500_100), owner._clock)
    waits = []
    # Only the platform wait is controlled; receiver time advances explicitly.
    monkeypatch.setattr(persistence_control_channel, "TIMEOUT_MAX", 1.0)

    def wait(timeout):
        waits.append(timeout)
        assert timeout == (1.0 if len(waits) < 3 else 0.5)
        if completed and len(waits) == 3:
            command.completion.set()
            return True
        owner._clock.advance_elapsed_us(int(timeout * 1_000_000))
        return False

    monkeypatch.setattr(command.completion, "wait", wait)
    assert owner.control._wait_for_completion(command, 2.5) is completed
    assert waits == [1.0, 1.0, 0.5]
    assert owner._clock.now_monotonic_us() == (2_000_100 if completed else 2_500_100)
