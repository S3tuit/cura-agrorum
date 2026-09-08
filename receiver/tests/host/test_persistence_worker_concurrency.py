import sqlite3
from threading import Event, current_thread

import pytest

from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    PersistenceAdmissionState as State,
    RadioState,
)
from cura_receiver.persist_queue_entities import (
    PROFILE_ONLY_V1_SPEC,
    RECEIVER_HEALTH_REQUEST_V1_SPEC,
    ProfileOnlyUnitV1,
)
from cura_receiver.persistence_control_values import (
    CommunicatorStateCondition as Condition,
)
from cura_receiver.ports.host_observations import HostObservations
from cura_receiver.protocol_ingress import ProtocolIngress, ProtocolIngressTerminalV1
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import INSTANCE, _profile, _health_request
from tests.support.builders.protocol_ingress import (
    REVIEWED_NODE_ID,
    REVIEWED_NODE_KEY,
    ingress_packet,
)
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
from tests.support.coordination.threads import (
    start_checked_threads,
    join_checked_threads,
)
from tests.support.fakes.os_clock import FakeOsClock


@pytest.fixture
def create(worker_files):
    workers = []

    def build(cls=CheckedPersistenceWorker, **kwargs):
        path, config, boot = worker_files
        owner = cls(
            instance=ReceiverInstanceStart(INSTANCE, 0),
            database_path=path,
            configuration_path=config,
            boot_id_path=boot,
            clock=FakeOsClock(monotonic_us=100),
            **kwargs,
        )
        workers.append(owner)
        return owner

    yield build
    for owner in workers:
        if owner.ident is not None:
            owner.finish_test()


def publish(owner, sequence=1):
    owner.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
        ProfileOnlyUnitV1(_profile(sequence=sequence))
    )


# Clear/recheck observes each predicate whether its notification precedes or follows Event.clear().
@pytest.mark.parametrize("kind", ["publication", "control", "shutdown"])
@pytest.mark.parametrize("side", ["before_clear", "after_clear"])
def test_clear_recheck_races(create, kind, side):
    armed, arrived, release, completed, submitted = (
        Event(),
        Event(),
        Event(),
        Event(),
        Event(),
    )

    class Observe(CheckedPersistenceWorker):
        def _dispatch_work(self, action):
            super()._dispatch_work(action)
            completed.set()

    owner = create(Observe, wake_threshold_entities=1)
    original_clear = owner._wake.clear

    def clear():
        if armed.is_set():
            armed.clear()
            if side == "after_clear":
                original_clear()
            arrived.set()
            assert release.wait(5), "clear/recheck boundary not released"
            if side == "before_clear":
                original_clear()
        else:
            original_clear()

    owner._wake.clear = clear
    armed.set()
    owner.start()
    callers = ()
    try:
        assert arrived.wait(5)
        if kind == "publication":
            publish(owner)
        elif kind == "shutdown":
            owner.request_stop(deadline_monotonic_us=0)
        else:
            original_wait = owner.control._wait_for_completion

            def wait(command, remaining):
                submitted.set()
                return original_wait(command, remaining)

            owner.control._wait_for_completion = wait

            def call():
                assert (
                    owner.control.load_communicator_state(
                        deadline_monotonic_us=5_000_100
                    ).state_condition
                    is Condition.MISSING
                )
                completed.set()

            callers = start_checked_threads([("clear-recheck-control", call)])
            assert submitted.wait(5)
        release.set()
        if kind == "shutdown":
            owner.join(5)
            assert not owner.is_alive()
        else:
            assert completed.wait(5)
            assert owner.queue.snapshot().published_entities == 0
        join_checked_threads(callers)
    finally:
        release.set()


# Dispatch that already won the scheduler lock retains exactly one turn before a submitted control.
def test_submission_after_dispatch_before_sql(create):
    selected, release, submitted, done = Event(), Event(), Event(), Event()
    trace = []

    class Scheduled(CheckedPersistenceWorker):
        def _dispatch_work(self, action):
            if not selected.is_set():
                selected.set()
                assert release.wait(5), "selected ordinary dispatch not released"
            trace.append(action)
            super()._dispatch_work(action)

        def _dispatch_control(self, command):
            trace.append("control")
            super()._dispatch_control(command)
            done.set()

    owner = create(Scheduled, wake_threshold_entities=1, batch_limit_entities=1)
    original_wait = owner.control._wait_for_completion

    def wait(command, remaining):
        submitted.set()
        return original_wait(command, remaining)

    owner.control._wait_for_completion = wait
    owner.start()
    assert owner.wait_started(deadline_monotonic_us=5_000_100)
    publish(owner)
    assert selected.wait(5)
    publish(owner, 2)
    callers = start_checked_threads(
        [
            (
                "after-dispatch-caller",
                lambda: owner.control.load_communicator_state(
                    deadline_monotonic_us=5_000_100
                ),
            )
        ]
    )
    try:
        assert submitted.wait(5)
        assert trace == []
        release.set()
        assert done.wait(5)
        join_checked_threads(callers)
        assert trace[:2] == ["ordinary", "control"]
    finally:
        release.set()


# Control waits behind each real open-transaction phase, including rollback after a failed COMMIT.
@pytest.mark.parametrize("phase", ["after_begin", "before_commit", "before_rollback"])
def test_control_waits_for_safe_sql_boundary(create, phase):
    arrived, release, submitted = Event(), Event(), Event()
    trace = []

    class Gated(SqliteTransactions):
        def begin(self, db):
            super().begin(db)
            if phase == "after_begin":
                arrived.set()
                assert release.wait(5)

        def commit(self, db):
            if phase == "before_commit":
                arrived.set()
                assert release.wait(5)
            if phase == "before_rollback":
                raise sqlite3.OperationalError("unknown commit boundary")
            super().commit(db)
            trace.append("commit")

        def rollback(self, db):
            arrived.set()
            assert release.wait(5)
            super().rollback(db)
            trace.append("rollback")

    class Observed(CheckedPersistenceWorker):
        def _dispatch_control(self, command):
            trace.append("control")
            super()._dispatch_control(command)

    owner = create(Observed, transactions=Gated(), wake_threshold_entities=1)
    wait = owner.control._wait_for_completion

    def completion(command, remaining):
        submitted.set()
        return wait(command, remaining)

    owner.control._wait_for_completion = completion
    owner.start()
    assert owner.wait_started(deadline_monotonic_us=5_000_100)
    publish(owner)
    assert arrived.wait(5)
    callers = start_checked_threads(
        [
            (
                "open-transaction-control",
                lambda: owner.control.load_communicator_state(
                    deadline_monotonic_us=5_000_100
                ),
            )
        ]
    )
    try:
        assert submitted.wait(5)
        assert trace == []
        release.set()
        join_checked_threads(callers)
        assert trace[:2] == [
            ("rollback" if phase == "before_rollback" else "commit"),
            "control",
        ]
    finally:
        release.set()


# Existing ingress continues in memory during disk I/O; SQLite and host sampling stay on the worker.
def test_ingress_and_resource_ownership_during_disk_stall(create, monkeypatch):
    import cura_receiver.receiver_startup as startup

    arrived, release, drained = Event(), Event(), Event()
    owners = []
    original_open = startup.open_receiver_database

    def opener(*args, **kwargs):
        result = original_open(*args, **kwargs)
        result.database.connection.set_trace_callback(
            lambda sql: owners.append(("sql", current_thread()))
        )
        return result

    monkeypatch.setattr(startup, "open_receiver_database", opener)

    class Host:
        def sample(self):
            owners.append(("host", current_thread()))
            return HostObservations(memory_available_bytes=123)

    class Gated(SqliteTransactions):
        def begin(self, db):
            super().begin(db)
            if not arrived.is_set():
                arrived.set()
                assert release.wait(5)

    class Observed(CheckedPersistenceWorker):
        def _dispatch_work(self, action):
            super()._dispatch_work(action)
            if not self.queue.snapshot().published_entities:
                drained.set()

    owner = create(
        Observed,
        transactions=Gated(),
        host_observations=Host(),
        wake_threshold_entities=1,
    )
    owner.start()
    assert owner.wait_started(deadline_monotonic_us=5_000_100)
    publish(owner)
    try:
        assert arrived.wait(5)
        ingress = ProtocolIngress(
            queue=owner.queue,
            monotonic_clock=FakeOsClock(monotonic_us=20),
            auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
        )
        decision = ingress.begin(
            ingress_packet(receiver_instance_id=INSTANCE, occurrence_sequence=2)
        )
        assert decision.pre_tx_profile.ack_selected is AckSelection.ACCEPTED
        ingress.finalize(
            decision,
            ProtocolIngressTerminalV1(
                AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
                None,
                None,
                23,
                radio_state=RadioState.RX_SINGLE,
            ),
        )
        owner.queue.try_reserve_one(
            RECEIVER_HEALTH_REQUEST_V1_SPEC
        ).reservation.publish(_health_request())
        assert owner.queue.snapshot().published_entities == 3
        assert owner.queue.snapshot().claimed_entities == 1
        release.set()
        assert drained.wait(5)
        assert {"sql", "host"} <= {kind for kind, _ in owners}
        assert all(thread is owner for _, thread in owners)
        assert owner.queue.snapshot().admission_snapshot.state is State.AVAILABLE
    finally:
        release.set()
