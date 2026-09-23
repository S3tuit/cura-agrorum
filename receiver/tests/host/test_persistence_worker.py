import sqlite3
from threading import Event, current_thread

import pytest

from cura_receiver.generated.receiver_enums_generated import (
    PersistenceAdmissionState as State,
)
from cura_receiver.persistence_worker import PersistenceWorker
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition,
    CommunicatorStateCondition,
)
from cura_receiver.generated.receiver_entities_generated import (
    communicator_state_v1_parameters,
)
from cura_receiver.receiver_startup import ReceiverInstanceStart
from tests.support.builders.persistence import INSTANCE
from tests.support.builders.persistence_control import state, synthetic
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
from tests.support.fakes.os_clock import FakeOsClock


def create_worker(paths, **kwargs):
    database, configuration, boot = paths
    return PersistenceWorker(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=database,
        configuration_path=configuration,
        boot_id_path=boot,
        clock=FakeOsClock(monotonic_us=100),
        **kwargs,
    )


def stop_worker(worker):
    worker.request_stop(deadline_monotonic_us=0)
    worker.join(5)
    assert not worker.is_alive()


# Configuration I/O, database opening, state loading and admission occur on the persistence owner.
def test_worker_startup_ownership(worker_files, monkeypatch):
    import cura_receiver.receiver_configuration as configuration
    import cura_receiver.receiver_startup as startup

    arrived, release = Event(), Event()
    observed = []
    original_loader = configuration.load_receiver_group
    original_open = startup.open_receiver_database

    def loader(*args, **kwargs):
        observed.append(("configuration", current_thread()))
        return original_loader(*args, **kwargs)

    def opener(*args, **kwargs):
        observed.append(("database", current_thread()))
        arrived.set()
        assert release.wait(5), "database-open boundary not released"
        return original_open(*args, **kwargs)

    monkeypatch.setattr(configuration, "load_receiver_group", loader)
    monkeypatch.setattr(startup, "open_receiver_database", opener)
    worker = create_worker(worker_files)
    worker.start()
    try:
        assert arrived.wait(5)
        assert (
            worker.queue.snapshot().admission_snapshot.state
            is State.UNAVAILABLE_STARTING
        )
        with sqlite3.connect(worker_files[0]) as observer:
            assert observer.execute("SELECT * FROM receiver_instances").fetchall() == []
        release.set()
        result = worker.wait_started(deadline_monotonic_us=5_000_100)
        assert result.database_failure is None
        assert worker.queue.snapshot().admission_snapshot.state is State.AVAILABLE
        assert observed == [("configuration", worker), ("database", worker)]
        assert not hasattr(result, "database")
        with sqlite3.connect(worker_files[0]) as observer:
            assert observer.execute(
                "SELECT receiver_instance_id FROM receiver_instances"
            ).fetchall() == [(INSTANCE,)]
    finally:
        release.set()
        stop_worker(worker)


# Configuration and storage rejection never open ordinary admission or invent a lifecycle row.
@pytest.mark.parametrize("case", ["configuration", "database"])
def test_failed_worker_startup(worker_files, case):
    database, configuration, _ = worker_files
    if case == "configuration":
        configuration.chmod(0o644)
    else:
        database.write_bytes(b"corrupt receiver database")
    worker = create_worker(worker_files)
    worker.start()
    try:
        result = worker.wait_started(deadline_monotonic_us=5_000_100)
        assert result is not None
        assert worker.queue.snapshot().admission_snapshot.state is not State.AVAILABLE
        if case == "database":
            assert result.database_failure.admission_state is State.UNAVAILABLE_CORRUPT
        else:
            with sqlite3.connect(database) as observer:
                assert (
                    observer.execute("SELECT * FROM receiver_instances").fetchall()
                    == []
                )
    finally:
        stop_worker(worker)


# F-002: invalid UTF-8 is recoverable application state through real worker startup and controls.
@pytest.mark.parametrize(
    "column",
    [
        "singleton_id",
        "state_format_version",
        "generation",
        "state_blob",
        "state_sha256",
    ],
)
def test_worker_recovers_invalid_text_state(worker_files, column):
    database, configuration, boot = worker_files
    with sqlite3.connect(database) as db:
        db.execute(
            "INSERT INTO communicator_state VALUES (?, ?, ?, ?, ?)",
            communicator_state_v1_parameters(state()),
        )
        db.execute(f"UPDATE communicator_state SET {column} = CAST(X'80' AS TEXT)")
    worker = CheckedPersistenceWorker(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=database,
        configuration_path=configuration,
        boot_id_path=boot,
        clock=FakeOsClock(monotonic_us=100),
    )
    worker.start()
    try:
        startup = worker.wait_started(deadline_monotonic_us=5_000_100)
        assert startup.database_failure is None
        assert startup.state_load.state_condition is CommunicatorStateCondition.CORRUPT
        assert worker.queue.snapshot().admission_snapshot.state is State.AVAILABLE
        assert (
            worker.control.load_communicator_state(
                deadline_monotonic_us=5_000_100
            ).state_condition
            is CommunicatorStateCondition.CORRUPT
        )
        assert (
            worker.control.commit_communicator_state(
                synthetic(), deadline_monotonic_us=5_000_100
            ).disposition
            is CommunicatorStateCommitDisposition.COMMITTED
        )
        assert (
            worker.control.load_communicator_state(
                deadline_monotonic_us=5_000_100
            ).state
            == synthetic()
        )
        with sqlite3.connect(database) as observer:
            assert observer.execute(
                f"SELECT typeof(observed_{column}), hex(observed_{column}) FROM quarantined_communicator_states"
            ).fetchall() == [("text", "80")]
    finally:
        worker.finish_test()


# F-001: missing control schema rejects worker startup before inserting its lifecycle row.
@pytest.mark.parametrize(
    "table", ["communicator_state", "quarantined_communicator_states"]
)
def test_worker_rejects_missing_control_schema(worker_files, table):
    database, configuration, boot = worker_files
    with sqlite3.connect(database) as db:
        db.execute(f"DROP TABLE {table}")
    worker = CheckedPersistenceWorker(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=database,
        configuration_path=configuration,
        boot_id_path=boot,
        clock=FakeOsClock(monotonic_us=100),
    )
    worker.start()
    try:
        startup = worker.wait_started(deadline_monotonic_us=5_000_100)
        assert (
            startup.database_failure.admission_state
            is State.UNAVAILABLE_INCOMPATIBLE_SCHEMA
        )
        assert startup.instance_start is None
        assert (
            worker.queue.snapshot().admission_snapshot.state
            is State.UNAVAILABLE_INCOMPATIBLE_SCHEMA
        )
        with sqlite3.connect(database) as observer:
            assert observer.execute("SELECT * FROM receiver_instances").fetchall() == []
    finally:
        worker.finish_test()


# An undersized publication retains its flush deadline across unrelated wakeups.
def test_flush_deadline_and_unrelated_wake(worker_files):
    from queue import Queue
    from cura_receiver.persist_queue_entities import (
        PROFILE_ONLY_V1_SPEC,
        ProfileOnlyUnitV1,
    )
    from tests.support.builders.persistence import _profile

    waits = Queue()

    class Observed(PersistenceWorker):
        def _wait_for_work(self, timeout):
            waits.put(timeout)
            # Receiver time advances only when the test advances its manual clock.
            assert self._wake.wait(5), "manual-clock wait needs an explicit wake"

    database, config, boot = worker_files
    clock = FakeOsClock(monotonic_us=100)
    worker = Observed(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=database,
        configuration_path=config,
        boot_id_path=boot,
        clock=clock,
    )
    worker.start()
    try:
        assert worker.wait_started(deadline_monotonic_us=5_000_100)
        assert waits.get(timeout=5) is None
        worker.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
            ProfileOnlyUnitV1(_profile())
        )
        assert waits.get(timeout=5) == 5.0
        clock.advance_elapsed_us(4_999_999)
        worker._wake.set()
        assert waits.get(timeout=5) == 0.000001
        assert worker.queue.snapshot().published_entities == 1
        clock.advance_elapsed_us(1)
        worker._wake.set()
        assert waits.get(timeout=5) is None
        assert worker.queue.snapshot().published_entities == 0
        with sqlite3.connect(database) as observer:
            assert observer.execute(
                "SELECT count(*) FROM message_profiles"
            ).fetchone() == (1,)
    finally:
        stop_worker(worker)


# Retry wakeups never bypass the ordinary component's exact retained-work deadline.
def test_worker_reuses_retry_deadline(worker_files):
    from queue import Queue
    from cura_receiver.sqlite_transactions import SqliteTransactions
    from cura_receiver.persist_queue_entities import (
        PROFILE_ONLY_V1_SPEC,
        ProfileOnlyUnitV1,
    )
    from tests.support.builders.persistence import _profile

    waits, attempts = Queue(), []

    class BusyOnce(SqliteTransactions):
        def begin(self, connection):
            attempts.append(clock.now_monotonic_us())
            if len(attempts) == 1:
                raise sqlite3.OperationalError("injected busy")
            super().begin(connection)

    class Observed(PersistenceWorker):
        def _wait_for_work(self, timeout):
            waits.put(timeout)
            # Receiver time advances only when the test advances its manual clock.
            assert self._wake.wait(5), "manual-clock wait needs an explicit wake"

    database, config, boot = worker_files
    clock = FakeOsClock(monotonic_us=100)
    worker = Observed(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=database,
        configuration_path=config,
        boot_id_path=boot,
        clock=clock,
        transactions=BusyOnce(),
        wake_threshold_entities=1,
    )
    worker.start()
    try:
        assert worker.wait_started(deadline_monotonic_us=5_000_100)
        assert waits.get(timeout=5) is None
        worker.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
            ProfileOnlyUnitV1(_profile())
        )
        first_wait = waits.get(timeout=5)
        assert first_wait > 0.25
        deadline = worker._ordinary.retry_deadline_monotonic_us
        worker._wake.set()
        assert waits.get(timeout=5) == first_wait
        assert attempts == [100]
        clock.advance_elapsed_us(deadline - clock.now_monotonic_us())
        worker._wake.set()
        assert waits.get(timeout=5) is None
        assert attempts == [100, deadline]
        assert worker.queue.snapshot().admission_snapshot.state is State.AVAILABLE
        assert worker.queue.snapshot().published_entities == 0
    finally:
        stop_worker(worker)


# Reaching the count threshold starts draining in bounded batches and finishes a later undersized tail.
def test_count_threshold_and_batch_limit(worker_files):
    from queue import Queue
    from cura_receiver.sqlite_transactions import SqliteTransactions
    from cura_receiver.persist_queue_entities import (
        PROFILE_ONLY_V1_SPEC,
        ProfileOnlyUnitV1,
    )
    from tests.support.builders.persistence import _profile

    waits, arrived, release, batch_sizes = Queue(), Event(), Event(), []

    class Batches(SqliteTransactions):
        def begin(self, connection):
            super().begin(connection)
            if not batch_sizes:
                arrived.set()
                assert release.wait(5)

        def commit(self, connection):
            batch_sizes.append(worker.queue.snapshot().claimed_entities)
            super().commit(connection)

    class Observed(PersistenceWorker):
        def _wait_for_work(self, timeout):
            waits.put(timeout)
            # Receiver time advances only when the test advances its manual clock.
            assert self._wake.wait(5), "manual-clock wait needs an explicit wake"

    database, config, boot = worker_files
    worker = Observed(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=database,
        configuration_path=config,
        boot_id_path=boot,
        clock=FakeOsClock(monotonic_us=100),
        transactions=Batches(),
        wake_threshold_entities=2,
        batch_limit_entities=1,
    )
    worker.start()
    try:
        assert worker.wait_started(deadline_monotonic_us=5_000_100)
        assert waits.get(timeout=5) is None
        for sequence in (1, 2):
            worker.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
                ProfileOnlyUnitV1(_profile(sequence=sequence))
            )
        assert arrived.wait(5)
        worker.queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
            ProfileOnlyUnitV1(_profile(sequence=3))
        )
        release.set()
        while waits.get(timeout=5) is not None:
            pass  # Drain any earlier named undersized-wait observation.
        assert batch_sizes == [1, 1, 1]
        assert worker.queue.snapshot().published_entities == 0
    finally:
        release.set()
        stop_worker(worker)


# Empty-queue checkpoint faults retain paced recovery through repeated capped retries without probe rows.
def test_worker_empty_checkpoint_recovery_through_cap(worker_files):
    from queue import Queue
    from cura_receiver.elapsed_duration import minimum_wait_monotonic_us
    from cura_receiver.sqlite_transactions import SqliteTransactions

    waits, attempts = Queue(), []

    class Checkpoints(SqliteTransactions):
        def checkpoint(self, connection):
            attempts.append(clock.now_monotonic_us())
            if len(attempts) <= 8:
                raise sqlite3.OperationalError("injected checkpoint I/O")
            return super().checkpoint(connection)

    class Observed(PersistenceWorker):
        def _wait_for_work(self, timeout):
            waits.put(timeout)
            # Receiver time advances only when the test advances its manual clock.
            assert self._wake.wait(5), "manual-clock wait needs an explicit wake"

    database, config, boot = worker_files
    clock = FakeOsClock(monotonic_us=100)
    worker = Observed(
        instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=database,
        configuration_path=config,
        boot_id_path=boot,
        clock=clock,
        transactions=Checkpoints(),
        checkpoint_threshold_bytes=1,
    )
    worker.start()
    try:
        assert worker.wait_started(deadline_monotonic_us=5_000_100)
        for index in range(8):
            expected = minimum_wait_monotonic_us(min(250_000 * 2**index, 5_000_000))
            assert waits.get(timeout=5) == expected / 1_000_000
            assert (
                worker.queue.snapshot().admission_snapshot.state is State.UNAVAILABLE_IO
            )
            assert worker._ordinary.checkpoint_pending
            worker._wake.set()
            assert waits.get(timeout=5) == expected / 1_000_000
            assert len(attempts) == index + 1
            clock.advance_elapsed_us(expected)
            worker._wake.set()
        assert waits.get(timeout=5) == 5.0
        assert len(attempts) == 9
        assert not worker._ordinary.checkpoint_pending
        assert worker.queue.snapshot().admission_snapshot.state is State.AVAILABLE
        with sqlite3.connect(database) as observer:
            for table in (
                "clock_observations",
                "message_profiles",
                "receiver_health",
                "diagnostics",
                "quarantined_entities",
            ):
                assert observer.execute(f"SELECT count(*) FROM {table}").fetchone() == (
                    0,
                )
    finally:
        stop_worker(worker)
