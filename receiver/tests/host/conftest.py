from __future__ import annotations

import pytest
from cura_receiver.database_initializer import initialize_database
from cura_receiver.ordinary_persistence import OrdinaryPersistence
from cura_receiver.persist_queue import PersistQueue
from cura_receiver.receiver_startup import (
    ReceiverInstanceStart,
    insert_receiver_instance_start,
)
from cura_receiver.sqlite_database import open_receiver_database

from tests.support.builders.persistence import GROUP, INSTANCE
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.coordination.persistence_worker import prepare_worker_files


@pytest.fixture
def setup(tmp_path):
    path = tmp_path / "receiver.db"
    initialize_database(path, GROUP)
    database = open_receiver_database(path, GROUP, minimum_free_bytes=0).database
    connection = database.connection
    instance = ReceiverInstanceStart(INSTANCE, 0)
    insert_receiver_instance_start(connection, instance, b"b" * 16)
    queue = PersistQueue()
    clock = FakeOsClock(monotonic_us=100)
    instances = []

    def create(transactions=None, *, instance=instance, **kwargs):
        persistence = OrdinaryPersistence(
            database,
            queue,
            instance=instance,
            clock=clock,
            transactions=transactions,
            **kwargs,
        )
        instances.append(persistence)
        return persistence

    yield path, connection, queue, clock, create
    for persistence in instances:
        persistence.close()
    database.close()


@pytest.fixture
def worker_files(tmp_path):
    return prepare_worker_files(tmp_path)


@pytest.fixture
def worker_file_factory():
    return prepare_worker_files


@pytest.fixture
def airtime_component(tmp_path):
    import hashlib
    import sqlite3
    from cura_receiver.airtime_ledger import AirtimeCorrelation
    from cura_receiver.communicator_state_owner import CommunicatorStateOwner
    from cura_receiver.generated.receiver_entities_generated import (
        communicator_state_v1_parameters,
    )
    from cura_receiver.generated.receiver_enums_generated import (
        RtcHealth as RH,
        SystemTimeQuality as Q,
    )
    from cura_receiver.persistence_control_values import CommunicatorStateCondition as C
    from cura_receiver.time_observations import TrustedTimeSample
    from cura_receiver.tx_airtime import TxAirtimePolicy
    from tests.support.coordination.persistence_worker import CheckedPersistenceWorker
    from tests.support.builders.persistence_control import state

    workers = []

    def create(condition=C.MISSING, *, utc=0, initial_state=None, rate_bound_ppm=3700):
        root = tmp_path / str(len(workers))
        root.mkdir()
        database, config, boot = prepare_worker_files(root)
        with sqlite3.connect(database) as connection:
            if initial_state is not None:
                condition = C.NONE
                connection.execute(
                    "INSERT INTO communicator_state VALUES (?,?,?,?,?)",
                    communicator_state_v1_parameters(initial_state),
                )
            elif condition is C.CORRUPT:
                connection.executemany(
                    "INSERT INTO communicator_state VALUES (?,?,?,?,?)",
                    [(None, "bad", 1.5, 42, None), (2, 1, 0, b"bad", b"bad")],
                )
            elif condition is C.UNSUPPORTED_VERSION:
                connection.execute(
                    "INSERT INTO communicator_state VALUES (?,?,?,?,?)",
                    (1, 2, 1, b"\x02\x00", hashlib.sha256(b"\x02\x00").digest()),
                )
            elif condition is C.POLICY_MISMATCH:
                connection.execute(
                    "INSERT INTO communicator_state VALUES (?,?,?,?,?)",
                    communicator_state_v1_parameters(
                        state(tx_airtime_budget_us=35_000_000)
                    ),
                )
        clock = FakeOsClock(monotonic_us=100)
        worker = CheckedPersistenceWorker(
            instance=ReceiverInstanceStart(INSTANCE, 0),
            database_path=database,
            configuration_path=config,
            boot_id_path=boot,
            clock=clock,
        )
        workers.append(worker)
        worker.start()
        loaded = worker.wait_started(deadline_monotonic_us=5_000_100).state_load
        assert loaded.state_condition is condition
        owner = CommunicatorStateOwner.from_load(control=worker.control, loaded=loaded)
        policy = TxAirtimePolicy(
            state_owner=owner, clock=clock, rate_bound_ppm=rate_bound_ppm
        )
        policy.update_time(
            AirtimeCorrelation(
                TrustedTimeSample(100, utc, 1, Q.NETWORK_SYNCED, 1), 1, 10_000_000_100
            ),
            rtc_health=RH.PRESENT,
        )
        return policy, worker, database, clock, loaded

    yield create
    for worker in workers:
        worker.finish_test()
