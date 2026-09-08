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
