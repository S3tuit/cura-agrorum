import json
import shutil
import sqlite3
from dataclasses import replace

from cura_receiver.generated.receiver_enums_generated import (
    PersistenceAdmissionState as State,
)
from cura_receiver.ordinary_persistence import OrdinaryPersistence
from cura_receiver.persist_queue import PersistQueue
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC, ProfileOnlyUnitV1
from cura_receiver.receiver_configuration import ReceiverConfigurationReader
from cura_receiver.receiver_startup import (
    create_receiver_instance,
    start_receiver_instance,
)
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import GROUP, INSTANCE, _profile


# A completed prefix needs no RAM ledger; installing older history requires fresh startup.
def test_operator_restore_loses_queue_and_newer_history(setup, tmp_path):
    path, connection, queue, clock, create = setup
    backup = tmp_path / "old-backup.db"
    with sqlite3.connect(backup) as destination:
        connection.backup(destination)

    class CorruptQuarantine(SqliteTransactions):
        def commit(self, connection):
            if connection.execute(
                "SELECT count(*) FROM quarantined_entities"
            ).fetchone() == (1,):
                error = sqlite3.DatabaseError("injected quarantine corruption result")
                error.sqlite_errorcode = sqlite3.SQLITE_CORRUPT
                raise error
            super().commit(connection)

    owner = create(CorruptQuarantine())
    owner.enable_admission()
    for profile in (_profile(), replace(_profile(sequence=2), busy_wait_count=-1)):
        queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
            ProfileOnlyUnitV1(profile)
        )
    owner.attempt(max_entities=2)  # Roll back the mixed batch for isolation.
    assert owner.attempt(max_entities=2).acknowledged_entities == 1
    assert queue.snapshot().published_entities == queue.snapshot().claimed_entities == 1
    assert connection.execute(
        "SELECT occurrence_sequence FROM message_profiles"
    ).fetchall() == [(1,)]
    owner.attempt(max_entities=2)  # Reproduce poison B alone.
    failure = owner.attempt(max_entities=2).failure
    assert failure.admission_state is State.UNAVAILABLE_CORRUPT
    assert owner.attempt(max_entities=2) is None

    # All SQLite handles are now closed. Real damaged bytes remain for diagnosis.
    with path.open("r+b") as target:
        target.write(b"not a SQLite db!")
    originals = {
        candidate: (candidate.stat().st_ino, candidate.read_bytes())
        for candidate in (
            path,
            path.with_name(path.name + "-wal"),
            path.with_name(path.name + "-shm"),
        )
        if candidate.exists()
    }
    owner.request_operator_recovery()
    assert (
        owner.attempt(max_entities=2).failure.admission_state
        is State.UNAVAILABLE_CORRUPT
    )
    for candidate, (inode, contents) in originals.items():
        assert candidate.stat().st_ino == inode
        assert candidate.stat().st_size == len(contents)
        if not candidate.name.endswith("-shm"):
            assert candidate.read_bytes() == contents

    # Operator keeps the originals and installs an internally consistent older backup.
    evidence = tmp_path / "corrupt-originals"
    evidence.mkdir()
    preserved = {}
    for candidate in originals:
        saved = evidence / candidate.name
        candidate.rename(saved)
        preserved[saved] = saved.read_bytes()
    shutil.copyfile(backup, path)
    owner.request_operator_recovery()
    assert (
        owner.attempt(max_entities=2).failure.admission_state
        is State.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    )
    assert (
        queue.snapshot().published_entities == 1
    )  # No acknowledgement across replacement.
    assert owner.counters.batch_entities_committed == 1
    owner.close()

    # Compose the new process state through actual configuration/open/startup.
    configuration = tmp_path / "group.json"
    configuration.write_text(
        json.dumps(
            {
                "format_version": 1,
                "group_id": GROUP.hex(),
                "group_master_key": "00" * 32,
                "active_node_ids": [],
                "retired_node_ids": [],
            }
        )
    )
    configuration.chmod(0o600)
    boot = tmp_path / "boot-id"
    boot.write_text("11223344-5566-7788-9900-aabbccddeeff\n")
    instance = create_receiver_instance(clock)
    startup = start_receiver_instance(
        instance,
        configuration_reader=ReceiverConfigurationReader(
            configuration, boot_id_path=boot
        ),
        database_path=path,
        minimum_free_bytes=0,
    )
    assert startup.started
    assert instance.receiver_instance_id != INSTANCE
    restarted_queue = PersistQueue()
    restarted = OrdinaryPersistence(
        startup.database, restarted_queue, instance=instance, clock=clock
    )
    try:
        restarted.enable_admission()
        assert restarted_queue.snapshot().published_entities == 0
        assert restarted.attempt(max_entities=2) is None
        for table in ("message_profiles", "quarantined_entities"):
            assert startup.database.connection.execute(
                f"SELECT count(*) FROM {table}"
            ).fetchone() == (0,)
        assert startup.database.connection.execute(
            "SELECT count(*) FROM receiver_instances"
        ).fetchone() == (2,)
        profile = replace(
            _profile(), receiver_instance_id=instance.receiver_instance_id
        )
        restarted_queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).reservation.publish(
            ProfileOnlyUnitV1(profile)
        )
        assert restarted.attempt(max_entities=1).acknowledged_entities == 1
        assert startup.database.connection.execute(
            "SELECT receiver_instance_id FROM message_profiles"
        ).fetchall() == [(instance.receiver_instance_id,)]
        assert {saved: saved.read_bytes() for saved in preserved} == preserved
    finally:
        restarted.close()
