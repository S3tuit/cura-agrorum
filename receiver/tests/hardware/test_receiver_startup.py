from __future__ import annotations

import json
import os
import sqlite3
import subprocess
import sys
from pathlib import Path
from uuid import UUID

import pytest

from cura_receiver.database_initializer import initialize_database
from cura_receiver.generated.receiver_entities_generated import ClockObservationV1
from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth,
    SystemTimeQuality,
)
from cura_receiver.platform.linux_boot_identity import read_linux_boot_id
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.receiver_configuration import (
    ReceiverConfigurationLoadStatus,
    ReceiverConfigurationReader,
)
from cura_receiver.receiver_startup import (
    ReceiverInstanceStartDisposition,
    create_receiver_instance,
    insert_receiver_instance_start,
)
from cura_receiver.sqlite_database import open_receiver_database
from cura_receiver.sqlite_repository import SqliteRepository

pytestmark = pytest.mark.hardware
GROUP = bytes.fromhex("0102030405060708")


def _configuration(tmp_path: Path) -> Path:
    path = tmp_path / "test-receiver-group.json"
    path.write_text(
        json.dumps(
            {
                "format_version": 1,
                "group_id": GROUP.hex(),
                "group_master_key": "00" * 32,
                "active_node_ids": [],
                "retired_node_ids": [],
            }
        ),
        encoding="utf-8",
    )
    path.chmod(0o600)
    return path


# The deployed SQLite stack establishes production durability settings and exact on-disk rows.
def test_target_sqlite_capabilities(tmp_path: Path) -> None:
    path = tmp_path / "test-receiver.db"
    initialize_database(path, GROUP)
    opened = open_receiver_database(path, GROUP, minimum_free_bytes=1)
    assert opened.failure is None
    connection = opened.connection
    try:
        pragmas = {
            name: connection.execute(f"PRAGMA {name}").fetchone()[0]
            for name in (
                "journal_mode",
                "synchronous",
                "foreign_keys",
                "busy_timeout",
                "wal_autocheckpoint",
            )
        }
        assert pragmas == {
            "journal_mode": "wal",
            "synchronous": 2,
            "foreign_keys": 1,
            "busy_timeout": 250,
            "wal_autocheckpoint": 0,
        }
        assert connection.getconfig(sqlite3.SQLITE_DBCONFIG_NO_CKPT_ON_CLOSE)
        clock = LinuxOsClock()
        instance = create_receiver_instance(clock)
        start = insert_receiver_instance_start(
            connection, instance, read_linux_boot_id()
        )
        assert start.disposition is ReceiverInstanceStartDisposition.STARTED
        sample = clock.now_monotonic_us()
        repository = SqliteRepository(connection)
        connection.execute("BEGIN IMMEDIATE")
        repository.insert_clock_observation(
            ClockObservationV1(
                instance.receiver_instance_id,
                1,
                0,
                sample,
                None,
                False,
                SystemTimeQuality.UNTRUSTED,
                RtcHealth.MISSING,
            )
        )
        connection.execute("COMMIT")
        assert repository.find_clock_observation(instance.receiver_instance_id, 1) == (
            instance.receiver_instance_id,
            1,
            0,
            sample,
            None,
            0,
            0,
            2,
        )
        assert connection.execute("PRAGMA integrity_check").fetchall() == [("ok",)]
        assert connection.execute("PRAGMA foreign_key_check").fetchall() == []
        filesystem = subprocess.run(
            [
                "findmnt",
                "--target",
                str(tmp_path),
                "--noheadings",
                "--output",
                "FSTYPE,SOURCE,TARGET",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
        metadata = {
            "python": sys.version,
            "sqlite": sqlite3.sqlite_version,
            "compile_options": [
                item[0] for item in connection.execute("PRAGMA compile_options")
            ],
            "pragmas": pragmas,
            "filesystem": filesystem.stdout.strip(),
            "uid": os.geteuid(),
            "test_path": str(tmp_path),
        }
        (tmp_path / "sqlite-capabilities.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(metadata, sort_keys=True))
    finally:
        connection.close()


# The actual unprivileged account loads a private fixture and the kernel's boot identity.
def test_target_configuration_and_boot_identity(tmp_path: Path) -> None:
    assert (
        os.geteuid() != 0
    ), "receiver configuration test must use an unprivileged account"
    path = _configuration(tmp_path)
    result = ReceiverConfigurationReader(path, expected_owner_uid=os.geteuid()).read()
    assert result.status is ReceiverConfigurationLoadStatus.LOADED
    assert result.configuration.group_id == GROUP
    assert result.linux_boot_id == read_linux_boot_id()
    assert len(result.linux_boot_id) == 16
    assert "configuration=" not in repr(result)


# Two real process starts create ordered durable rows under one unchanged kernel boot UUID.
def test_target_process_restart_identities(tmp_path: Path) -> None:
    path = tmp_path / "test-receiver.db"
    initialize_database(path, GROUP)
    configuration = _configuration(tmp_path)
    receiver_root = Path(__file__).resolve().parents[2]
    protocol_root = receiver_root.parent / "protocol/protocol-v2-lora/python"
    child_code = """
import json, sys
from pathlib import Path
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.receiver_configuration import ReceiverConfigurationReader
from cura_receiver.receiver_startup import create_receiver_instance, start_receiver_instance
instance = create_receiver_instance(LinuxOsClock())
result = start_receiver_instance(instance, configuration_reader=ReceiverConfigurationReader(Path(sys.argv[1])), database_path=Path(sys.argv[2]), minimum_free_bytes=1)
assert result.started, repr(result)
print(json.dumps({'instance': instance.receiver_instance_id.hex(), 'boot': result.configuration_load.linux_boot_id.hex(), 'ordinal': result.instance_start.instance_ordinal}))
result.connection.close()
"""
    results = []
    for _ in range(2):
        child = subprocess.run(
            [sys.executable, "-c", child_code, str(configuration), str(path)],
            env={
                **os.environ,
                "PYTHONPATH": os.pathsep.join((str(receiver_root), str(protocol_root))),
            },
            capture_output=True,
            text=True,
            check=True,
            timeout=10,
        )
        results.append(json.loads(child.stdout))
    assert results[0]["instance"] != results[1]["instance"]
    assert all(UUID(hex=result["instance"]).version == 4 for result in results)
    assert [result["ordinal"] for result in results] == [1, 2]
    assert results[0]["boot"] == results[1]["boot"] == read_linux_boot_id().hex()
    opened = open_receiver_database(path, GROUP, minimum_free_bytes=1)
    assert opened.failure is None
    try:
        assert opened.connection.execute(
            "SELECT instance_ordinal, clean_stopped_at_monotonic_us, clean_stop_state_generation FROM receiver_instances ORDER BY instance_ordinal"
        ).fetchall() == [(1, None, None), (2, None, None)]
    finally:
        opened.connection.close()
