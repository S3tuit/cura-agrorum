"""RF-020 accepts actual SQLite snapshots and rejects incomplete evidence."""
from copy import deepcopy
import json
import os
from pathlib import Path
import sqlite3
import struct

from cryptography.hazmat.primitives.ciphers.aead import AESCCM
import pytest

from evidence import REPO, digest
from run_service import package_files, same_service, InstalledService, verify_service_stop
from service_probe import database_observation, snapshot, validate_config, expected_unit
from verify_service import BODY, FIELDS, verify_service

NODE, KEY, INSTANCE = bytes(range(8)), bytes(range(16)), bytes(range(16, 32))


def insert(db, table, values):
    columns = list(values)
    db.execute(f"INSERT INTO {table} ({','.join(columns)}) VALUES ({','.join('?' for _ in columns)})", list(values.values()))


def transcript(path, *, interval=12_000_000):
    db = sqlite3.connect(path)
    db.executescript((REPO / "receiver/db/schema.sql").read_text())
    db.execute("PRAGMA foreign_keys=ON")
    insert(db, "receiver_instances", dict(instance_ordinal=1, receiver_instance_id=INSTANCE,
        linux_boot_id=b"b" * 16, started_at_monotonic_us=1))
    deliveries = []
    for i in range(2):
        sample, message = 40 + i, 100 + i
        body = BODY.pack(sample, 1100, 2300, 2350, 2100, 2200, 2300, 101325, 4500,
                         8 if i else 1, i, 1500 if i else 0, 300 if i else 0, i, i, 0x3ff if i else 0xfe)
        frame = struct.pack("<BB8sI", 32, 1, NODE, message)
        frame += AESCCM(KEY, tag_length=8).encrypt(struct.pack("<8sIB", NODE, message, 1), body, frame)
        ack = struct.pack("<BB8sI", 32, 3, NODE, message)
        ack += AESCCM(KEY, tag_length=8).encrypt(struct.pack("<8sIB", NODE, message, 3), b"\0", ack)
        at = 10_000_000 + i * interval
        p = {r[1]: (0 if r[3] and r[2] == "INTEGER" else None) for r in db.execute("PRAGMA table_info(message_profiles)")}
        p.update(receiver_instance_id=INSTANCE, occurrence_sequence=i + 1, received_at_monotonic_us=at,
            received_frame_length=54, received_frame=frame + bytes(201), claimed_control=32,
            claimed_domain=1, claimed_node_id=NODE, claimed_message_id=message, header_authenticated=1,
            decoded_sample_id=sample, processing_result_id=11, ack_selected_id=3, ack_tx_result_id=5,
            ack_frame=ack, persistence_classification_id=1, t1_handler_started_monotonic_us=at + 1,
            t2_packet_copied_monotonic_us=at + 2, t3_authentication_completed_monotonic_us=at + 3,
            t4_set_tx_attempted_monotonic_us=at + 150000, t5_tx_done_monotonic_us=at + 211696,
            t6_set_rx_issued_monotonic_us=at + 212000)
        insert(db, "message_profiles", p)
        r = dict(zip(FIELDS, BODY.unpack(body), strict=True))
        r.update(node_id=NODE, message_id=message, reading_body=body, is_canonical_for_sample=1,
                 first_receiver_instance_id=INSTANCE, first_occurrence_sequence=i + 1)
        insert(db, "reading_messages", r)
        identity = dict(cycle_sample_id=sample, sample_id=sample, message_id=message, domain=1)
        deliveries.extend([dict(type=4, **identity), dict(type=5, **identity, final_result=1, attempt_count=1)])
    db.execute("UPDATE receiver_instances SET clean_stopped_at_monotonic_us=950000000, clean_stop_state_generation=1")
    db.commit()
    db.close()
    return dict(logs={"pending.log": [], "quarantine.log": None,
                      "diagnostic.log": None, "delivery.log": deliveries})


def test_consistent_real_sqlite_and_two_wake_node_capture(tmp_path):
    path = tmp_path / "receiver.sqlite3"
    decoded = transcript(path)
    output = tmp_path / "snapshot.sqlite3"
    binding = snapshot(path, output)
    assert binding["sha256"] == digest(output)
    assert verify_service(output, NODE, KEY, decoded, INSTANCE)["wakes"] == 2
    observed = database_observation(path)
    assert observed["instance"] == INSTANCE.hex() and observed["samples"] == 2
    with pytest.raises(FileExistsError):
        snapshot(path, output)


@pytest.mark.parametrize("damage", ["frame", "ack", "projection", "missing_timestamp", "no_clean_stop", "wrong_interval", "missing_reading"])
def test_incomplete_or_wrong_sqlite_cannot_pass(tmp_path, damage):
    path = tmp_path / "receiver.sqlite3"
    decoded = transcript(path)
    with sqlite3.connect(path) as db:
        # Corrupt a captured copy, not a live production database.
        for (name,) in db.execute("SELECT name FROM sqlite_master WHERE type='trigger'").fetchall():
            db.execute('DROP TRIGGER "' + name + '"')
        changes = {
            "frame": "UPDATE message_profiles SET received_frame=zeroblob(255) WHERE occurrence_sequence=1",
            "ack": "UPDATE message_profiles SET ack_frame=zeroblob(23) WHERE occurrence_sequence=1",
            "projection": "UPDATE reading_messages SET soil_0_mv=2400 WHERE message_id=100",
            "missing_timestamp": "UPDATE message_profiles SET t5_tx_done_monotonic_us=NULL WHERE occurrence_sequence=1",
            "no_clean_stop": "UPDATE receiver_instances SET clean_stopped_at_monotonic_us=NULL, clean_stop_state_generation=NULL",
            "wrong_interval": "UPDATE message_profiles SET received_at_monotonic_us=11000000 WHERE occurrence_sequence=2",
            "missing_reading": "DELETE FROM reading_messages WHERE message_id=101",
        }
        db.execute(changes[damage])
    with pytest.raises(ValueError):
        verify_service(path, NODE, KEY, decoded, INSTANCE)


@pytest.mark.parametrize("damage", ["missing_finish", "unaccepted", "attempts", "pending", "diagnostic"])
def test_node_consequences_are_required(tmp_path, damage):
    path = tmp_path / "receiver.sqlite3"
    decoded = transcript(path)
    logs = decoded["logs"]
    if damage == "missing_finish":
        logs["delivery.log"].pop()
    elif damage == "unaccepted":
        logs["delivery.log"][1]["final_result"] = 2
    elif damage == "attempts":
        logs["delivery.log"][1]["attempt_count"] = 2
    elif damage == "pending":
        logs["pending.log"] = [dict(sample_id=40)]
    else:
        logs["diagnostic.log"] = [dict(error_code=1)]
    with pytest.raises(ValueError):
        verify_service(path, NODE, KEY, decoded, INSTANCE)


def config():
    return dict(schema=1, unit="cura-pilot-test.service", package="/opt/cura-pilot-test",
                test_root="/var/lib/cura-pilot-test", user="cura-receiver",
                files={"receiver/cura_receiver/__main__.py": "a" * 64})


@pytest.mark.parametrize("field,value", [("unit", "cura-receiver.service"), ("user", "root"),
    ("package", "/opt/cura-agrorum"), ("test_root", "/var/lib/cura-agrorum"),
    ("files", {"../secret": "a" * 64})])
def test_production_or_ambiguous_service_inputs_rejected(field, value):
    value_config = config()
    value_config[field] = value
    with pytest.raises(ValueError):
        validate_config(value_config)


def test_service_template_and_current_bundle():
    template = (REPO / "receiver/deploy/systemd/cura-receiver.service").read_text()
    unit = expected_unit(config(), template)
    assert "/opt/cura-agrorum" not in unit
    assert "cura-pilot-rtc-test.service" in unit
    files = package_files()
    assert not any(".pytest_cache" in Path(name).parts for name in files)
    from inputs import source_manifest
    staged = source_manifest()["files"]
    assert all(staged.get(name) == expected for name, expected in files.items())


@pytest.mark.parametrize("key", ["unit_sha256", "environment_sha256", "configuration_sha256", "boot_id", "InvocationID", "MainPID", "NRestarts", "ActiveState"])
def test_changes_during_episode_fail(key):
    before = dict(unit_sha256="unit", environment_sha256="env", configuration_sha256="group", boot_id="boot",
                  InvocationID="invocation", MainPID="50", NRestarts="0", ActiveState="active", SubState="running")
    after = deepcopy(before)
    after[key] = "changed"
    with pytest.raises(ValueError):
        same_service(before, after, running=before)


def test_failed_service_restoration_does_not_mark_stopped():
    service = InstalledService(type("Remote", (), dict(remote="/var/tmp/cura-rf-test"))(), config())
    service.admin = lambda *_: type("Result", (), dict(stdout="ActiveState=failed\nSubState=failed\nMainPID=0\nResult=exit-code\nExecMainStatus=1\n"))()
    service.probe = lambda *_: {}
    with pytest.raises(ValueError, match="binding"):
        service.stop()
    assert not service.stopped


@pytest.mark.parametrize("damage", [None, "source", "unit", "uid", "dropin", "environment", "credentials_mode"])
def test_installed_source_service_and_environment_guards(tmp_path, monkeypatch, damage):
    import service_probe as probe
    from types import SimpleNamespace
    c = config()
    package = tmp_path / "package"
    root = tmp_path / "root"
    (package / "receiver/cura_receiver").mkdir(parents=True)
    for name in ("config", "data/tmp"):
        (root / name).mkdir(parents=True)
    main = package / "receiver/cura_receiver/__main__.py"
    main.write_text("production source\n")
    main.chmod(0o644)
    c["files"]["receiver/cura_receiver/__main__.py"] = digest(main)
    group = root / "config/receiver-group.json"
    group.write_text("private group fixture")
    group.chmod(0o600)
    env = dict(CURA_RECEIVER_TEST_ROOT=c["test_root"],
        CURA_RECEIVER_CONFIGURATION=c["test_root"] + "/config/receiver-group.json",
        CURA_RECEIVER_DATABASE=c["test_root"] + "/data/receiver.sqlite3",
        SQLITE_TMPDIR=c["test_root"] + "/data/tmp", RTC_HELPER_SHA256="a" * 64, RTC_KERNEL_BOUND_US="3000000")
    unit = tmp_path / "unit.service"
    unit.write_text(expected_unit(c, (REPO / "receiver/deploy/systemd/cura-receiver.service").read_text()))
    state = dict(ActiveState="inactive", SubState="dead", MainPID="0", User="cura-receiver", Group="cura-receiver",
        FragmentPath=str(unit), DropInPaths="", NeedDaemonReload="no", NRestarts="0", InvocationID="", Result="success", ExecMainStatus="0")
    def mapped(path):
        path = Path(path)
        for prefix, replacement in ((c["package"], package), (c["test_root"], root)):
            if path.is_relative_to(prefix):
                return replacement / path.relative_to(prefix)
        return unit
    monkeypatch.setattr(probe, "checked_path", mapped)
    monkeypatch.setattr(probe.pwd, "getpwnam", lambda _: SimpleNamespace(pw_uid=os.geteuid()))
    monkeypatch.setattr(probe, "dependencies", lambda: {})
    original_read = Path.read_bytes
    monkeypatch.setattr(Path, "read_bytes", lambda p: b"pi-board\0" if str(p) == "/proc/device-tree/serial-number" else original_read(p))
    monkeypatch.setattr(probe.subprocess, "check_output", lambda *a, **kw: "\n".join(k + "=" + v for k, v in state.items()))
    if damage == "source":
        main.write_text("stale or wrong image")
    elif damage == "unit":
        unit.write_text(unit.read_text().replace("-m cura_receiver", "-m component_peer"))
    elif damage == "uid":
        state["User"] = "root"
    elif damage == "dropin":
        state["DropInPaths"] = "/unexpected/override.conf"
    elif damage == "environment":
        env["CURA_RECEIVER_DATABASE"] = "/var/lib/cura-agrorum/receiver.sqlite3"
    elif damage == "credentials_mode":
        group.chmod(0o644)
    (root / "config/deployment.env").write_text("\n".join(k + "=" + v for k, v in env.items()))
    if damage:
        with pytest.raises(ValueError):
            probe.inspect(c)
    else:
        assert probe.inspect(c)["MainPID"] == "0"


@pytest.mark.parametrize('exit_pair', [('1', '0'), ('2', '15')])
@pytest.mark.parametrize('damage', [None, 'status', 'code', 'pid', 'cgroup', 'inhibitor',
                                    'instance', 'boot', 'generation', 'marker', 'restart', 'invocation'])
def test_combined_stop_evidence(exit_pair, damage):
    running = dict(unit_sha256='u', environment_sha256='e', configuration_sha256='c',
                   boot_id='aa-bb', InvocationID='inv', MainPID='55', NRestarts='0',
                   ActiveState='active', SubState='running')
    instance = dict(instance='new', ordinal=2, linux_boot_id='aabb', started=100, clean_stop=None)
    state = dict(running, ActiveState='inactive', SubState='dead', MainPID='0', Result='success',
                 InvocationID='', ExecMainCode=exit_pair[0], ExecMainStatus=exit_pair[1],
                 ControlGroup='', remaining_pids=[], receiver_inhibitors=[])
    database = dict(instance, clean_stop=200, clean_generation=3, state_generation=3)
    if damage == 'status': state['ExecMainStatus'] = '9'
    if damage == 'code': state['ExecMainCode'] = '3'
    if damage == 'pid': state['remaining_pids'] = [55]
    if damage == 'cgroup': state['ControlGroup'] = '/system.slice/cura-pilot-test.service'
    if damage == 'inhibitor': state['receiver_inhibitors'] = [['sleep']]
    if damage == 'instance': database['instance'] = 'old'
    if damage == 'boot': database['linux_boot_id'] = 'old'
    if damage == 'generation': database['state_generation'] = 4
    if damage == 'marker': database['clean_stop'] = None
    if damage == 'restart': state['NRestarts'] = '1'
    if damage == 'invocation': state['InvocationID'] = 'other'
    if damage:
        with pytest.raises(ValueError):
            verify_service_stop(running, instance, state, database)
    else:
        assert verify_service_stop(running, instance, state, database)['invocation'] == 'inv'


@pytest.mark.parametrize("elapsed,valid", [
    (9_499_999, False), (9_500_000, True), (12_000_000, True),
    (45_500_000, True), (45_500_001, False), (900_000_000, False),
])
def test_accelerated_wake_bound(tmp_path, elapsed, valid):
    path = tmp_path / "receiver.sqlite3"
    decoded = transcript(path, interval=elapsed)
    if valid:
        assert verify_service(path, NODE, KEY, decoded, INSTANCE)["wakes"] == 2
    else:
        with pytest.raises(ValueError, match="accelerated 10-second wake"):
            verify_service(path, NODE, KEY, decoded, INSTANCE)
