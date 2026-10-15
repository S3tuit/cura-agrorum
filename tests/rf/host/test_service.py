"""service.reading_delivery accepts actual SQLite snapshots and rejects incomplete evidence."""
from copy import deepcopy
import json
import os
from pathlib import Path
import sqlite3
import struct
import subprocess
import shutil
import tarfile
from types import SimpleNamespace

from cryptography.hazmat.primitives.ciphers.aead import AESCCM
import pytest

from capture import REPO, digest
from run_service import package_files, same_service, InstalledService, verify_service_stop
from service_probe import database_observation, snapshot, validate_config, expected_unit
from verify_service import BODY, FIELDS, verify_service

NODE, KEY, INSTANCE = bytes(range(8)), bytes(range(16)), bytes(range(16, 32))


def insert(db, table, values):
    columns = list(values)
    db.execute(f"INSERT INTO {table} ({','.join(columns)}) VALUES ({','.join('?' for _ in columns)})", list(values.values()))


def transcript(path, *, interval=12_000_000, samples=(40, 41)):
    db = sqlite3.connect(path)
    db.executescript((REPO / "receiver/db/schema.sql").read_text())
    db.execute("PRAGMA foreign_keys=ON")
    insert(db, "receiver_instances", dict(instance_ordinal=1, receiver_instance_id=INSTANCE,
        linux_boot_id=b"b" * 16, started_at_monotonic_us=1))
    deliveries = []
    for i, sample in enumerate(samples):
        message = 100 + i
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


@pytest.mark.parametrize('local_note', [False, True], ids=['fresh-checkout', 'local-notes'])
@pytest.mark.parametrize('note_suffix', ['.notes', '.notes.md'])
def test_source_manifest_and_bundle_ignore_planning_notes_and_rf_markdown(tmp_path, monkeypatch, local_note, note_suffix):
    import inputs
    import run_service
    import transport
    repo = tmp_path / 'checkout.notes'
    repo.mkdir()
    # Snapshot the proposed tree, including moves/additions before commit, while
    # excluding ignored workspace artifacts as a fresh checkout would.
    names = subprocess.check_output(
        ['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard'], cwd=REPO,
    ).decode().split('\0')
    for name in set(filter(None, names)):
        source = REPO / name
        if not source.is_file():
            continue  # A deletion in the proposed tree.
        destination = repo / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    # Use a synthetic guide so this fixture does not depend on committed names.
    guide = repo / 'tests/rf/host/capture-guide.md'
    guide.write_text('capture instructions\n')
    assert not (repo / f'deployment_remaining{note_suffix}').exists()
    if local_note:
        (repo / f'deployment_remaining{note_suffix}').write_text('local planning only\n')
        (repo / f'tests/rf/host/session{note_suffix}').write_text('local RF planning only\n')
    # Both manifest builders read HEAD; share metadata without creating a commit.
    git_dir = subprocess.run(['git', 'rev-parse', '--absolute-git-dir'], cwd=REPO,
                             check=True, capture_output=True, text=True).stdout.strip()
    (repo / '.git').write_text(f'gitdir: {git_dir}\n')
    monkeypatch.setattr(inputs, 'REPO', repo)
    monkeypatch.setattr(inputs, 'APP', repo / 'firmware/hardware/radio/tests/component')
    monkeypatch.setattr(run_service, 'REPO', repo)
    monkeypatch.setattr(transport, 'REPO', repo)
    manifest = inputs.source_manifest()
    staged = manifest['files']
    files = run_service.package_files()
    assert f'deployment_remaining{note_suffix}' not in staged
    assert not any(name.startswith('tests/rf/') and name.endswith('.md') for name in staged)
    assert files and all(staged.get(name) == expected for name, expected in files.items())

    # Adding, editing or removing RF documentation must not change staged inputs.
    (repo / 'tests/rf/host/new-guide.md').write_text('new capture instructions\n')
    (repo / 'tests/rf/README.md').write_text('updated RF instructions\n')
    (repo / f'receiver/tests/local{note_suffix}').write_text('local work in progress\n')
    (repo / f'receiver/deploy/session{note_suffix}').write_text('local deployment notes\n')
    (repo / 'receiver/deploy/README.md.bak').write_text('editor backup\n')
    (repo / f'receiver/hardware/radio/tests/carrier/local{note_suffix}').write_text('local fixture notes\n')
    guide.unlink()
    notes = [repo / name for name in (
        'firmware/hardware/radio/tests/component/main/session.notes/saved.c',
        'tests/rf/session.notes.d/measurements.json',
        f'receiver/cura_receiver/session{note_suffix}/saved.py',
        'firmware/components/node_core/scratch.notes.c')]
    fixture = {key: key for key in ('c6_dut', 'pi_board_id', 'c6_transmitter', 'pi_transmitter')}
    identity = inputs.session_identity(manifest, {}, fixture)
    for contents in ('temporary session data\n', 'changed session data\n'):
        for path in notes:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(contents)
        current = inputs.source_manifest()
        assert current == manifest
        assert inputs.session_identity(current, {}, fixture) == identity

    # Inspect the real archive boundary; only network operations are stubbed.
    stage = tmp_path / 'stage'
    stage.mkdir()
    remote = transport.RemoteTransport('example.invalid', 'test', '0' * 32, stage)
    with monkeypatch.context() as network:
        network.setattr(remote, 'command', lambda *a, **kw: None)
        network.setattr(transport.subprocess, 'run', lambda *a, **kw:
                       SimpleNamespace(returncode=0, stdout='', stderr=''))
        remote.stage(current, fixture)
    with tarfile.open(stage / 'source.tar.gz') as archive:
        assert set(archive.getnames()) == set(staged) | {'source-manifest.json', 'rf-fixture.json'}
        assert not any(str(path.relative_to(repo)) in archive.getnames() for path in notes)
    for path in notes:
        path.unlink()
    assert inputs.source_manifest()['files'] == staged
    assert run_service.package_files() == files

    # Executable changes and contracts outside tests/rf remain source-bound.
    for name in ('tests/rf/run.py', 'firmware/hardware/radio/tests/component/main/radio_app.c',
                 'receiver/tests/support/README.md',
                 'receiver/ARCHITECTURE.md', 'receiver/HARDWARE.md', 'receiver/conftest.py',
                 'receiver/hardware/radio/tests/carrier/radio_fixture.py',
                 'receiver/hardware/radio/tests/carrier/test_radio.py', 'firmware/ARCHITECTURE.md',
                 'protocol/protocol-v2-lora/README.md'):
        path = repo / name
        path.write_text(path.read_text() + '\n# changed input\n')
        changed = inputs.source_manifest()['files']
        assert changed.keys() == staged.keys()
        assert changed[name] != staged[name]
        assert {key for key in staged if staged[key] != changed[key]} == {name}
        staged = changed


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


@pytest.mark.parametrize("damage", [None, "source", "unit", "uid", "dropin", "environment", "startup_budget", "credentials_mode"])
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
    env = dict(CURA_RECEIVER_STARTUP_BUDGET_US="12000000",
        CURA_RECEIVER_TEST_ROOT=c["test_root"],
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
    elif damage == "startup_budget":
        env["CURA_RECEIVER_STARTUP_BUDGET_US"] = "1000000"
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
