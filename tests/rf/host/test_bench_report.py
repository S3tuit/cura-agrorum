"""Real SQLite/LittleFS captures and incomplete-observation bench accounting."""
from copy import deepcopy
import json
from pathlib import Path
import sqlite3
import struct

import pytest

from bench_report import (seal_capture, report, read_database, reconcile,
                          DATABASE_SCHEMA_VERSION, DATABASE_SCHEMA_FINGERPRINT, SQLITE_APPLICATION_ID)
from evidence import REPO, digest
from host.test_service import transcript, NODE
from host.test_node_capture import binaries, image_from, record

GROUP = '1122334455667788'
EMPTY = dict(readings=[], profiles=[], instances=[])
LOGS = {k: [] for k in ('pending.log', 'quarantine.log', 'delivery.log', 'diagnostic.log')}


def metadata(path):
    with sqlite3.connect(path) as db:
        db.execute(f'PRAGMA application_id={SQLITE_APPLICATION_ID}')
        db.execute('INSERT INTO database_metadata VALUES (1,?,?,?)',
                   (bytes.fromhex(GROUP), DATABASE_SCHEMA_VERSION, DATABASE_SCHEMA_FINGERPRINT))


def observed(tmp_path):
    path = tmp_path / 'receiver.sqlite3'
    logs = transcript(path)['logs']
    metadata(path)
    return read_database(path, GROUP, NODE.hex()), logs


def test_accounting_does_not_invent_received_attempts(tmp_path):
    after, logs = observed(tmp_path)
    # Sample41 was not received durably, but remains on the node.
    retained = after['readings'].pop()
    after['profiles'].pop()
    logs['pending.log'] = [dict(type=1, sample_id=41, reading_body=retained['reading_body'].hex())]
    logs['delivery.log'][-1].update(final_result=6, attempt_count=7)
    result = reconcile(EMPTY, after, LOGS, logs)
    assert result['counts']['observed_samples'] == 2
    assert result['counts']['node_attempts_recorded'] == 8
    assert result['counts']['received_authenticated_profiles'] == 1
    assert result['counts']['durable_samples'] == 1
    assert result['counts']['retained_only_samples'] == 1
    assert result['missing_from_both'] == []
    assert 'verdict' in result['scope']


def test_missing_from_both_is_not_hidden_by_node_acceptance(tmp_path):
    after, logs = observed(tmp_path)
    after['readings'].pop()
    after['profiles'].pop()
    result = reconcile(EMPTY, after, LOGS, logs)
    assert result['missing_from_both'] == [41]
    assert result['accepted_without_profile'] == [101]


def test_incomplete_delivery_reports_unknown_attempt_count(tmp_path):
    after, logs = observed(tmp_path)
    logs['delivery.log'].pop()
    result = reconcile(EMPTY, after, LOGS, logs)
    assert result['counts']['node_attempts_recorded'] == 1
    assert result['unfinished_deliveries'] == [[41, 41, 101, 1]]
    assert result['observation_gaps']


def test_conflicting_retained_body_is_not_silently_counted(tmp_path):
    after, logs = observed(tmp_path)
    raw = bytearray(after['readings'][0]['reading_body']); raw[6] ^= 1
    logs['pending.log'] = [dict(type=1, sample_id=40, reading_body=raw.hex())]
    assert reconcile(EMPTY, after, LOGS, logs)['conflicting_bodies'] == [40]


@pytest.mark.parametrize('damage', ['duplicate_profile', 'duplicate_delivery', 'lost_row', 'truncated_delivery'])
def test_inconsistent_histories_rejected(tmp_path, damage):
    after, logs = observed(tmp_path)
    before, initial = deepcopy(EMPTY), deepcopy(LOGS)
    if damage == 'duplicate_profile': after['profiles'].append(after['profiles'][0])
    elif damage == 'duplicate_delivery': logs['delivery.log'].append(logs['delivery.log'][0])
    elif damage == 'lost_row':
        before = deepcopy(after); after['readings'].pop()
    else:
        initial = deepcopy(logs); logs['delivery.log'].pop()
    with pytest.raises(ValueError): reconcile(before, after, initial, logs)


def test_baseline_is_excluded_but_starting_backlog_is_retained(tmp_path):
    after, logs = observed(tmp_path)
    before = deepcopy(after)
    raw = after['readings'][0]['reading_body'].hex()
    initial = deepcopy(logs)
    initial['pending.log'] = [dict(type=1, sample_id=40, reading_body=raw)]
    result = reconcile(before, after, initial, logs)
    assert result['counts']['observed_samples'] == 1
    assert result['counts']['durable_samples'] == 1
    assert result['counts']['node_attempts_recorded'] == 0


@pytest.fixture
def captures(tmp_path, binaries):
    before = tmp_path / 'before'; after = tmp_path / 'after'
    before.mkdir(); after.mkdir()
    empty_db = before / 'receiver.sqlite3'
    with sqlite3.connect(empty_db) as db:
        db.executescript((REPO / 'receiver/db/schema.sql').read_text())
    metadata(empty_db)
    data = transcript(after / 'receiver.sqlite3')['logs']; metadata(after / 'receiver.sqlite3')
    raw = b''
    for r in data['delivery.log']:
        identity = (r['cycle_sample_id'], r['sample_id'], r['message_id'], r['domain'])
        payload = (struct.pack('<IIIBI', *identity, 10) if r['type'] == 4 else
                   struct.pack('<IIIBBB', *identity, r['attempt_count'], r['final_result']))
        raw += record(r['type'], payload)
    images = [image_from(before, binaries, {'pending.log': b''}),
              image_from(after, binaries, {'pending.log': b'', 'delivery.log': raw})]
    build = tmp_path / 'build.json'
    build.write_text(json.dumps(dict(node_id=NODE.hex(), files={
        'cura_agrorum_firmware.bin': 'a' * 64, 'partition_table/partition-table.bin': 'b' * 64})))
    service = tmp_path / 'service.json'
    service.write_text(json.dumps(dict(schema=1, unit='cura-pilot-bench.service', package='/opt/cura-pilot-bench',
        test_root='/var/lib/cura-pilot-bench', user='cura-receiver',
        files={'receiver/cura_receiver/__main__.py': 'c' * 64})))
    manifests = []
    for root, image in zip((before, after), images):
        binding = root / 'binding.json'
        binding.write_text(json.dumps(dict(node_id=NODE.hex(), image_sha256=digest(image),
            application_sha256='a' * 64, partition_sha256='b' * 64, size=image.stat().st_size,
            c6_dut='0123456789ab')))
        path = root / 'capture.json'
        seal_capture(root / 'receiver.sqlite3', image, binding, build, service, GROUP, path)
        manifests.append(path)
    return manifests


def test_real_captures_report_without_mutating_inputs(tmp_path, captures):
    originals = {p: digest(p) for p in tmp_path.rglob('*') if p.is_file()}
    result = report(*captures, tmp_path / 'report.json')
    assert result['counts']['durable_samples'] == 2
    assert result['counts']['node_accepted_deliveries'] == 2
    assert not result['missing_from_both']
    assert all(digest(p) == h for p, h in originals.items())
    with pytest.raises(FileExistsError): report(*captures, tmp_path / 'report.json')


@pytest.mark.parametrize('damage', ['changed_image', 'configuration', 'node_identity', 'missing_file'])
def test_capture_mismatch_rejected(tmp_path, captures, damage):
    value = json.loads(captures[1].read_text())
    if damage == 'changed_image': Path(value['artifacts']['image']['path']).write_bytes(b'broken')
    elif damage == 'missing_file': Path(value['artifacts']['image']['path']).unlink()
    elif damage == 'node_identity':
        value['c6_dut'] = 'ffffffffffff'; captures[1].write_text(json.dumps(value))
    else:
        service = tmp_path / 'other-service.json'
        original = json.loads(Path(value['artifacts']['service']['path']).read_text())
        original['files']['receiver/cura_receiver/__main__.py'] = 'd' * 64
        service.write_text(json.dumps(original))
        value['artifacts']['service'] = dict(path=str(service), sha256=digest(service))
        captures[1].write_text(json.dumps(value))
    with pytest.raises((ValueError, FileNotFoundError)): report(*captures, tmp_path / 'report.json')


def test_database_identity_and_live_wal_rejected(tmp_path):
    path = tmp_path / 'receiver.sqlite3'; transcript(path); metadata(path)
    with pytest.raises(ValueError, match='group/schema'): read_database(path, '00' * 8, NODE.hex())
    Path(str(path) + '-wal').touch()
    with pytest.raises(ValueError, match='standalone'): read_database(path, GROUP, NODE.hex())


def test_retry_across_capture_boundary_is_not_a_new_transport(tmp_path):
    after, logs = observed(tmp_path)
    before = deepcopy(after)
    retry = deepcopy(after['profiles'][0])
    retry['occurrence_sequence'] = 3
    retry['received_at_monotonic_us'] += 1000000000
    after['profiles'].append(retry)
    result = reconcile(before, after, logs, logs)
    assert result['counts']['received_retries'] == 1
    assert result['counts']['durable_transport_rows_added'] == 0
    assert result['messages'][0]['message_id'] == 100
    assert result['messages'][0]['prior_received_profiles'] == 1


def test_reboot_and_missing_clean_stop_are_reported_not_relabelled(tmp_path):
    after, logs = observed(tmp_path)
    after['instances'][0]['clean_stopped_at_monotonic_us'] = None
    new = deepcopy(after['instances'][0])
    new.update(receiver_instance_id=b'x' * 16, linux_boot_id=b'y' * 16, instance_ordinal=2)
    after['instances'].append(new)
    result = reconcile(EMPTY, after, LOGS, logs)
    assert len(result['receiver_instances']) == 2
    assert all(not i['clean_stop_recorded'] for i in result['receiver_instances'])
    assert 'no UTC' in result['time_basis']
