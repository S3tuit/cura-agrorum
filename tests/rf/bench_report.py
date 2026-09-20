"""Local-only bench capture sealing and sample reconciliation; never a soak verdict."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sqlite3
import sys
import tempfile

from evidence import REPO, digest
from node_capture import build_reader, decode_image
from service_probe import validate_config

sys.path[:0] = [str(REPO / 'receiver'), str(REPO / 'protocol/protocol-v2-lora/python')]
from cura_receiver.generated.receiver_enums_generated import (
    DATABASE_SCHEMA_FINGERPRINT, DATABASE_SCHEMA_VERSION, SQLITE_APPLICATION_ID, AckTxResult,
)
from cura_receiver.generated.protocol_v2_lora_generated import decode_reading


def read_database(path, group, node):
    path = Path(path).resolve()
    if any(Path(str(path) + suffix).exists() for suffix in ('-wal', '-journal')):
        raise ValueError('use a standalone consistent SQLite backup, not a live database')
    with sqlite3.connect(path.as_uri() + '?mode=ro&immutable=1', uri=True) as db:
        db.row_factory = sqlite3.Row
        if db.execute('PRAGMA integrity_check').fetchone()[0] != 'ok' or db.execute('PRAGMA foreign_key_check').fetchall():
            raise ValueError('inconsistent database')
        meta = db.execute('SELECT * FROM database_metadata').fetchall()
        if (len(meta) != 1 or meta[0]['group_id'] != bytes.fromhex(group) or
                meta[0]['database_schema_version'] != DATABASE_SCHEMA_VERSION or
                meta[0]['database_schema_fingerprint'] != DATABASE_SCHEMA_FINGERPRINT or
                db.execute('PRAGMA application_id').fetchone()[0] != SQLITE_APPLICATION_ID):
            raise ValueError('database group/schema mismatch')
        return dict(
            readings=[dict(r) for r in db.execute('SELECT * FROM reading_messages WHERE node_id=?', (bytes.fromhex(node),))],
            profiles=[dict(r) for r in db.execute('SELECT * FROM message_profiles WHERE claimed_node_id=? AND header_authenticated=1', (bytes.fromhex(node),))],
            instances=[dict(r) for r in db.execute('SELECT * FROM receiver_instances ORDER BY instance_ordinal')],
        )


def seal_capture(database, image, binding, build, service, group, output):
    """Bind already collected artifacts; this neither snapshots a live DB nor touches devices."""
    paths = {k: Path(v).resolve() for k, v in dict(database=database, image=image,
             binding=binding, build=build, service=service).items()}
    if any(not p.is_file() for p in paths.values()):
        raise ValueError('missing capture artifact')
    artifacts = {k: dict(path=str(p), sha256=digest(p)) for k, p in paths.items()}
    production = json.loads(paths['build'].read_text())
    config = validate_config(json.loads(paths['service'].read_text()))
    node = production['node_id']
    if len(bytes.fromhex(node)) != 8 or len(bytes.fromhex(group)) != 8:
        raise ValueError('invalid identity')
    b = json.loads(paths['binding'].read_text())
    if (b['node_id'] != node or b['image_sha256'] != artifacts['image']['sha256'] or
            b['application_sha256'] != production['files']['cura_agrorum_firmware.bin'] or
            b['partition_sha256'] != production['files']['partition_table/partition-table.bin'] or
            paths['image'].stat().st_size != b['size']):
        raise ValueError('node capture/build binding mismatch')
    read_database(paths['database'], group, node)
    value = dict(schema=1, node_id=node, group_id=group, artifacts=artifacts,
                 service_unit=config['unit'], c6_dut=b['c6_dut'])
    # Recheck artifacts after inspection; no moving input may be sealed.
    if any(digest(paths[k]) != v['sha256'] for k, v in artifacts.items()):
        raise ValueError('capture changed while sealing')
    with Path(output).open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write('\n')
    return value


def load_capture(manifest, reader):
    value = json.loads(Path(manifest).read_text())
    if value.get('schema') != 1 or set(value['artifacts']) != {'database', 'image', 'binding', 'build', 'service'}:
        raise ValueError('invalid capture manifest')
    for artifact in value['artifacts'].values():
        if digest(Path(artifact['path'])) != artifact['sha256']:
            raise ValueError('capture artifact changed')
    db = read_database(value['artifacts']['database']['path'], value['group_id'], value['node_id'])
    logs = decode_image(Path(value['artifacts']['image']['path']), reader)['logs']
    return value, db, logs


def reconcile(before, after, initial, final):
    """Compare validated SQLite observations and production-decoded log records."""
    gaps, conflicts = [], []
    # Receiver rows/profiles are append-only across receiver reboots.
    def indexed(rows, fields):
        result = {}
        for row in rows:
            key = tuple(row[f] for f in fields)
            if key in result:
                raise ValueError('duplicate observation identity')
            result[key] = row
        return result
    old_rows = indexed(before['readings'], ('node_id', 'message_id'))
    new_rows = indexed(after['readings'], ('node_id', 'message_id'))
    old_profiles = indexed(before['profiles'], ('receiver_instance_id', 'occurrence_sequence'))
    new_profiles = indexed(after['profiles'], ('receiver_instance_id', 'occurrence_sequence'))
    for old, new in ((old_rows, new_rows), (old_profiles, new_profiles)):
        if any(new.get(k) != v for k, v in old.items()):
            raise ValueError('receiver observation disappeared or changed')
    prior_delivery = initial['delivery.log'] or []
    delivery = final['delivery.log'] or []
    if delivery[:len(prior_delivery)] != prior_delivery:
        raise ValueError('delivery log truncated/replaced; interval coverage unknown')
    delta = delivery[len(prior_delivery):]
    observed, bodies, durable, retained = set(), {}, set(), set()
    baseline = {r['sample_id'] for r in before['readings']}
    baseline.update(r['sample_id'] for r in prior_delivery)
    new_evidence = {r['sample_id'] for r in delta}
    new_evidence.update(r['sample_id'] for k, r in new_rows.items() if k not in old_rows)
    initial_retained = set()

    def body(sample, raw):
        parsed = decode_reading(raw)
        if parsed.sample_id != sample:
            raise ValueError('sample/body identity mismatch')
        if sample in bodies and bodies[sample] != raw:
            conflicts.append(sample)
        bodies.setdefault(sample, raw)
        observed.add(sample)

    for logs, is_final in ((initial, False), (final, True)):
        for name in ('pending.log', 'quarantine.log'):
            for r in logs[name] or []:
                if r['type'] in (1, 2):
                    body(r['sample_id'], bytes.fromhex(r['reading_body']))
                    (retained if is_final else initial_retained).add(r['sample_id'])
    canonical = Counter()
    for r in after['readings']:
        body(r['sample_id'], r['reading_body'])
        if r['is_canonical_for_sample']:
            durable.add(r['sample_id']); canonical[r['sample_id']] += 1
    if any(n != 1 for n in canonical.values()):
        raise ValueError('duplicate canonical sample')
    profiles = [r for k, r in new_profiles.items() if k not in old_profiles]
    received = Counter()
    prior_received = Counter(p['claimed_message_id'] for p in before['profiles'])
    for p in profiles:
        if p['decoded_sample_id'] is not None:
            observed.add(p['decoded_sample_id']); new_evidence.add(p['decoded_sample_id'])
        received[p['claimed_message_id']] += 1
    observed.update(r['sample_id'] for r in delta)
    # Include starting backlog even when it was already recorded by the receiver.
    scope = (observed - baseline) | new_evidence | initial_retained
    starts, finished = {}, {}
    for r in delta:
        identity = (r['cycle_sample_id'], r['sample_id'], r['message_id'], r['domain'])
        target = starts if r['type'] == 4 else finished if r['type'] == 5 else None
        if target is None or identity in target:
            raise ValueError('duplicate/invalid delivery boundary')
        target[identity] = r
    if any(k not in starts for k in finished):
        gaps.append('delivery outcome without interval-local start')
    unfinished = [list(k) for k in starts if k not in finished]
    if unfinished:
        gaps.append('delivery start without outcome; actual attempt count unknown')
    currents = sorted({r['cycle_sample_id'] for r in delta if r['domain'] == 1})
    # Report ranges instead of allocating potentially huge counter gaps.
    counter_gaps = [[a + 1, b - 1] for a, b in zip(currents, currents[1:]) if b > a + 1]
    if counter_gaps:
        gaps.append('sample counter gaps do not establish whether readings were generated')
    quarantined = {r['sample_id'] for r in final['quarantine.log'] or [] if r['type'] == 2}
    missing = sorted(scope - durable - retained)
    absent_profiles = sorted({r['message_id'] for r in finished.values() if r['final_result'] == 1} - set(received))
    if absent_profiles:
        gaps.append('node-accepted messages without interval receiver profiles; volatile profile loss is possible')
    transport_ids = set(received) | {r['message_id'] for r in delta}
    messages = []
    for message in sorted(transport_ids):
        samples = {p['decoded_sample_id'] for p in profiles if p['claimed_message_id'] == message
                   and p['decoded_sample_id'] is not None}
        samples.update(r['sample_id'] for r in delta if r['message_id'] == message)
        outcomes = [r for r in finished.values() if r['message_id'] == message]
        if len(samples) > 1:
            raise ValueError('transport identity maps to conflicting samples')
        messages.append(dict(message_id=message, sample_ids=sorted(samples),
                             received_profiles=received[message], prior_received_profiles=prior_received[message],
                             recorded_node_attempts=sum(r['attempt_count'] for r in outcomes),
                             node_outcomes=[dict(cycle_sample_id=r['cycle_sample_id'], domain=r['domain'],
                                                 final_result=r['final_result']) for r in outcomes]))
    return dict(
        scope='observed samples only; not an automatic bench acceptance verdict',
        counts=dict(observed_samples=len(scope), durable_samples=len(scope & durable),
                    retained_samples=len(scope & retained), retained_only_samples=len(scope & retained - durable),
                    node_attempts_recorded=sum(r['attempt_count'] for r in finished.values()),
                    received_authenticated_profiles=len(profiles), received_transport_ids=len(received),
                    received_retries=sum(n if prior_received[m] else max(0, n - 1) for m, n in received.items()),
                    durable_transport_rows_added=len(new_rows.keys() - old_rows.keys()),
                    receiver_ack_tx_done_profiles=sum(p['ack_tx_result_id'] == AckTxResult.TX_DONE.value for p in profiles),
                    node_accepted_deliveries=sum(r['final_result'] == 1 for r in finished.values())),
        messages=messages,
        samples=[dict(sample_id=s, durable=s in durable, retained=s in retained,
                      quarantined=s in quarantined) for s in sorted(scope)],
        missing_from_both=missing, conflicting_bodies=sorted(set(conflicts) & scope),
        unfinished_deliveries=unfinished, sample_counter_gap_ranges=counter_gaps,
        accepted_without_profile=absent_profiles, observation_gaps=gaps,
        receiver_instances=[dict(instance_id=r['receiver_instance_id'].hex(), boot_id=r['linux_boot_id'].hex(),
                                 clean_stop_recorded=r['clean_stopped_at_monotonic_us'] is not None)
                            for r in after['instances']],
        diagnostics=final['diagnostic.log'] or [],
        time_basis='receiver monotonic timestamps belong to their boot; no UTC or cross-boot latency inferred',
        limits=['No proof of samples unobserved by all inputs, including before first/after last durable boundary.',
                'Recorded node attempts exclude unfinished deliveries and unavailable/full log records.',
                'Power-cut volatile observation loss is not automatically sample loss.',
                'Review duration, interventions, supply continuity, airtime and recovery evidence separately.'])


def report(before_path, after_path, output):
    with tempfile.TemporaryDirectory(prefix='cura-bench-reader-') as directory:
        reader = Path(directory) / 'reader'
        sources = build_reader(reader)
        bm, before, initial = load_capture(before_path, reader)
        am, after, final = load_capture(after_path, reader)
        if any(bm[k] != am[k] for k in ('node_id', 'group_id', 'service_unit', 'c6_dut')) or any(
                bm['artifacts'][k]['sha256'] != am['artifacts'][k]['sha256'] for k in ('build', 'service')):
            raise ValueError('source/configuration/identity changed across captures')
        result = reconcile(before, after, initial, final)
        result.update(schema=1, node_id=am['node_id'], group_id=am['group_id'], decoder_sources=sources,
                      inputs={str(p): digest(Path(p)) for p in (before_path, after_path)})
        for capture in (bm, am):
            if any(digest(Path(v['path'])) != v['sha256'] for v in capture['artifacts'].values()):
                raise ValueError('capture changed during analysis')
    with Path(output).open('x') as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write('\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    seal = sub.add_parser('seal')
    for name in ('database', 'image', 'binding', 'build', 'service', 'output'):
        seal.add_argument('--' + name, type=Path, required=True)
    seal.add_argument('--group', required=True)
    analyze = sub.add_parser('report')
    for name in ('before', 'after', 'output'):
        analyze.add_argument('--' + name, type=Path, required=True)
    args = vars(parser.parse_args()); command = args.pop('command')
    if command == 'seal':
        seal_capture(**args)
    else:
        report(args['before'], args['after'], args['output'])


if __name__ == '__main__':
    main()
