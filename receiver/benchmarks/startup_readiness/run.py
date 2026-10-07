#!/usr/bin/env python3
"""Explicit Pi startup characterization. Preparation/arming never runs at boot."""

import argparse
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import sqlite3
import statistics
import sys
import tarfile
import time
from uuid import uuid4

SOURCE = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(SOURCE / 'receiver'), str(SOURCE / 'protocol/protocol-v2-lora/python')]
SENTINEL = 'CURA STARTUP BENCHMARK V1\n'
FIXTURES = ('pilot', 'large', 'wal')
OBSERVATION_BUDGET_US = 60_000_000


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write_json(path, value):
    """Offline preparation / post-measurement evidence, never a stage callback."""
    if path.exists():
        raise FileExistsError(path)
    temporary = path.with_name(path.name + '.partial')
    with temporary.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def check_root(root):
    if not root.is_absolute() or root.is_symlink() or root.resolve() != root:
        raise ValueError('benchmark root must be canonical and absolute')
    if (root / '.startup-benchmark').read_text() != SENTINEL:
        raise ValueError('benchmark root sentinel mismatch')
    return root


def source_manifest():
    paths = list((SOURCE / 'receiver/cura_receiver').rglob('*.py'))
    paths += list((SOURCE / 'receiver/db').glob('*.sql'))
    paths += list((SOURCE / 'protocol/protocol-v2-lora/python').rglob('*.py'))
    paths += list((SOURCE / 'receiver/benchmarks/startup_readiness').glob('*'))
    paths += list((SOURCE / 'receiver/tests/support/builders').glob('*.py'))
    paths += [SOURCE / 'receiver/tests/__init__.py', SOURCE / 'receiver/tests/support/__init__.py']
    paths += list((SOURCE / 'receiver/deploy/systemd').glob('*'))
    return {str(path.relative_to(SOURCE)): digest(path) for path in sorted(paths) if path.is_file()}


def prepare(root, scale, source_commit):
    from cura_receiver.database_initializer import initialize_database
    from cura_receiver.generated import receiver_entities_generated as rows
    from cura_receiver.generated.receiver_enums_generated import PersistQueueEntityKind, PersistenceClassification
    from cura_receiver.ordinary_persistence import OrdinaryPersistence
    from cura_receiver.persist_queue import PersistQueue
    from cura_receiver.platform.linux_clocks import LinuxOsClock
    from cura_receiver.receiver_startup import ReceiverInstanceStart, insert_receiver_instance_start
    from cura_receiver.sqlite_database import open_receiver_database
    from cura_receiver.sqlite_repository import SqliteRepository
    from tests.support.builders.persistence import GROUP, INSTANCE, _health_request, _observation, _profile
    from tests.support.builders.persistence_control import synthetic

    if not root.is_absolute() or root.exists() or root.resolve() != root:
        raise ValueError('prepare requires a new canonical absolute root')
    root.mkdir(mode=0o750)
    (root / '.startup-benchmark').write_text(SENTINEL)
    for name in ('templates', 'attempts', 'results', 'boot-claims', 'tmp'):
        (root / name).mkdir()
    write_json(root / 'receiver-group.json', dict(format_version=1, group_id=GROUP.hex(),
        group_master_key='00' * 32, active_node_ids=[], retired_node_ids=[]))
    (root / 'receiver-group.json').chmod(0o600)
    inventory = dict(format_version=1, source_commit=source_commit,
        source_files=source_manifest(), fixture_scale=scale, fixtures={},
        fixture_basis='Synthetic current-schema rows: approximately 10000 health, 10000 clock and 800 profiles for pilot; large is 10x. Historical pilot database was about 3 MiB. These are controlled size/history fixtures, not a byte replay of the historical database.',
        prepared_on=dict(hostname=platform.node(), python=sys.version, sqlite=sqlite3.sqlite_version))
    for name in FIXTURES:
        directory = root / 'templates' / name
        directory.mkdir()
        path = directory / 'receiver.sqlite3'
        initialize_database(path, GROUP, known_empty_airtime=False)
        database = open_receiver_database(path, GROUP, minimum_free_bytes=0).database
        db = database.connection
        try:
            instance = ReceiverInstanceStart(INSTANCE, 0)
            insert_receiver_instance_start(db, instance, bytes(16))
            owner = OrdinaryPersistence(database, PersistQueue(), instance=instance, clock=LinuxOsClock())
            health = owner._prepare(_health_request(), PersistQueueEntityKind.RECEIVER_HEALTH_REQUEST).row
            repository = SqliteRepository(db)
            factor = scale * (10 if name == 'large' else 1)
            health_count = max(1, round(10000 * factor))
            profile_count = max(1, round(800 * factor))
            db.execute('BEGIN IMMEDIATE')
            for sequence in range(1, health_count + 1):
                repository.insert_receiver_health(replace(health, health_sequence=sequence))
                repository.insert_clock_observation(_observation(sequence=sequence))
            for sequence in range(1, profile_count + 1):
                repository.insert_message_profile(rows.MessageProfileRowV1(
                    _profile(sequence=sequence), PersistenceClassification.NOT_APPLICABLE))
            db.execute('INSERT INTO communicator_state VALUES (?,?,?,?,?)', rows.communicator_state_v2_parameters(synthetic()))
            db.execute('COMMIT')
            assert db.execute('PRAGMA integrity_check').fetchall() == [('ok',)]
            assert db.execute('PRAGMA foreign_key_check').fetchall() == []
            if name != 'wal':
                assert db.execute('PRAGMA wal_checkpoint(TRUNCATE)').fetchone() == (0, 0, 0)
        finally:
            database.close()  # Production NO_CKPT_ON_CLOSE preserves inherited WAL.
        # SQLite reconstructs SHM for each private attempt; templates are never opened by the runner.
        files = {p.name: dict(bytes=p.stat().st_size, sha256=digest(p))
                 for p in directory.iterdir() if p.name in ('receiver.sqlite3', 'receiver.sqlite3-wal')}
        if name == 'wal' and files.get('receiver.sqlite3-wal', {}).get('bytes', 0) == 0:
            raise RuntimeError('WAL fixture did not retain committed WAL')
        inventory['fixtures'][name] = dict(files=files, health_rows=health_count,
            clock_rows=health_count, profile_rows=profile_count, valid_state=True,
            inherited_wal=name == 'wal', shm_reconstructed=True)
    write_json(root / 'inventory.json', inventory)
    print(json.dumps({name: value['files'] for name, value in inventory['fixtures'].items()}, indent=2))


def arm(root, fixture, mode, label):
    check_root(root)
    if (root / 'armed.json').exists():
        raise ValueError('an attempt is already armed; preserve or explicitly cancel it')
    inventory = json.loads((root / 'inventory.json').read_text())
    attempt = f'{time.time_ns()}-{uuid4().hex[:8]}'
    directory = root / 'attempts' / attempt
    directory.mkdir()
    files = inventory['fixtures'][fixture]['files']
    for name, info in files.items():
        source = root / 'templates' / fixture / name
        if digest(source) != info['sha256']:
            raise ValueError('fixture template changed')
        destination = directory / name
        shutil.copyfile(source, destination)
        if digest(destination) != info['sha256']:
            raise ValueError('fixture copy mismatch')
        with destination.open('rb') as stream:
            os.fsync(stream.fileno())
    value = dict(format_version=1, attempt_id=attempt, fixture=fixture, mode=mode, label=label,
        armed_boot_id=boot_id(), armed_at_unix_ns=time.time_ns(), fixture_files=files,
        inventory_sha256=digest(root / 'inventory.json'), observation_budget_us=OBSERVATION_BUDGET_US)
    write_json(directory / 'request.json', value)
    # Exclusive publication prevents an accidental overlapping armer from replacing a request.
    with (root / 'armed.json').open('x') as stream:
        json.dump(value, stream)
        stream.flush()
        os.fsync(stream.fileno())
    print(attempt)
    return attempt


def post_cleanup_startup(worker, decision, instance_id, clock):
    """Supplementary evidence only; never revise the frozen startup decision."""
    from cura_receiver.persistence_startup import startup_failure

    snapshot = worker.startup_evidence(deadline_monotonic_us=decision.deadline_monotonic_us,
                                       nonblocking=True)
    observed = snapshot.observed_at_monotonic_us if snapshot else clock.now_monotonic_us()
    evidence = replace(decision, snapshot=snapshot, observed_at_monotonic_us=observed).summary(instance_id)
    evidence.pop('outcome')
    result = snapshot.result if snapshot else None
    published = result.published_at_monotonic_us if result else None
    evidence.update(
        terminal_outcome=None if result is None else startup_failure(result) or 'SUCCESS',
        published_at_or_after_deadline=None if published is None else published >= decision.deadline_monotonic_us,
        published_after_initial_observation=None if published is None else published > decision.observed_at_monotonic_us,
        state_load_status=None if result is None or result.state_load is None else result.state_load.status.name,
    )
    return evidence


def run(root):
    from cura_receiver.persistence_startup import observe_persistence_startup
    from cura_receiver.persistence_worker import PersistenceWorker
    from cura_receiver.platform.linux_clocks import LinuxOsClock
    from cura_receiver.platform.linux_process_ownership import claim_receiver_process
    from cura_receiver.platform.linux_signal_wait import CompletionNotification, LinuxSignalWait
    from cura_receiver.receiver_startup import create_receiver_instance
    from cura_receiver.service_evidence import emit_service_evidence
    from cura_receiver.stop_intent import StopIntent

    check_root(root)
    armed = root / 'armed.json'
    if not armed.exists():
        return 0
    request = json.loads(armed.read_text())
    current_boot = boot_id()
    if request['mode'] in ('reboot', 'cold') and request['armed_boot_id'] == current_boot:
        return 0  # Boot experiments cannot be consumed by an accidental manual start.
    claim_receiver_process()
    attempt = request['attempt_id']
    directory = root / 'attempts' / attempt
    if directory.parent != root / 'attempts' or '/' in attempt or '..' in attempt:
        raise ValueError('invalid attempt identity')
    if request['mode'] in ('reboot', 'cold'):
        (root / 'boot-claims' / current_boot).mkdir()  # Exactly one boot sample per boot.
    os.rename(armed, directory / f'started.{current_boot}.json')
    os.environ['SQLITE_TMPDIR'] = str(root / 'tmp')
    clock = LinuxOsClock()
    instance = create_receiver_instance(clock)
    stop = StopIntent(clock, 10_000_000)
    with CompletionNotification() as notification, LinuxSignalWait(clock, stop) as wait:
        worker = PersistenceWorker(instance=instance, database_path=directory / 'receiver.sqlite3',
            configuration_path=root / 'receiver-group.json', expected_owner_uid=os.geteuid(),
            clock=clock, minimum_free_bytes=1 << 30, startup_notification=notification)
        decision = observe_persistence_startup(worker=worker, clock=clock, stop_intent=stop,
            wait=wait, budget_us=request['observation_budget_us'])
        summary = decision.summary(instance.receiver_instance_id)
        summary.update(attempt_id=attempt, fixture=request['fixture'], mode=request['mode'],
            label=request['label'], boot_id=current_boot, boot_age_us=decision.begin_monotonic_us,
            observation_budget_us=request['observation_budget_us'],
            inventory_sha256=request['inventory_sha256'], python=sys.version,
            sqlite=sqlite3.sqlite_version, kernel=platform.release(), uid=os.geteuid(),
            state_load_status=None if decision.snapshot is None or decision.snapshot.result is None or decision.snapshot.result.state_load is None else decision.snapshot.result.state_load.status.name)
        summary['journal_emission'] = emit_service_evidence(summary)
        # Stop intent/deadline precede post-measurement disk output. Neither this
        # persistence-only fixture nor its cleanup manufactures a radio clean-stop marker.
        stop.request()
        worker.request_stop(deadline_monotonic_us=stop.deadline_monotonic_us)
        write_json(directory / 'startup.json', summary)
        if worker.ident is not None:
            worker.join(max(0, stop.deadline_monotonic_us - clock.now_monotonic_us()) / 1_000_000)
        summary['cleanup'] = dict(worker_stopped=not worker.is_alive(),
            deadline_monotonic_us=stop.deadline_monotonic_us,
            observed_at_monotonic_us=clock.now_monotonic_us())
        summary['post_cleanup_startup'] = post_cleanup_startup(
            worker, decision, instance.receiver_instance_id, clock)
        write_json(root / 'results' / f'{attempt}.json', summary)
        return 0 if decision.outcome == 'SUCCESS' and not worker.is_alive() else 1


def analyze(root):
    check_root(root)
    groups = {}
    records = []
    for directory in sorted((root / 'attempts').iterdir()):
        request_path = directory / 'request.json'
        if not request_path.exists():
            continue
        request = json.loads(request_path.read_text())
        result_path = root / 'results' / f'{directory.name}.json'
        result = json.loads(result_path.read_text()) if result_path.exists() else None
        status = result['outcome'] if result else ('INCOMPLETE' if list(directory.glob('started.*.json')) else 'NOT_STARTED')
        confirmation = ((directory / 'physical-confirmation.json').exists()
                        if request['mode'] == 'cold' else None)
        verified_cold = request['mode'] != 'cold' or confirmation
        # Warm/smoke samples are same-boot process starts; reboot/cold samples
        # need a new boot. A warm request consumed at boot (for example after
        # an unexpected restart) ran with a cold cache: retain it, never count it.
        boot_eligible = None if result is None else (
            (result['boot_id'] == request['armed_boot_id']) == (request['mode'] in ('smoke', 'warm')))
        records.append(dict(attempt_id=directory.name, fixture=request['fixture'], mode=request['mode'],
                            label=request['label'], observation_budget_us=request['observation_budget_us'],
                            inventory_sha256=request['inventory_sha256'],
                            outcome=status, physical_confirmation=confirmation,
                            boot_eligible=boot_eligible,
                            boot_id=None if result is None else result['boot_id'],
                            cleanup_worker_stopped=None if result is None else result['cleanup']['worker_stopped']))
        key = (f"{request['fixture']}/{request['mode']}/{request['observation_budget_us']}us/"
               f"{request['inventory_sha256']}")
        group = groups.setdefault(key, dict(fixture=request['fixture'], mode=request['mode'],
            observation_budget_us=request['observation_budget_us'], inventory_sha256=request['inventory_sha256'],
            attempts=0, outcomes={}, complete_us=[], stages_us={}))
        group['attempts'] += 1
        group['outcomes'][status] = group['outcomes'].get(status, 0) + 1
        if (result and status == 'SUCCESS' and result['cleanup']['worker_stopped'] and verified_cold
                and boot_eligible):
            group['complete_us'].append(result['completion_elapsed_us'])
            # JSON is sorted, so order stage names by the production enum.
            from cura_receiver.persistence_startup import StartupStage
            ordered = [(stage.name, result['stage_entry_offsets_us'][stage.name]) for stage in StartupStage]
            for index, (name, start) in enumerate(ordered):
                end = ordered[index + 1][1] if index + 1 < len(ordered) else result['completion_elapsed_us']
                if start is not None and end is not None:
                    group['stages_us'].setdefault(name, []).append(end - start)
    for group in groups.values():
        values = group.pop('complete_us')
        group['eligible_successes'] = len(values)
        group['median_us'] = statistics.median(values) if values else None
        group['max_us'] = max(values) if values else None
        group['stages_us'] = {name: dict(median=statistics.median(v), max=max(v)) for name, v in group['stages_us'].items()}
    return dict(groups=groups, attempts=records,
        interpretation='Timeouts are censored, incomplete/missing runs are retained. Initial sample counts do not establish p99 or a worst-case bound. Production budget remains a separate decision.')


def verify(root):
    inventory = json.loads((check_root(root) / 'inventory.json').read_text())
    actual = source_manifest()
    changed = [name for name in sorted(actual.keys() | inventory['source_files'].keys())
               if actual.get(name) != inventory['source_files'].get(name)]
    for fixture, info in inventory['fixtures'].items():
        changed += [f'templates/{fixture}/{name}' for name, value in info['files'].items()
                    if digest(root / 'templates' / fixture / name) != value['sha256']]
    if changed:
        raise ValueError('source or template verification failed: ' + ','.join(changed))
    return dict(verified=True, inventory_sha256=digest(root / 'inventory.json'), source_files=len(inventory['source_files']))


def main():
    os.umask(0o077)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    commands = parser.add_subparsers(dest='command', required=True)
    prep = commands.add_parser('prepare')
    prep.add_argument('--scale', type=float, default=1)
    prep.add_argument('--source-commit', required=True)
    armer = commands.add_parser('arm')
    armer.add_argument('--fixture', choices=FIXTURES, default='pilot')
    armer.add_argument('--mode', choices=('smoke', 'warm', 'reboot', 'cold'), required=True)
    armer.add_argument('--label', required=True)
    commands.add_parser('run')
    commands.add_parser('status')
    commands.add_parser('analyze')
    commands.add_parser('verify')
    attest = commands.add_parser('attest-cold')
    attest.add_argument('--attempt', required=True)
    attest.add_argument('--note', required=True)
    export = commands.add_parser('export')
    export.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'prepare':
        if not 0 < args.scale <= 10:
            parser.error('scale must be in (0, 10]')
        prepare(args.root, args.scale, args.source_commit)
    elif args.command == 'arm':
        arm(args.root, args.fixture, args.mode, args.label)
    elif args.command == 'run':
        return run(args.root)
    elif args.command in ('status', 'analyze'):
        report = analyze(args.root)
        report['armed'] = json.loads((args.root / 'armed.json').read_text()) if (args.root / 'armed.json').exists() else None
        print(json.dumps(report, indent=2))
    elif args.command == 'verify':
        print(json.dumps(verify(args.root)))
    elif args.command == 'attest-cold':
        check_root(args.root)
        directory = args.root / 'attempts' / args.attempt
        if directory.parent != args.root / 'attempts' or not args.attempt.replace('-', '').isalnum():
            parser.error('invalid attempt identity')
        request = json.loads((directory / 'request.json').read_text())
        if request['mode'] != 'cold':
            parser.error('physical confirmation is only for cold samples')
        write_json(directory / 'physical-confirmation.json', dict(note=args.note, recorded_at_unix_ns=time.time_ns()))
    else:
        verification = verify(args.root)
        write_json(args.root / f'verification-{time.time_ns()}.json', verification)
        with tarfile.open(args.output, 'x:gz') as archive:
            for path in sorted(args.root.rglob('*.json')):
                if path.name != 'receiver-group.json':
                    archive.add(path, arcname=str(path.relative_to(args.root)), recursive=False)
        print(json.dumps(dict(output=str(args.output), sha256=digest(args.output))))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
