"""Benchmark controls preserve each attempt and keep boot samples distinct."""

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / 'benchmarks/startup_readiness/run.py'
spec = importlib.util.spec_from_file_location('startup_benchmark', SCRIPT)
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


@pytest.fixture
def prepared(tmp_path):
    root = tmp_path / 'benchmark'
    benchmark.prepare(root, 0.0001, 'host-test')
    return root


def run(root, *, code=None):
    command = [sys.executable, str(SCRIPT), '--root', str(root), 'run']
    if code:
        command = [sys.executable, '-c', code, str(SCRIPT), str(root)]
    return subprocess.run(command, capture_output=True, text=True, timeout=15)


@pytest.mark.parametrize('fixture', benchmark.FIXTURES)
def test_real_worker_attempt_and_repeat_protection(prepared, fixture):
    attempt = benchmark.arm(prepared, fixture, 'smoke', 'host-test')
    with pytest.raises(ValueError, match='already armed'):
        benchmark.arm(prepared, fixture, 'smoke', 'overlap')
    result = run(prepared)
    assert result.returncode == 0, result.stderr
    evidence = json.loads((prepared / 'results' / f'{attempt}.json').read_text())
    assert evidence['outcome'] == 'SUCCESS' and evidence['cleanup']['worker_stopped']
    assert evidence['state_load_status'] == 'LOADED'
    assert evidence['observation_budget_us'] == 60_000_000
    assert evidence['startup_deadline_monotonic_us'] - evidence['startup_begin_monotonic_us'] == 60_000_000
    assert evidence['post_cleanup_startup']['terminal_outcome'] == 'SUCCESS'
    assert evidence['post_cleanup_startup']['published_at_monotonic_us'] == evidence['published_at_monotonic_us']
    assert not evidence['post_cleanup_startup']['published_at_or_after_deadline']
    assert len(evidence['stage_entry_offsets_us']) == 5
    assert not (prepared / 'armed.json').exists()
    assert run(prepared).returncode == 0
    assert len(list((prepared / 'results').glob('*.json'))) == 1
    assert benchmark.verify(prepared)['verified']
    assert benchmark.analyze(prepared)['attempts'][0]['physical_confirmation'] is None


@pytest.mark.parametrize('mode', ['reboot', 'cold'])
def test_boot_request_cannot_run_in_arming_boot(prepared, mode):
    attempt = benchmark.arm(prepared, 'pilot', mode, 'next-boot')
    assert run(prepared).returncode == 0
    assert (prepared / 'armed.json').exists()
    assert not list((prepared / 'attempts' / attempt).glob('started.*.json'))
    assert benchmark.analyze(prepared)['attempts'][0]['outcome'] == 'NOT_STARTED'


@pytest.mark.parametrize('late_failure', [False, True])
def test_timeout_retained_separately_from_success_and_late_worker(prepared, late_failure):
    attempt = benchmark.arm(prepared, 'pilot', 'smoke', 'held-worker')
    armed = prepared / 'armed.json'
    request = json.loads(armed.read_text())
    request['observation_budget_us'] = 10_000
    armed.write_text(json.dumps(request))
    (prepared / 'attempts' / attempt / 'request.json').write_text(json.dumps(request))
    code = f'''import runpy,sys,time,errno
module=runpy.run_path(sys.argv[1])
from cura_receiver import persistence_worker as workers
from cura_receiver.persistence_worker import PersistenceWorker
def fail(*args,**kwargs):
    raise OSError(errno.EIO, 'injected late failure')
if {late_failure!r}:
    workers.PersistenceControlOperations=fail
original=PersistenceWorker._initialize
def held(self):
    time.sleep(0.2)
    original(self)
PersistenceWorker._initialize=held
raise SystemExit(module['run'](module['Path'](sys.argv[2])))
'''
    result = run(prepared, code=code)
    assert result.returncode == 1, result.stderr
    evidence = json.loads((prepared / 'results' / f'{attempt}.json').read_text())
    assert evidence['outcome'] == 'PERSISTENCE_STARTUP_INCOMPLETE'
    assert evidence['completion_elapsed_us'] is None
    assert evidence['last_stage'] is None
    assert evidence['cleanup']['worker_stopped']
    later = evidence['post_cleanup_startup']
    assert later['terminal_outcome'] == ('UNAVAILABLE_IO' if late_failure else 'SUCCESS')
    assert later['published_at_or_after_deadline'] and later['published_after_initial_observation']
    assert later['completion_elapsed_us'] >= 200_000
    initial = json.loads((prepared / 'attempts' / attempt / 'startup.json').read_text())
    assert initial['completion_elapsed_us'] is None and 'post_cleanup_startup' not in initial
    assert next(iter(benchmark.analyze(prepared)['groups'].values()))['eligible_successes'] == 0


def test_unavailable_post_cleanup_snapshot_does_not_wait_or_change_decision():
    from cura_receiver.persistence_startup import StartupDecision
    from tests.support.builders.persistence import INSTANCE
    from tests.support.fakes.os_clock import FakeOsClock
    class Worker:
        def startup_evidence(self, *, deadline_monotonic_us, nonblocking):
            assert nonblocking and deadline_monotonic_us == 10
            return None
    decision = StartupDecision('PERSISTENCE_STARTUP_INCOMPLETE', 0, 10, 10, None)
    value = benchmark.post_cleanup_startup(Worker(), decision, INSTANCE, FakeOsClock(monotonic_us=20))
    assert not value['snapshot_available'] and value['terminal_outcome'] is None
    assert value['observed_at_monotonic_us'] == 20
    assert decision.outcome == 'PERSISTENCE_STARTUP_INCOMPLETE'


def test_analysis_keeps_old_cap_and_source_cohorts_separate(prepared):
    for number, (budget, inventory) in enumerate([(10_000_000, 'a' * 64), (60_000_000, 'a' * 64), (60_000_000, 'b' * 64)]):
        attempt = benchmark.arm(prepared, 'pilot', 'warm', f'cohort-{number}')
        armed = prepared / 'armed.json'
        request = json.loads(armed.read_text())
        request.update(observation_budget_us=budget, inventory_sha256=inventory)
        armed.write_text(json.dumps(request))
        (prepared / 'attempts' / attempt / 'request.json').write_text(json.dumps(request))
        assert run(prepared).returncode == 0
    report = benchmark.analyze(prepared)
    assert len(report['groups']) == 3
    assert {group['observation_budget_us'] for group in report['groups'].values()} == {10_000_000, 60_000_000}
    assert all(group['attempts'] == group['eligible_successes'] == 1 for group in report['groups'].values())


def test_corrupt_database_and_missing_completion_are_kept(prepared):
    attempt = benchmark.arm(prepared, 'pilot', 'smoke', 'corrupt')
    (prepared / 'attempts' / attempt / 'receiver.sqlite3').write_bytes(b'corrupt')
    assert run(prepared).returncode == 1
    evidence = json.loads((prepared / 'results' / f'{attempt}.json').read_text())
    assert evidence['outcome'] == 'UNAVAILABLE_CORRUPT'
    missing = benchmark.arm(prepared, 'pilot', 'warm', 'missing')
    os.rename(prepared / 'armed.json', prepared / 'attempts' / missing / f'started.{benchmark.boot_id()}.json')
    assert [row['outcome'] for row in benchmark.analyze(prepared)['attempts']] == ['UNAVAILABLE_CORRUPT', 'INCOMPLETE']


def test_changed_template_is_rejected_and_existing_evidence_is_not_overwritten(prepared):
    (prepared / 'templates/pilot/receiver.sqlite3').write_bytes(b'changed')
    with pytest.raises(ValueError, match='template changed'):
        benchmark.arm(prepared, 'pilot', 'warm', 'bad-template')
    with pytest.raises(FileExistsError):
        benchmark.write_json(prepared / 'inventory.json', {})


def test_one_boot_sample_claim_blocks_a_second_sample(prepared):
    for number in range(2):
        attempt = benchmark.arm(prepared, 'pilot', 'reboot', f'simulated-boot-{number}')
        path = prepared / 'armed.json'
        request = json.loads(path.read_text())
        request['armed_boot_id'] = 'previous-simulated-boot'
        path.write_text(json.dumps(request))
        result = run(prepared)
        assert result.returncode == (0 if number == 0 else 1)
    assert not (prepared / 'results' / f'{attempt}.json').exists()


def test_export_copy_uses_owned_regular_files_and_preserves_existing_paths(tmp_path):
    import hashlib
    spec = importlib.util.spec_from_file_location('startup_benchmark_control', SCRIPT.with_name('control.py'))
    control = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(control)
    source, target = tmp_path / 'source', tmp_path / 'target'
    source.write_bytes(b'evidence')
    kwargs = dict(source_uid=os.getuid(), target_uid=os.getuid(), target_gid=os.getgid())
    assert control.copy_evidence(source, target, **kwargs) == hashlib.sha256(b'evidence').hexdigest()
    assert target.read_bytes() == b'evidence' and target.stat().st_mode & 0o777 == 0o600
    with pytest.raises(FileExistsError):
        control.copy_evidence(source, target, **kwargs)
    alias = tmp_path / 'alias'
    alias.symlink_to(source)
    with pytest.raises(OSError):
        control.copy_evidence(alias, tmp_path / 'other', **kwargs)
    with pytest.raises(ValueError, match='owned by the service'):
        control.copy_evidence(source, tmp_path / 'other', **{**kwargs, 'source_uid':os.getuid() + 1})
