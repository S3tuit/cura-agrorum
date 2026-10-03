"""Kernel exclusion and guarded startup across real competing processes."""

from contextlib import contextmanager
import errno
import multiprocessing
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
from unittest.mock import Mock

import pytest

from cura_receiver.platform import linux_process_ownership as ownership
from cura_receiver.generated.receiver_entities_generated import communicator_state_v2_parameters
from tests.support.builders.persistence_control import state
from tests.support.coordination.persistence_worker import prepare_worker_files
from tests.support.coordination.receiver_process import run_receiver


@contextmanager
def receiver_process(root, boundary, **kwargs):
    context = multiprocessing.get_context('spawn')
    parent, child = context.Pipe()
    process = context.Process(target=run_receiver, args=(child, str(root), boundary), kwargs=kwargs)
    process.start()
    child.close()
    try:
        assert parent.poll(10), f'receiver did not reach {boundary}'
        evidence = parent.recv()
        assert evidence.get('boundary') == boundary, evidence
        yield process, parent, evidence
    finally:
        if process.is_alive():
            process.kill()
        process.join(5)
        assert not process.is_alive()
        parent.close()


def environment(root):
    repo = Path(__file__).resolve().parents[3]
    return {
        **os.environ,
        'PYTHONPATH': os.pathsep.join((str(repo / 'receiver'), str(repo / 'protocol/protocol-v2-lora/python'))),
        'CURA_RECEIVER_CONFIGURATION': str(root / 'test-group.json'),
        'CURA_RECEIVER_DATABASE': str(root / 'worker.db'),
        'SQLITE_TMPDIR': str(root / 'sqlite-temp'),
        'CURA_RECEIVER_TEST_ROOT': str(root),
    }


def reject_contender(root):
    result = subprocess.run(
        [sys.executable, '-m', 'cura_receiver', '--rtc-helper-sha256', '0' * 64,
         '--rtc-kernel-bound-us', '3000000'],
        env=environment(root), capture_output=True, text=True, timeout=10)
    assert result.returncode == 1, result.stderr
    assert result.stdout == 'receiver startup: RECEIVER_ALREADY_RUNNING\n'
    assert result.stderr == ''


def durable_rows(path):
    with sqlite3.connect(path) as db:
        return tuple(db.execute(f'SELECT * FROM {table}').fetchall() for table in (
            'receiver_instances', 'communicator_state', 'airtime_commissioning'))


@pytest.mark.parametrize('boundary', [
    'before_runtime', 'after_start', 'running', 'shutdown', 'returned',
    'construction_failed', 'returned_with_worker',
])
def test_guard_covers_entire_entrypoint_lifetime_and_all_database_paths(tmp_path, boundary):
    root = tmp_path / 'owner'
    root.mkdir()
    database, _, _ = prepare_worker_files(root, known_empty_airtime=True)
    alias = tmp_path / 'alias'
    alias.symlink_to(root, target_is_directory=True)
    different = tmp_path / 'different'
    with receiver_process(root, boundary) as (process, pipe, evidence):
        before = durable_rows(database)
        if boundary in ('before_runtime', 'construction_failed'):
            assert before[0] == [] and before[1] == []
        else:
            assert len(before[0]) == 1
        for contender_root in (root, alias, different):
            reject_contender(contender_root)
        assert durable_rows(database) == before
        assert not different.exists()
        pipe.send('continue')
        process.join(10)
        assert process.exitcode == 0
    # Actual process exit releases the guard even after failed construction or
    # a returned shutdown with a daemon worker still blocked in SQLite.
    with receiver_process(root, 'before_runtime'):
        pass


@pytest.mark.parametrize('history', ['commissioning', 'persisted'])
def test_crash_replacement_loads_fresh_state_and_accounts_for_unsaved_airtime(tmp_path, history):
    database, _, _ = prepare_worker_files(tmp_path, known_empty_airtime=history == 'commissioning')
    trusted = history == 'persisted'
    if trusted:
        with sqlite3.connect(database) as db:
            db.execute('INSERT INTO communicator_state VALUES (?,?,?,?,?)',
                       communicator_state_v2_parameters(state()))
    with receiver_process(tmp_path, 'running', trusted=trusted, spend=True) as (process, _, spent):
        assert spent['generation'] == (2 if trusted else 1)
        assert spent['initial_used'] == (2_000_000 if trusted else 0)
        assert spent['unsaved'] == 1_900_248
        before = durable_rows(database)
        reject_contender(tmp_path)
        assert durable_rows(database) == before
        process.kill()
        process.join(5)
        assert process.exitcode == -9
    with receiver_process(tmp_path, 'running', trusted=trusted) as (_, _, recovered):
        assert recovered['generation'] == (3 if trusted else 2)
        assert recovered['initial_used'] == (4_000_000 if trusted else 36_000_000)
        assert recovered['unsaved'] == 0
        assert recovered['available'] == (2_000_000 if trusted else 0)
        rows = durable_rows(database)
        assert len(rows[0]) == 2 and rows[2] == []


def race_claim(pipe, gate):
    gate.wait()
    try:
        ownership.claim_receiver_process()
    except ownership.ReceiverAlreadyRunning:
        pipe.send('busy')
    else:
        assert not os.get_inheritable(ownership._ownership_fd)
        # A second call cannot allocate/release a second process-lifetime claim.
        with pytest.raises(ownership.ReceiverAlreadyRunning):
            ownership.claim_receiver_process()
        pipe.send('owner')
    assert pipe.recv() == 'exit'
    pipe.close()


def test_simultaneous_claims_have_exactly_one_owner():
    context = multiprocessing.get_context('spawn')
    gate = context.Event()
    peers = []
    try:
        for _ in range(2):
            parent, child = context.Pipe()
            process = context.Process(target=race_claim, args=(child, gate))
            process.start()
            child.close()
            peers.append((process, parent))
        gate.set()
        results = []
        for _, pipe in peers:
            assert pipe.poll(10)
            results.append(pipe.recv())
        assert sorted(results) == ['busy', 'owner']
        for process, pipe in peers:
            pipe.send('exit')
            process.join(5)
            assert process.exitcode == 0
    finally:
        for process, pipe in peers:
            if process.is_alive():
                process.kill()
            process.join(5)
            pipe.close()


def test_exec_does_not_inherit_ownership_even_without_close_fds(tmp_path):
    # The helper attempts to claim the real address while its parent is alive,
    # then retries after the parent's SIGKILL. Inherited ownership would keep
    # the name occupied forever despite the parent being gone.
    helper_source = '''
import sys
from cura_receiver.platform.linux_process_ownership import claim_receiver_process, ReceiverAlreadyRunning
try:
    claim_receiver_process()
except ReceiverAlreadyRunning:
    print('busy', flush=True)
else:
    raise AssertionError('parent must still own the guard')
assert sys.stdin.readline().strip() == 'retry'
claim_receiver_process()
print('acquired', flush=True)
'''
    owner_source = '''
import subprocess, sys
from cura_receiver.platform.linux_process_ownership import claim_receiver_process
claim_receiver_process()
helper = subprocess.Popen([sys.executable, '-c', sys.argv[1]], close_fds=False)
helper.wait()
'''
    process = subprocess.Popen([sys.executable, '-c', owner_source, helper_source],
        env=environment(tmp_path), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, text=True)
    try:
        import select
        assert select.select([process.stdout], [], [], 10)[0]
        assert process.stdout.readline() == 'busy\n'
        process.kill()
        process.wait(timeout=5)
        process.stdin.write('retry\n')
        process.stdin.flush()
        assert select.select([process.stdout], [], [], 10)[0]
        assert process.stdout.readline() == 'acquired\n'
        assert process.stderr.read() == ''
    finally:
        if process.poll() is None:
            process.kill()
        process.wait(timeout=5)
        process.stdin.close()
        process.stdout.close()
        process.stderr.close()


@pytest.mark.parametrize('code', [errno.EADDRINUSE, errno.EACCES, errno.ENOMEM])
def test_failed_bind_closes_temporary_socket_without_retaining_ownership(monkeypatch, code):
    guard = Mock()
    guard.bind.side_effect = OSError(code, 'injected')
    monkeypatch.setattr(ownership.socket, 'socket', Mock(return_value=guard))
    error_type = ownership.ReceiverAlreadyRunning if code == errno.EADDRINUSE else OSError
    with pytest.raises(error_type):
        ownership.claim_receiver_process()
    guard.close.assert_called_once_with()
    guard.detach.assert_not_called()
    assert ownership._ownership_fd is None


def test_socket_creation_failure_does_not_claim_ownership(monkeypatch):
    monkeypatch.setattr(ownership.socket, 'socket', Mock(side_effect=OSError(errno.EMFILE, 'limit')))
    with pytest.raises(OSError) as error:
        ownership.claim_receiver_process()
    assert error.value.errno == errno.EMFILE
    assert ownership._ownership_fd is None
