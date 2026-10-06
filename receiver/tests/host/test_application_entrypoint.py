"""CLI inputs must reach the real adapter boundary in its contracted type."""

from types import SimpleNamespace
from unittest.mock import Mock, MagicMock

import pytest

from cura_receiver import __main__ as entry


@pytest.mark.parametrize('digest', ['', '0' * 63, '0' * 65, 'g' * 64,
                                   'A' * 64, '00 ' * 32, '0x' + '0' * 64])
def test_invalid_helper_digest_fails_before_device_construction(monkeypatch, capsys, digest):
    monkeypatch.setattr('sys.argv', ['receiver', '--rtc-helper-sha256', digest,
                                   '--rtc-kernel-bound-us', '3000000'])
    clock = Mock(side_effect=AssertionError('clock constructed before input rejection'))
    monkeypatch.setattr(entry, 'LinuxOsClock', clock)
    with pytest.raises(SystemExit) as result:
        entry.main()
    assert result.value.code == 2
    clock.assert_not_called()
    error = capsys.readouterr().err
    assert 'helper digest must be' in error
    if digest:
        assert digest not in error


def test_valid_cli_passes_exact_digest_bytes_through_successful_composition(monkeypatch):
    digest = bytes(range(32))
    monkeypatch.setattr('sys.argv', ['receiver', '--rtc-helper-sha256', digest.hex(),
                                   '--rtc-kernel-bound-us', '3000000'])
    monkeypatch.setattr(entry.os, 'environ', {'CURA_RECEIVER_STARTUP_BUDGET_US': '12000000'})
    claimed = Mock()
    monkeypatch.setattr(entry, 'claim_receiver_process', claimed)
    clock = SimpleNamespace(now_monotonic_us=lambda: 0)
    def runtime_clock():
        claimed.assert_called_once_with()
        return clock
    monkeypatch.setattr(entry, 'LinuxOsClock', runtime_clock)
    monkeypatch.setattr(entry, 'create_receiver_instance', Mock(return_value=SimpleNamespace(receiver_instance_id=bytes(16))))
    rtc = Mock(return_value=object())
    monkeypatch.setattr(entry, 'LinuxDs3231Control', rtc)
    for name in ('LinuxChronyControl', 'ServicePersistenceWorker', 'LinuxRadioIo',
                 'Sx1262', 'Radio', 'LinuxKernelClock'):
        monkeypatch.setattr(entry, name, Mock(return_value=object()))
    application = Mock()
    application.start_result = SimpleNamespace(ready=True)
    application.start.return_value = application.start_result
    application.startup_decision = Mock()
    application.startup_decision.summary.return_value = {'outcome': 'SUCCESS'}
    application.run.return_value = 0
    application.shutdown.return_value = SimpleNamespace(failure=None)
    monkeypatch.setattr(entry, 'ReceiverApplication', Mock(return_value=application))
    stop = Mock()
    stop.is_requested.return_value = True
    monkeypatch.setattr(entry, 'StopIntent', Mock(return_value=stop))
    waiter = MagicMock()
    monkeypatch.setattr(entry, 'LinuxSignalWait', Mock(return_value=waiter))
    evidence = Mock(return_value='SENT')
    monkeypatch.setattr(entry, 'emit_service_evidence', evidence)
    assert entry.main() == 0
    application.start.assert_called_once_with(wait=waiter.__enter__.return_value)
    record = evidence.call_args.args[0]
    assert record['outcome'] == record['application_outcome'] == 'SUCCESS'
    assert record['event'] == 'receiver_startup'
    rtc.assert_called_once_with(clock, kernel_operation_bound_us=3000000,
                                helper_sha256=digest, receiver_gid=entry.os.getegid())
    application.shutdown.assert_called_once_with(clean_requested=True)
    application.run.assert_called_once_with(wait=waiter.__enter__.return_value)
    entry.LinuxSignalWait.assert_called_once_with(clock, stop)
    assert entry.ReceiverApplication.call_args.kwargs['stop_intent'] is stop
    assert entry.Radio.call_args.kwargs['stop_requested'] == stop.is_requested
    waiter.__exit__.assert_called_once_with(None, None, None)
    assert entry.os.environ['SQLITE_TMPDIR'] == '/var/lib/cura-agrorum/tmp'


@pytest.mark.parametrize('error,message', [
    (entry.ReceiverAlreadyRunning(), 'RECEIVER_ALREADY_RUNNING'),
    (OSError(24, 'descriptor limit'), 'PROCESS_OWNERSHIP_FAILED'),
])
def test_ownership_failure_exits_before_any_runtime_construction(monkeypatch, capsys, error, message):
    monkeypatch.setattr('sys.argv', ['receiver', '--rtc-helper-sha256', '0' * 64,
                                   '--rtc-kernel-bound-us', '3000000'])
    monkeypatch.setattr(entry.os, 'environ', {'CURA_RECEIVER_STARTUP_BUDGET_US': '12000000'})
    monkeypatch.setattr(entry, 'claim_receiver_process', Mock(side_effect=error))
    constructors = []
    for name in ('LinuxOsClock', 'create_receiver_instance', 'LinuxDs3231Control',
                 'LinuxChronyControl', 'ServicePersistenceWorker', 'LinuxRadioIo', 'Radio',
                 'Sx1262', 'LinuxKernelClock', 'ReceiverApplication', 'LinuxSignalWait'):
        constructor = Mock(side_effect=AssertionError('runtime constructed without ownership'))
        monkeypatch.setattr(entry, name, constructor)
        constructors.append(constructor)
    evidence = Mock(return_value='BACKPRESSURE')
    monkeypatch.setattr(entry, 'emit_service_evidence', evidence)
    assert entry.main() == 1
    for constructor in constructors:
        constructor.assert_not_called()
    assert capsys.readouterr().out == ''
    evidence.assert_called_once()
    assert evidence.call_args.args[0]['outcome'] == message
    assert entry.os.environ == {'CURA_RECEIVER_STARTUP_BUDGET_US': '12000000'}


def blocked_output_entrypoint(pipe, output, mode):
    """Child entrypoint with a saturated real service stream and platform fakes."""
    import os
    import sys
    from cura_receiver.service_evidence import emit_service_evidence
    from cura_receiver.receiver_startup import ReceiverInstanceStart

    os.dup2(output.fileno(), 1)
    os.dup2(output.fileno(), 2)
    sys.argv = ['receiver', '--rtc-helper-sha256', '0' * 64,
                '--rtc-kernel-bound-us', '3000000']
    entry.os.environ = {'CURA_RECEIVER_STARTUP_BUDGET_US': '12000000'}
    emitted, notifications, shutdowns = [], [], []
    def record(value):
        emitted.append((dict(value), emit_service_evidence(value)))
        return emitted[-1][1]
    entry.emit_service_evidence = record
    def claim():
        if mode == 'ownership':
            raise entry.ReceiverAlreadyRunning()
    entry.claim_receiver_process = claim
    clock = SimpleNamespace(now_monotonic_us=lambda: 0)
    def create_clock():
        if mode == 'setup':
            raise RuntimeError('secret exception text')
        return clock
    entry.LinuxOsClock = create_clock
    entry.create_receiver_instance = lambda clock: ReceiverInstanceStart(bytes.fromhex('00112233445546778899aabbccddeeff'), 0)
    for name in ('LinuxDs3231Control', 'LinuxChronyControl', 'LinuxKernelClock',
                 'LinuxRadioIo', 'Sx1262', 'Radio'):
        setattr(entry, name, lambda *args, **kwargs: object())
    def worker(**kwargs):
        notifications.append(kwargs['startup_notification'])
        return object()
    entry.ServicePersistenceWorker = worker
    class Application:
        runtime = None
        start_result = None
        startup_decision = None
        def __init__(self, **kwargs):
            pass
        def start(self, *, wait):
            if mode == 'start_exception':
                raise RuntimeError('secret exception text')
            self.startup_decision = SimpleNamespace(summary=lambda instance: {
                'outcome': 'UNAVAILABLE_CORRUPT' if mode == 'start_failure' else 'SUCCESS'})
            self.start_result = SimpleNamespace(ready=mode != 'start_failure',
                                               failure='UNAVAILABLE_CORRUPT')
            return self.start_result
        def run(self, *, wait):
            if mode == 'runtime':
                raise RuntimeError('secret exception text')
            return 0
        def shutdown(self, **kwargs):
            shutdowns.append(True)
            if mode == 'shutdown_exception':
                raise RuntimeError('secret exception text')
            return SimpleNamespace(failure='WORKER_NOT_STOPPED' if mode == 'shutdown' else None,
                                   worker_stopped=mode != 'shutdown', clean_stop_confirmed=False)
    entry.ReceiverApplication = Application
    if mode == 'environment':
        entry.os.environ = {}
    code = entry.main()
    for notification in notifications:
        with pytest.raises(RuntimeError, match='closed'):
            notification.fileno()
        notification.notify()  # A late worker cannot reuse the closed descriptor.
    pipe.send(dict(code=code, emitted=emitted, shutdowns=len(shutdowns)))
    pipe.close()


@pytest.mark.parametrize('mode', ['success', 'environment', 'ownership', 'setup',
                                  'start_failure', 'start_exception', 'runtime',
                                  'shutdown', 'shutdown_exception'])
def test_entrypoint_exits_with_saturated_stdout_and_stderr(mode):
    import multiprocessing
    import socket
    context = multiprocessing.get_context('spawn')
    parent, child = context.Pipe()
    reader, writer = socket.socketpair()
    with reader, writer:
        while True:
            try:
                writer.send(b'x' * 4096, socket.MSG_DONTWAIT)
            except BlockingIOError:
                break
        process = context.Process(target=blocked_output_entrypoint, args=(child, writer, mode))
        process.start()
        child.close()
        try:
            assert parent.poll(10), 'entrypoint blocked on service output or cleanup'
            result = parent.recv()
            process.join(5)
            assert process.exitcode == 0, 'buffered exit output blocked'
            assert result['code'] == (0 if mode == 'success' else 2 if mode == 'environment' else 1)
            startup = [r for r, _ in result['emitted'] if r['event'] == 'receiver_startup']
            assert len(startup) == 1
            assert 'secret' not in repr(result)
            assert all(status == 'BACKPRESSURE' for _, status in result['emitted'])
            assert result['shutdowns'] == (0 if mode in ('environment', 'ownership', 'setup') else 1)
            if mode == 'start_failure':
                assert startup[0]['outcome'] == startup[0]['application_outcome'] == 'UNAVAILABLE_CORRUPT'
        finally:
            if process.is_alive():
                process.kill()
            process.join(5)
            parent.close()


def test_service_worker_cleanup_exception_never_invokes_traceback(tmp_path, monkeypatch):
    import errno
    import threading
    from cura_receiver.platform.linux_clocks import LinuxOsClock
    from cura_receiver.receiver_startup import ReceiverInstanceStart
    from cura_receiver.sqlite_database import ReceiverDatabase
    from tests.support.builders.persistence import INSTANCE
    from tests.support.coordination.persistence_worker import prepare_worker_files

    database, configuration, boot = prepare_worker_files(tmp_path)
    clock = LinuxOsClock()
    worker = entry.ServicePersistenceWorker(instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=database, configuration_path=configuration, boot_id_path=boot, clock=clock)
    close = ReceiverDatabase.close
    def failed_close(self):
        close(self)
        raise OSError(errno.EIO, 'secret cleanup text')
    monkeypatch.setattr(ReceiverDatabase, 'close', failed_close)
    traceback = Mock()
    monkeypatch.setattr(threading, 'excepthook', traceback)
    records = []
    monkeypatch.setattr(entry, 'emit_service_evidence', lambda record: records.append(record) or 'BACKPRESSURE')
    worker.start()
    try:
        startup = worker.wait_started(deadline_monotonic_us=clock.now_monotonic_us() + 3_000_000)
        assert startup is not None and startup.state_load is not None
    finally:
        worker.request_stop(deadline_monotonic_us=0)
        worker.join(3)
    assert not worker.is_alive()
    traceback.assert_not_called()
    assert records == [dict(format_version=1, event='receiver_persistence',
                            outcome='WORKER_FAILED', receiver_instance_id=INSTANCE.hex())]
