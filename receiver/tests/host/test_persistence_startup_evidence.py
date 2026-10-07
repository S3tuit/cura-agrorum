"""Real worker/SQLite boundaries: evidence precedes cleanup, never diagnoses a hang."""

import errno
import os
import signal
import socket
import sqlite3
from threading import Event, Thread

import pytest

from cura_receiver import persistence_worker as workers
from cura_receiver import sqlite_database as database
from cura_receiver.persistence_startup import StartupStage, observe_persistence_startup
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_signal_wait import CompletionNotification, LinuxSignalWait
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.service_evidence import emit_service_evidence
from cura_receiver.stop_intent import StopIntent
from tests.support.builders.persistence import INSTANCE
from tests.support.fakes.os_clock import FakeOsClock


def worker_for(paths, **kwargs):
    path, config, boot = paths
    return workers.PersistenceWorker(instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=path, configuration_path=config, boot_id_path=boot, **kwargs)


def finish(worker):
    worker.request_stop(deadline_monotonic_us=0)
    worker.join(3)
    assert not worker.is_alive()


def test_all_stage_entries_keep_zero_and_terminal_publication(worker_files):
    clock = FakeOsClock(monotonic_us=0)
    worker = worker_for(worker_files, clock=clock)
    enter = worker._enter_startup_stage
    def stage(value):
        clock.advance_elapsed_us(int(value) * 10 - clock.now_monotonic_us())
        enter(value)
    worker._enter_startup_stage = stage
    worker.start()
    try:
        assert worker.wait_started(deadline_monotonic_us=1_000_000).state_load is not None
        evidence = worker.startup_evidence(deadline_monotonic_us=1_000_000)
        assert evidence.stage_entries == (0, 10, 20, 30, 40)
        assert evidence.result.published_at_monotonic_us == 40
    finally:
        finish(worker)


@pytest.mark.parametrize('stage', list(StartupStage))
def test_unexpected_stage_failure_publishes_once(worker_files, stage):
    worker = worker_for(worker_files)
    enter = worker._enter_startup_stage
    def injected(value):
        enter(value)
        if value is stage:
            raise RuntimeError('secret must never be retained')
    worker._enter_startup_stage = injected
    worker.start()
    try:
        result = worker.wait_started(deadline_monotonic_us=worker._clock.now_monotonic_us() + 1_000_000)
        assert result.unexpected_failure
        assert 'secret' not in repr(result)
        assert worker.startup_snapshot is result
        evidence = worker.startup_evidence(deadline_monotonic_us=0)
        assert all(value is None for value in evidence.stage_entries[int(stage) + 1:])
    finally:
        finish(worker)


@pytest.mark.parametrize('boundary', ['connect_close', 'validation_close', 'instance_rollback', 'component_close', 'state_close'])
def test_classified_failure_is_visible_while_cleanup_is_blocked(worker_files, monkeypatch, boundary):
    entered, release = Event(), Event()
    connect = sqlite3.connect
    def block():
        entered.set()
        assert release.wait(3)
    class Connection(sqlite3.Connection):
        def execute(self, sql, *args, **kwargs):
            if boundary == 'connect_close' and sql == 'PRAGMA synchronous = FULL':
                raise OSError(errno.EIO, 'fixture')
            if boundary == 'instance_rollback':
                if sql == 'COMMIT':
                    raise OSError(errno.EIO, 'fixture')
                if sql == 'ROLLBACK':
                    block()
            return super().execute(sql, *args, **kwargs)
        def close(self):
            if boundary == 'connect_close':
                block()
            return super().close()
    monkeypatch.setattr(sqlite3, 'connect', lambda *a, **k: connect(*a, factory=Connection, **k))
    if boundary == 'validation_close':
        monkeypatch.setattr(database, '_validate_integrity', lambda *_: (_ for _ in ()).throw(OSError(errno.EIO, 'fixture')))
        original_close = Connection.close
        def close(self):
            block()
            return original_close(self)
        monkeypatch.setattr(Connection, 'close', close)
    if boundary in ('component_close', 'state_close'):
        target = 'PersistenceControlOperations' if boundary == 'component_close' else 'classify_communicator_state_rows'
        monkeypatch.setattr(workers, target, lambda *a, **k: (_ for _ in ()).throw(OSError(errno.EIO, 'fixture')))
        close = database.ReceiverDatabase.close
        def held_close(self):
            block()
            close(self)
        monkeypatch.setattr(database.ReceiverDatabase, 'close', held_close)
    worker = worker_for(worker_files)
    worker.start()
    try:
        assert entered.wait(3)
        result = worker.wait_started(deadline_monotonic_us=worker._clock.now_monotonic_us() + 100_000)
        assert result is not None and result.database_failure.os_errno == errno.EIO
        if boundary == 'instance_rollback':
            assert result.instance_start.disposition.name == 'OUTCOME_UNKNOWN'
        release.set()
        worker.join(0.01)
        assert worker.startup_snapshot is result
    finally:
        release.set()
        finish(worker)


@pytest.mark.parametrize('offset,expected', [(-1, 'SUCCESS'), (0, 'PERSISTENCE_STARTUP_INCOMPLETE'), (1, 'PERSISTENCE_STARTUP_INCOMPLETE')])
def test_publication_boundary_survives_delayed_observer(worker_files, offset, expected):
    clock = FakeOsClock(monotonic_us=0)
    with CompletionNotification() as notification:
        worker = worker_for(worker_files, clock=clock, startup_notification=notification)
        publish = worker._publish_startup
        def published(*a, **k):
            clock.advance_elapsed_us(100 + offset - clock.now_monotonic_us())
            publish(*a, **k)
        worker._publish_startup = published
        class DelayedWait:
            def wait_until_monotonic_us(self, *_args, **_kwargs):
                assert worker._startup_completed.wait(3)
                clock.advance_elapsed_us(200 - clock.now_monotonic_us())
        try:
            result = observe_persistence_startup(worker=worker, clock=clock,
                stop_intent=StopIntent(clock, 1000), wait=DelayedWait(), budget_us=100)
            assert result.outcome == expected
            assert result.summary(INSTANCE)['completion_elapsed_us'] == 100 + offset
        finally:
            finish(worker)


@pytest.mark.parametrize('failing,expected', [(False, 'SUCCESS'), (True, 'UNEXPECTED_INITIALIZATION_ERROR')])
def test_timely_publication_survives_scheduler_lock_contention(worker_files, monkeypatch, failing, expected):
    clock = FakeOsClock(monotonic_us=0)
    if failing:
        monkeypatch.setattr(workers, 'PersistenceControlOperations',
                            lambda *a, **k: (_ for _ in ()).throw(RuntimeError('fixture')))
    with CompletionNotification() as notification:
        worker = worker_for(worker_files, clock=clock, startup_notification=notification)
        publish = worker._publish_startup
        def published(*a, **k):
            clock.advance_elapsed_us(99 - clock.now_monotonic_us())
            publish(*a, **k)
        worker._publish_startup = published
        holding, release = Event(), Event()
        def run_loop_holds_lock():
            with worker._scheduler_lock:
                holding.set()
                assert release.wait(3)
        holder = Thread(target=run_loop_holds_lock)
        class LateWait:
            def wait_until_monotonic_us(self, *_args, **_kwargs):
                assert worker._startup_completed.wait(3)
                holder.start()
                assert holding.wait(3)
                clock.advance_elapsed_us(104 - clock.now_monotonic_us())
        try:
            result = observe_persistence_startup(worker=worker, clock=clock,
                stop_intent=StopIntent(clock, 1000), wait=LateWait(), budget_us=100)
            assert result.outcome == expected
            assert result.snapshot.result.published_at_monotonic_us == 99
            assert result.observed_at_monotonic_us == 104
        finally:
            release.set()
            holder.join(3)
            finish(worker)


@pytest.mark.parametrize('signum', [signal.SIGTERM, signal.SIGINT])
def test_real_signal_ends_startup_wait_before_worker_cleanup(worker_files, signum):
    clock = LinuxOsClock()
    stop = StopIntent(clock, 100_000)
    entered, release = Event(), Event()
    with CompletionNotification() as notification:
        worker = worker_for(worker_files, clock=clock, startup_notification=notification)
        initialize = worker._initialize
        def held():
            entered.set()
            assert release.wait(3)
            initialize()
        worker._initialize = held
        def signal_stop():
            assert entered.wait(3)
            os.kill(os.getpid(), signum)
        sender = Thread(target=signal_stop)
        try:
            with LinuxSignalWait(clock, stop) as wait:
                sender.start()
                result = observe_persistence_startup(worker=worker, clock=clock,
                    stop_intent=stop, wait=wait, budget_us=2_000_000)
                assert result.outcome == 'STOP_REQUESTED'
                assert not release.is_set() and worker.is_alive()
                assert clock.now_monotonic_us() < stop.deadline_monotonic_us
        finally:
            release.set()
            sender.join(3)
            finish(worker)


def test_completion_after_notification_close_cannot_write_reused_fd():
    notification = CompletionNotification()
    notification.close()
    read_fd, write_fd = os.pipe2(os.O_NONBLOCK)
    try:
        notification.notify()
        with pytest.raises(BlockingIOError):
            os.read(read_fd, 1)
    finally:
        os.close(read_fd)
        os.close(write_fd)


def test_summary_delivery_and_saturated_broken_sinks_are_bounded():
    sender, reader = socket.socketpair()
    with sender, reader:
        assert emit_service_evidence({'outcome': 'SUCCESS'}, fd=sender.fileno()) == 'SENT'
        assert reader.recv(4096) == b'{"outcome":"SUCCESS"}\n'
        while True:
            try:
                sender.send(b'x' * 8192, socket.MSG_DONTWAIT)
            except BlockingIOError:
                break
        assert emit_service_evidence({'outcome': 'TIMEOUT'}, fd=sender.fileno()) == 'BACKPRESSURE'
        assert os.get_blocking(sender.fileno())
        reader.close()
        assert emit_service_evidence({'outcome': 'TIMEOUT'}, fd=sender.fileno()) == 'UNAVAILABLE'


def test_stop_before_launch_and_stop_concurrent_with_completion(worker_files):
    clock = FakeOsClock(monotonic_us=0)
    stop = StopIntent(clock, 1000)
    with CompletionNotification() as notification:
        worker = worker_for(worker_files, clock=clock, startup_notification=notification)
        stop.request()
        result = observe_persistence_startup(worker=worker, clock=clock,
            stop_intent=stop, wait=None, budget_us=100)
        assert result.outcome == 'STOP_REQUESTED'
        assert worker.ident is None
        stop = StopIntent(clock, 1000)
        class StoppingWait:
            def wait_until_monotonic_us(self, *args, **kwargs):
                assert worker._startup_completed.wait(3)
                stop.request()
        try:
            result = observe_persistence_startup(worker=worker, clock=clock,
                stop_intent=stop, wait=StoppingWait(), budget_us=100)
            assert result.snapshot.result.state_load is not None
            assert result.outcome == 'STOP_REQUESTED'
        finally:
            finish(worker)


def test_late_failure_does_not_replace_timeout(worker_files, monkeypatch):
    clock = FakeOsClock(monotonic_us=0)
    def fail(*args, **kwargs):
        clock.advance_elapsed_us(100)
        raise RuntimeError('fixture')
    monkeypatch.setattr(workers, 'PersistenceControlOperations', fail)
    with CompletionNotification() as notification:
        worker = worker_for(worker_files, clock=clock, startup_notification=notification)
        class DelayedWait:
            def wait_until_monotonic_us(self, *args, **kwargs):
                assert worker._startup_completed.wait(3)
        try:
            result = observe_persistence_startup(worker=worker, clock=clock,
                stop_intent=StopIntent(clock, 1000), wait=DelayedWait(), budget_us=100)
            assert result.outcome == 'PERSISTENCE_STARTUP_INCOMPLETE'
            assert result.summary(INSTANCE)['worker_failure'] == 'UNEXPECTED_INITIALIZATION_ERROR'
        finally:
            finish(worker)


def test_unavailable_snapshot_is_bounded_and_explicit(worker_files):
    worker = worker_for(worker_files, clock=FakeOsClock(monotonic_us=0))
    with worker._scheduler_lock:
        assert worker.startup_evidence(deadline_monotonic_us=0) is None
        assert worker.startup_evidence(deadline_monotonic_us=100, nonblocking=True) is None


def test_full_completion_pipe_and_spurious_progress_preserve_deadline(monkeypatch):
    from cura_receiver.platform import linux_signal_wait
    clock = FakeOsClock(monotonic_us=0)
    with CompletionNotification() as notification, LinuxSignalWait(clock, StopIntent(clock, 1000)) as wait:
        while True:
            try:
                os.write(notification._write_fd, b'x' * 4096)
            except BlockingIOError:
                break
        notification.notify()  # Saturation is already a wake hint, never a blocking write.
        waits = []
        def poll(readers, _writes, _errors, timeout):
            waits.append(round(timeout * 1_000_000))
            clock.advance_elapsed_us(25)
            notification.notify()  # Notification cannot itself establish completion.
            return [notification.fileno()], [], []
        monkeypatch.setattr(linux_signal_wait.select, 'select', poll)
        wait.wait_until_monotonic_us(100, completion=notification, completed=lambda: False)
        assert waits == [100, 75, 50, 25]


def test_partial_summary_is_not_retried_or_buffered(monkeypatch):
    from cura_receiver import service_evidence
    sender, reader = socket.socketpair()
    calls = []
    class PartialSocket:
        def __init__(self, *, fileno):
            self.fd = fileno
        def __enter__(self):
            return self
        def __exit__(self, *args):
            os.close(self.fd)
        def send(self, data, flags):
            calls.append((data, flags))
            return 1
    try:
        monkeypatch.setattr(service_evidence.socket, 'socket', PartialSocket)
        assert emit_service_evidence({'outcome': 'SUCCESS'}, fd=sender.fileno()) == 'PARTIAL'
        assert len(calls) == 1
        assert calls[0][1] & socket.MSG_DONTWAIT
    finally:
        sender.close()
        reader.close()
