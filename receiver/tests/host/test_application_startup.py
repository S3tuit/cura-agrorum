"""Application readiness uses publication time and shares real signal wakeups."""

from dataclasses import replace
import errno
import os
import signal
from threading import Event, Thread

import pytest

from cura_receiver import persistence_worker as workers
from cura_receiver import sqlite_database as database
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_signal_wait import LinuxSignalWait
from cura_receiver.stop_intent import StopIntent
from tests.host.test_application import make_application, start_application


def test_startup_outlives_runtime_control_budget_without_changing_it(tmp_path):
    app, _, _, _ = make_application(tmp_path)
    initialize = app.worker._initialize
    def delayed():
        app.clock.advance_elapsed_us(1_200_000)
        initialize()
    app.worker._initialize = delayed
    try:
        assert start_application(app).ready
        decision = app.startup_decision
        assert decision.deadline_monotonic_us - decision.begin_monotonic_us == 12_000_000
        assert decision.summary(app.instance.receiver_instance_id)['completion_elapsed_us'] == 1_200_000
        assert app.settings.time_settings.control_budget_us == 1_000_000
    finally:
        app.radio.shutdown()
        app.worker.finish_test()


@pytest.mark.parametrize('offset', [-1, 0, 1])
@pytest.mark.parametrize('reject_configuration', [False, True])
def test_application_uses_terminal_publication_despite_late_observation(tmp_path, offset, reject_configuration):
    app, io, _, config = make_application(tmp_path)
    app.settings = replace(app.settings, persistence_startup_budget_us=100)
    begin = app.clock.now_monotonic_us()
    if reject_configuration:
        config.chmod(0o644)
    publish = app.worker._publish_startup
    def at_boundary(*args, **kwargs):
        if not app.worker.startup_completed():
            app.clock.advance_elapsed_us(begin + 100 + offset - app.clock.now_monotonic_us())
        publish(*args, **kwargs)
    app.worker._publish_startup = at_boundary
    class DelayedObserver:
        def wait_until_monotonic_us(self, deadline, **kwargs):
            assert deadline == begin + 100
            assert app.worker._startup_completed.wait(3)
            app.clock.advance_elapsed_us(begin + 200 - app.clock.now_monotonic_us())
    try:
        result = app.start(wait=DelayedObserver())
        if offset >= 0:
            assert result.failure == 'PERSISTENCE_STARTUP_INCOMPLETE'
        elif reject_configuration:
            assert result.failure == 'CONFIGURATION_REJECTED'
        else:
            assert result.ready
        if not result.ready:
            assert not io.calls and app.runtime is None
        summary = app.startup_decision.summary(app.instance.receiver_instance_id)
        assert summary['completion_elapsed_us'] == 100 + offset
        assert summary['worker_failure'] == ('CONFIGURATION_REJECTED' if reject_configuration else None)
    finally:
        if app._radio_started:
            app.radio.shutdown()
        app.worker.finish_test()


def test_stop_before_start_never_launches_worker_or_radio(tmp_path):
    app, io, _, _ = make_application(tmp_path)
    app.request_stop()
    deadline = app.stop_deadline
    try:
        result = start_application(app)
        assert result.failure == 'STOP_REQUESTED'
        assert app.worker.ident is None and not io.calls
        assert app.shutdown().worker_stopped
        assert app.stop_deadline == deadline
    finally:
        app.worker.finish_test()


def test_deadline_overflow_fails_before_worker_launch(tmp_path):
    app, io, _, _ = make_application(tmp_path)
    app.settings = replace(app.settings, persistence_startup_budget_us=(1 << 64) - 1)
    try:
        with pytest.raises(OverflowError):
            start_application(app)
        assert app.worker.ident is None and not io.calls
    finally:
        app.worker.finish_test()


@pytest.mark.parametrize('signum', [signal.SIGTERM, signal.SIGINT])
def test_application_signal_interrupts_startup_and_bounds_cleanup(tmp_path, signum):
    clock = LinuxOsClock()
    app, io, _, _ = make_application(tmp_path, clock=clock, radio_wait=clock)
    app.stop_intent = StopIntent(clock, 100_000)
    entered, release = Event(), Event()
    initialize = app.worker._initialize
    def held():
        entered.set()
        assert release.wait(3)
        initialize()
    app.worker._initialize = held
    def interrupt():
        assert entered.wait(3)
        os.kill(os.getpid(), signum)
    sender = Thread(target=interrupt)
    try:
        with LinuxSignalWait(clock, app.stop_intent) as wait:
            sender.start()
            result = app.start(wait=wait)
            assert result.failure == 'STOP_REQUESTED'
            assert not io.calls and app.worker.is_alive()
            deadline = app.stop_deadline
            assert clock.now_monotonic_us() < deadline
            stopped = app.shutdown()
            assert not stopped.worker_stopped and not stopped.clean_stop_confirmed
            assert app.stop_deadline == deadline
            release.set()
            app.worker.join(3)
            assert app.start_result is result and result.failure == 'STOP_REQUESTED'
            assert not io.calls
    finally:
        release.set()
        sender.join(3)
        app.worker.finish_test()


def test_application_sees_known_failure_before_blocked_close(tmp_path, monkeypatch):
    clock = LinuxOsClock()
    app, io, _, _ = make_application(tmp_path, clock=clock, radio_wait=clock)
    app.stop_intent = StopIntent(clock, 100_000)
    entered, release = Event(), Event()
    def fail(*args, **kwargs):
        raise OSError(errno.EIO, 'not for service evidence')
    monkeypatch.setattr(workers, 'PersistenceControlOperations', fail)
    close = database.ReceiverDatabase.close
    def held_close(self):
        entered.set()
        assert release.wait(3)
        close(self)
    monkeypatch.setattr(database.ReceiverDatabase, 'close', held_close)
    try:
        result = start_application(app)
        assert result.failure == 'UNAVAILABLE_IO'
        assert entered.wait(3) and app.worker.is_alive()
        assert app.startup_decision.summary(app.instance.receiver_instance_id)['os_errno'] == errno.EIO
        assert not io.calls
        assert not app.shutdown().worker_stopped
    finally:
        release.set()
        app.worker.finish_test()
