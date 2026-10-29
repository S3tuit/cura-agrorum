"""Notification hints, absolute waits and process-global signal ownership."""

import os
import signal
from threading import Thread
from types import SimpleNamespace

import pytest

from cura_receiver.platform import linux_signal_wait as module
from cura_receiver.platform.linux_signal_wait import LinuxSignalWait
from cura_receiver.stop_intent import StopIntent


@pytest.fixture
def previous_signals():
    read_fd, write_fd = os.pipe2(os.O_NONBLOCK | os.O_CLOEXEC)
    handlers = {number: signal.getsignal(number) for number in (signal.SIGTERM, signal.SIGINT)}
    previous_fd = signal.set_wakeup_fd(write_fd)
    seen = []
    def previous_handler(number, _frame):
        seen.append(number)
    for number in handlers:
        signal.signal(number, previous_handler)
    try:
        yield read_fd, write_fd, previous_handler, seen
    finally:
        for number, handler in handlers.items():
            signal.signal(number, handler)
        signal.set_wakeup_fd(previous_fd)
        os.close(read_fd)
        os.close(write_fd)


def make_wait():
    clock = SimpleNamespace(now=100)
    clock.now_monotonic_us = lambda: clock.now
    stop = StopIntent(clock, 10_000_000)
    return LinuxSignalWait(clock, stop), clock, stop


@pytest.mark.parametrize('exception', [False, True])
def test_restore_previous_handlers_and_wakeup_before_descriptor_reuse(previous_signals, exception):
    read_fd, write_fd, previous_handler, seen = previous_signals
    wait, _, stop = make_wait()
    fds = ()
    try:
        with wait:
            fds = (wait._read_fd, wait._write_fd)
            if exception:
                raise RuntimeError('owner failure')
    except RuntimeError as error:
        assert exception and str(error) == 'owner failure'
    wait.close()
    for fd in fds:
        with pytest.raises(OSError):
            os.fstat(fd)
    assert signal.getsignal(signal.SIGTERM) is previous_handler
    assert signal.getsignal(signal.SIGINT) is previous_handler
    probe = os.open('/dev/null', os.O_WRONLY)  # Reuse a closed adapter descriptor.
    try:
        os.kill(os.getpid(), signal.SIGTERM)
        assert os.read(read_fd, 1) == bytes((signal.SIGTERM,))
        assert seen == [signal.SIGTERM] and not stop.is_requested()
        assert signal.set_wakeup_fd(write_fd) == write_fd
    finally:
        os.close(probe)


@pytest.mark.parametrize('fault', ['pipe', 'wakeup', 'term', 'int'])
def test_partial_installation_restores_previous_state_and_closes_owned_fds(monkeypatch, previous_signals, fault):
    read_fd, write_fd, previous_handler, seen = previous_signals
    wait, _, _ = make_wait()
    acquired = []
    pipe, wakeup, install = os.pipe2, signal.set_wakeup_fd, signal.signal
    def make_pipe(flags):
        if fault == 'pipe':
            raise OSError('injected pipe failure')
        result = pipe(flags)
        acquired.extend(result)
        return result
    def set_wakeup(fd, **kwargs):
        if fault == 'wakeup' and fd != write_fd:
            raise OSError('injected wakeup failure')
        return wakeup(fd, **kwargs)
    def set_handler(number, handler):
        if handler == wait._request_stop and ((fault == 'term' and number == signal.SIGTERM)
                                             or (fault == 'int' and number == signal.SIGINT)):
            raise OSError('injected handler failure')
        return install(number, handler)
    with monkeypatch.context() as patch:
        patch.setattr(os, 'pipe2', make_pipe)
        patch.setattr(signal, 'set_wakeup_fd', set_wakeup)
        patch.setattr(signal, 'signal', set_handler)
        with pytest.raises(OSError, match='injected'):
            wait.__enter__()
    for fd in acquired:
        with pytest.raises(OSError):
            os.fstat(fd)
    assert signal.getsignal(signal.SIGTERM) is previous_handler
    assert signal.getsignal(signal.SIGINT) is previous_handler
    os.kill(os.getpid(), signal.SIGINT)
    assert os.read(read_fd, 1) == bytes((signal.SIGINT,))
    assert seen == [signal.SIGINT]
    assert signal.set_wakeup_fd(write_fd) == write_fd


def test_installation_on_foreign_thread_fails_without_leaking_descriptors(monkeypatch):
    wait, _, _ = make_wait()
    pipe = os.pipe2
    acquired, errors = [], []
    def make_pipe(flags):
        result = pipe(flags)
        acquired.extend(result)
        return result
    monkeypatch.setattr(os, 'pipe2', make_pipe)
    def install():
        try:
            with wait:
                pytest.fail('foreign thread installed process signal state')
        except ValueError as error:
            errors.append(error)
    thread = Thread(target=install)
    thread.start()
    thread.join(5)
    assert not thread.is_alive() and len(errors) == 1
    for fd in acquired:
        with pytest.raises(OSError):
            os.fstat(fd)


def test_notifications_and_eintr_recheck_original_absolute_deadline(monkeypatch):
    wait, clock, stop = make_wait()
    timeouts = []
    with wait:
        os.write(wait._write_fd, b'not-authoritative')
        def poll(readers, _writers, _errors, timeout):
            timeouts.append(timeout)
            if len(timeouts) == 1:
                clock.now = 300
                return readers, (), ()
            if len(timeouts) == 2:
                clock.now = 500
                raise InterruptedError()
            clock.now = 1100
            return (), (), ()
        monkeypatch.setattr(module.select, 'select', poll)
        wait.wait_until_monotonic_us(1100)
        assert not stop.is_requested()
        assert timeouts == [0.001, 0.0008, 0.0006]
        with pytest.raises(BlockingIOError):
            os.read(wait._read_fd, 1)


@pytest.mark.parametrize('full', [False, True])
def test_signal_between_predicate_and_poll_is_observed_even_with_full_pipe(monkeypatch, capsys, full):
    wait, _, stop = make_wait()
    poll = module.select.select
    with wait:
        if full:
            while True:
                try:
                    os.write(wait._write_fd, b'x' * 4096)
                except BlockingIOError:
                    break
        def interrupted_poll(*args):
            os.kill(os.getpid(), signal.SIGTERM)
            return poll(*args)
        monkeypatch.setattr(module.select, 'select', interrupted_poll)
        wait.wait_until_monotonic_us(10_000_100)
        assert stop.is_requested() and stop.deadline_monotonic_us == 10_000_100
    assert 'wakeup fd' not in capsys.readouterr().err


def test_stop_before_wait_needs_no_notification_or_poll(monkeypatch):
    wait, _, stop = make_wait()
    stop.request()
    with wait:
        monkeypatch.setattr(module.select, 'select', lambda *_: pytest.fail('polled after stop'))
        wait.wait_until_monotonic_us(10_000_100)
    with pytest.raises(RuntimeError, match='not installed'):
        wait.wait_until_monotonic_us(10_000_100)
