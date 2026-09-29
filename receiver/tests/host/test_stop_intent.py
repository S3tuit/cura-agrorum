"""Stop intent keeps one coherent first deadline, including handler re-entry."""

import inspect
import sys
from types import SimpleNamespace

import pytest

from cura_receiver.stop_intent import StopIntent


def test_stop_is_read_only_until_request_and_repetition_keeps_first_pair():
    values = iter((100, 900))
    stop = StopIntent(SimpleNamespace(now_monotonic_us=lambda: next(values)), 10_000_000)
    assert not stop.is_requested()
    assert stop.requested_at_monotonic_us is None and stop.deadline_monotonic_us is None
    stop.request()
    stop.request()
    assert stop.is_requested()
    assert stop.requested_at_monotonic_us == 100
    assert stop.deadline_monotonic_us == 10_000_100
    assert next(values) == 900  # A repeat performs no clock read or deadline work.


@pytest.mark.parametrize('boundary', ['candidate =', 'retained =', 'self._request = candidate'])
def test_nested_later_request_cannot_extend_first_deadline(boundary):
    values = iter((100, 200))
    stop = StopIntent(SimpleNamespace(now_monotonic_us=lambda: next(values)), 10_000_000)
    source, first = inspect.getsourcelines(StopIntent.request)
    target = first + next(i for i, line in enumerate(source) if boundary in line)
    entered = False
    def trace(frame, event, _arg):
        nonlocal entered
        if frame.f_code is StopIntent.request.__code__ and event == 'line' and frame.f_lineno == target and not entered:
            entered = True
            stop.request()
        return trace
    previous = sys.gettrace()
    try:
        sys.settrace(trace)
        stop.request()
    finally:
        sys.settrace(previous)
    assert entered
    assert (stop.requested_at_monotonic_us, stop.deadline_monotonic_us) == (100, 10_000_100)


def test_nested_request_before_outer_clock_sample_remains_first():
    stop = None
    def sample():
        clock.now_monotonic_us = lambda: 100
        stop.request()
        return 200
    clock = SimpleNamespace(now_monotonic_us=sample)
    stop = StopIntent(clock, 10_000_000)
    stop.request()
    assert (stop.requested_at_monotonic_us, stop.deadline_monotonic_us) == (100, 10_000_100)


def test_overflow_never_leaves_half_recorded_intent():
    stop = StopIntent(SimpleNamespace(now_monotonic_us=lambda: (1 << 64) - 1), 10_000_000)
    with pytest.raises(OverflowError):
        stop.request()
    assert not stop.is_requested() and stop.deadline_monotonic_us is None
