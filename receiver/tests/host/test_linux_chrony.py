from dataclasses import replace
import sys

import pytest

from cura_receiver.platform import linux_chrony as L
from cura_receiver.platform import _time_process as P
from cura_receiver.ports.chrony import (
    ChronyQueryStatus as Q,
    ChronyStepDisposition as D,
)
from tests.support.fakes.os_clock import FakeOsClock

CSV = b"B99DE5FE,185.157.229.254,3,1789237487.797281660,-0.000354626,0.000372696,0.000408864,5.867,0.008,0.233,0.024403038,0.009397091,1033.5,Normal\n"


# A literal real 4.6.1 capture fixes source selection, signed correction and conservative conversions.
def test_real_capture():
    result = L.parse_tracking(CSV, 10, 20)
    assert result.status is Q.OK and result.source_selected and result.synchronized
    assert (
        result.remaining_correction_us,
        result.root_distance_us,
        result.estimated_skew_ppb,
    ) == (-355, 21599, 233)
    assert result.evidence().sample_started_at_monotonic_us == 10


# Decimal formation preserves signs and rounds half-delay plus dispersion only after addition.
@pytest.mark.parametrize(
    "offset,delay,dispersion,skew,expected",
    [
        ("0.000000001", "0.000000001", "0.000000001", "0.0001", (1, 1, 1)),
        ("-0.000000001", "0", "0", "0", (-1, 0, 0)),
        ("0", "0.000001001", "0.000000499", "1.0001", (0, 1, 1001)),
    ],
)
def test_fractional_rounding(offset, delay, dispersion, skew, expected):
    fields = CSV.decode().strip().split(",")
    fields[4], fields[10], fields[11], fields[9] = offset, delay, dispersion, skew
    value = L.parse_tracking(",".join(fields).encode(), 0, 0)
    assert (
        value.remaining_correction_us,
        value.root_distance_us,
        value.estimated_skew_ppb,
    ) == expected


# Every numeric field rejects nonfinite or malformed values, even when it is not used by policy.
@pytest.mark.parametrize("index", range(3, 13))
@pytest.mark.parametrize("bad", ["NaN", "Infinity", "x", "1e999", "1\n2"])
def test_malformed_numbers(index, bad):
    fields = CSV.decode().strip().split(",")
    fields[index] = bad
    with pytest.raises(ValueError):
        L.parse_tracking(",".join(fields).encode(), 0, 0)


# Local mode and absent/unselected/leap-unsynchronized sources are ordinary non-trusted inputs.
@pytest.mark.parametrize(
    "index,value",
    [(0, "7F7F0101"), (0, "00000000"), (2, "0"), (2, "16"), (13, "Not synchronised")],
)
def test_unusable_sources(index, value):
    fields = CSV.decode().strip().split(",")
    fields[index] = value
    assert not L.parse_tracking(",".join(fields).encode(), 0, 0).synchronized


# Structurally bad unsigned values, envelopes and overflowing normalized results are invalid.
@pytest.mark.parametrize(
    "index,value",
    [
        (3, "-1"),
        (6, "-1"),
        (9, "-1"),
        (10, "-1"),
        (11, "-1"),
        (12, "-1"),
        (4, "9223372036855"),
        (10, "18446744073709551615"),
        (1, "example.com"),
        (2, "17"),
        (13, "Unknown"),
    ],
)
def test_invalid_tracking_fields(index, value):
    fields = CSV.decode().strip().split(",")
    fields[index] = value
    with pytest.raises(ValueError):
        L.parse_tracking(",".join(fields).encode(), 0, 0)


def configured(monkeypatch):
    clock = FakeOsClock(monotonic_us=10)
    calls = []
    response = [
        L._ChildResult(0, b"chronyc (chrony) version 4.6.1 (+READLINE)\n", started=True)
    ]

    def run(argv, supplied_clock, deadline):
        calls.append(argv)
        assert supplied_clock is clock
        return response[0]

    monkeypatch.setattr(L, "_run_child", run)
    adapter = L.LinuxChronyControl(
        clock, socket_path="/run/chrony/chronyd.sock", deadline_monotonic_us=100
    )
    return adapter, clock, calls, response


# Per-call operations construct only the fixed local-socket argv and preserve result categories.
def test_fixed_commands(monkeypatch):
    adapter, _, calls, response = configured(monkeypatch)
    response[0] = L._ChildResult(0, CSV, started=True)
    assert adapter.read_tracking(deadline_monotonic_us=100).status is Q.OK
    response[0] = L._ChildResult(0, b"200 OK\n", started=True)
    assert (
        adapter.apply_pending_correction_by_step(deadline_monotonic_us=100).disposition
        is D.SUBMITTED
    )
    assert calls == [
        ("/usr/bin/chronyc", "-v"),
        ("/usr/bin/chronyc", "-n", "-c", "-h", "/run/chrony/chronyd.sock", "tracking"),
        ("/usr/bin/chronyc", "-n", "-c", "-h", "/run/chrony/chronyd.sock", "makestep"),
    ]


# A possibly executed command never becomes a definite rejection because a timeout/reply was lost.
@pytest.mark.parametrize(
    "response,expected",
    [
        (L._ChildResult(timed_out=True), D.NOT_SUBMITTED),
        (L._ChildResult(started=True, timed_out=True), D.OUTCOME_UNKNOWN),
        (
            L._ChildResult(0, b"200 OK\n", started=True, timed_out=True),
            D.OUTCOME_UNKNOWN,
        ),
        (L._ChildResult(1, b"501 Not authorised\n", started=True), D.NOT_SUBMITTED),
        (
            L._ChildResult(1, b"506 Cannot talk to daemon\n", started=True),
            D.OUTCOME_UNKNOWN,
        ),
        (
            L._ChildResult(1, b"508 Bad reply from daemon\n", started=True),
            D.OUTCOME_UNKNOWN,
        ),
        (L._ChildResult(-9, started=True), D.OUTCOME_UNKNOWN),
        (
            L._ChildResult(0, b"200 OK\n", started=True, overflow=True),
            D.OUTCOME_UNKNOWN,
        ),
    ],
)
def test_step_outcomes(monkeypatch, response, expected):
    adapter, _, _, replies = configured(monkeypatch)
    replies[0] = response
    assert (
        adapter.apply_pending_correction_by_step(deadline_monotonic_us=100).disposition
        is expected
    )


# Child output is bounded and an overproducing process is killed/reaped without unbounded allocation.
def test_real_child_output_bound():
    result = L._run_child(
        (sys.executable, "-c", 'import os; os.write(1,b"x"*8192)'),
        FakeOsClock(),
        1_000_000,
    )
    assert result.overflow and result.started and len(result.stdout) == 4096


# An explicit readiness output advances virtual time to expiry and causes exact child termination.
def test_real_child_virtual_deadline(monkeypatch):
    clock = FakeOsClock()
    read = P.os.read

    def ready(fd, count):
        data = read(fd, count)
        if data == b"READY\n":
            clock.advance_elapsed_us(1000)
        return data

    monkeypatch.setattr(P.os, "read", ready)
    result = L._run_child(
        (
            sys.executable,
            "-c",
            'import os,signal; os.write(1,b"READY\\n"); signal.pause()',
        ),
        clock,
        1000,
    )
    assert result.timed_out and result.returncode == -9


# Construction rejects host/fallback injection and unsupported version replies before use.
@pytest.mark.parametrize("path", ["localhost", "/run/a,/run/b", "/run/a\ntracking"])
def test_reject_socket_fallback(path):
    with pytest.raises(ValueError):
        L.LinuxChronyControl(FakeOsClock(), socket_path=path, deadline_monotonic_us=100)
