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
    assert (
        result.reference_id,
        result.reference_time_utc_us,
        result.stratum,
        result.root_delay_us,
        result.root_dispersion_us,
        result.estimated_frequency_ppb,
    ) == (0xB99DE5FE, 1789237487797281, 3, 24404, 9398, 5867)
    assert result.evidence().estimated_frequency_ppb == 5867


# The receiver Pi's own capture (2026-10-07) fixes the documented sign conventions:
# chronyc printed "2.640134096 seconds slow of NTP time" and "6.208 ppm fast".
PI_CSV = b"A29FC87B,162.159.200.123,4,1791399269.344973790,2.640070677,-0.000615063,0.000504202,6.208,1.894,0.166,0.038470995,0.001050412,65.1,Normal\n"


def test_pi_capture_sign_conventions():
    result = L.parse_tracking(PI_CSV, 0, 0)
    assert result.remaining_correction_us == 2_640_071  # positive: system clock behind
    assert result.estimated_frequency_ppb == 6_208  # positive: uncorrected clock fast
    assert (result.root_delay_us, result.root_dispersion_us) == (38_471, 1_051)
    assert result.root_distance_us == 20_286  # one ceiling over the exact sum


# Retained facts use documented roundings: reference time down, frequency toward zero,
# delay and dispersion up; they are separate from the combined distance rounding.
@pytest.mark.parametrize(
    "reference,frequency,delay,dispersion,expected",
    [
        ("1.0000019", "-0.0019", "0.0000000001", "0", (1_000_001, -1, 1, 0)),
        ("1.9999999", "0.0019", "0", "0.0000010001", (1_999_999, 1, 0, 2)),
        ("0", "-12.3456", "0.000002", "0.000003", (0, -12_345, 2, 3)),
    ],
)
def test_retained_fact_rounding(reference, frequency, delay, dispersion, expected):
    fields = CSV.decode().strip().split(",")
    fields[3], fields[7], fields[10], fields[11] = reference, frequency, delay, dispersion
    value = L.parse_tracking(",".join(fields).encode(), 0, 0)
    assert (
        value.reference_time_utc_us,
        value.estimated_frequency_ppb,
        value.root_delay_us,
        value.root_dispersion_us,
    ) == expected


# Failed queries carry no retained facts, exactly like the policy facts.
@pytest.mark.parametrize(
    "field", ["reference_id", "reference_time_utc_us", "stratum", "root_delay_us",
              "root_dispersion_us", "estimated_frequency_ppb"]
)
def test_failed_result_rejects_retained_facts(field):
    with pytest.raises(ValueError):
        L.ChronyTrackingResult(Q.UNAVAILABLE, 0, 0, **{field: 1})


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
        (7, "9223372036854775.808"),
        (3, "9223372036854.775808"),
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


@pytest.mark.parametrize("changes,expected", [
    ({}, Q.UNAVAILABLE),
    ({"returncode": 0}, Q.INVALID_RESPONSE),
    ({"returncode": 2}, Q.INVALID_RESPONSE),
    ({"stdout": b"unexpected"}, Q.INVALID_RESPONSE),
    ({"stderr": b"Could not open connection to daemon"}, Q.INVALID_RESPONSE),
    ({"stderr": b"Could not open connection to daemon\nextra"}, Q.INVALID_RESPONSE),
    ({"overflow": True}, Q.INVALID_RESPONSE),
    ({"timed_out": True}, Q.DEADLINE_EXCEEDED),
])
def test_pinned_connection_failure(monkeypatch, changes, expected):
    adapter, _, _, replies = configured(monkeypatch)
    fields = dict(returncode=1, stdout=b"",
                  stderr=b"Could not open connection to daemon\n", started=True)
    fields.update(changes)
    replies[0] = L._ChildResult(**fields)
    assert adapter.read_tracking(deadline_monotonic_us=100).status is expected
    assert adapter.apply_pending_correction_by_step(
        deadline_monotonic_us=100).disposition is D.OUTCOME_UNKNOWN


def test_connection_failure_at_deadline(monkeypatch):
    adapter, clock, _, replies = configured(monkeypatch)
    replies[0] = L._ChildResult(1, b"", b"Could not open connection to daemon\n", started=True)
    assert adapter.read_tracking(deadline_monotonic_us=clock.now_monotonic_us()).status is Q.DEADLINE_EXCEEDED
