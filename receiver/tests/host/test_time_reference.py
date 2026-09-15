"""Independent, reviewed metrology examples for the real laptop/Pi fixture."""

from decimal import Decimal
import io
import json
import sys
import pytest
from tests.hardware.time_reference import rate_interval, reference_value, slew_decision


# 1200 physical seconds at exactly +/-3500 ppm must yield the signed configured rate.
@pytest.mark.parametrize(
    "delta,expected",
    [(1_204_200_000, 3500), (1_195_800_000, -3500), (1_200_000_000, 0)],
)
def test_independent_rate_examples(delta, expected):
    first = {"utc_us": 0, "error_us": 0, "pi_before_us": 100, "pi_after_us": 100}
    last = {
        "utc_us": 1_200_000_000,
        "error_us": 0,
        "pi_before_us": 100 + delta,
        "pi_after_us": 100 + delta,
    }
    assert rate_interval(first, last) == (Decimal(expected), Decimal(expected))


# Endpoint error and transport brackets widen acceptance; midpoint alone cannot hide a violated bound.
def test_reference_uncertainty_is_charged():
    first = {"utc_us": 0, "error_us": 20_000, "pi_before_us": 0, "pi_after_us": 40_000}
    last = {
        "utc_us": 1_200_000_000,
        "error_us": 20_000,
        "pi_before_us": 1_204_320_000,
        "pi_after_us": 1_204_360_000,
    }
    low, high = rate_interval(first, last)
    assert 3500 < low < 3600 < high < 3700
    last["pi_before_us"] += 200_000
    last["pi_after_us"] += 200_000
    assert rate_interval(first, last)[1] > 3700


CSV = "CCD8D64C,204.216.214.76,3,1789240501,0.0005,0,0,4,0,0.1,0.026,0.001,64,Normal"


# Reported 14.5 ms error plus a 2 ms half-bracket and one us integer margin gives 16,501 us.
def test_laptop_interval_example():
    result = reference_value(1_000_000, 1_004_000, CSV, {"192.168.1.10"})
    assert result["utc_us"] == 1_002_000 and result["error_us"] == 16_501


@pytest.mark.parametrize(
    "csv,addresses,finish",
    [
        (CSV, {"204.216.214.76"}, 1_004_000),
        (CSV.replace("Normal", "Not synchronised"), set(), 1_004_000),
        (CSV, set(), 999_999),
        (CSV, set(), 1_100_001),
    ],
)
def test_unusable_reference_rejected(csv, addresses, finish):
    with pytest.raises(ValueError):
        reference_value(1_000_000, finish, csv, addresses)


# The first result with <200 ppm width after 20 minutes is terminal, including
# rate failures. Extending those failures would select evidence by its outcome.
@pytest.mark.parametrize(
    "elapsed,low,high,sign,expected",
    [
        (1_199_999_999, "3400", "3500", 1, "CONTINUE"),
        (1_200_000_000, "3400", "3500", 1, "PASS"),
        (1_200_000_000, "3500", "3700", 1, "CONTINUE"),
        (1_200_000_000, "3500", "3699.999999", 1, "PASS"),
        (2_399_999_999, "3500", "3700", 1, "CONTINUE"),
        (2_400_000_000, "3500", "3700", 1, "FAIL_RESOLUTION"),
        (2_400_000_001, "3500", "3700", 1, "FAIL_RESOLUTION"),
        (2_400_000_000, "3501", "3700", 1, "PASS"),
        (1_200_000_000, "3600", "3700.000001", 1, "FAIL_RATE_BOUND"),
        (2_300_000_000, "3600", "3700.000001", 1, "FAIL_RATE_BOUND"),
        (1_200_000_000, "3300", "3400", 1, "FAIL_MAXIMUM_SLEW"),
        (1_200_000_000, "3300.000001", "3400", 1, "PASS"),
        (1_200_000_000, "-3500", "-3400", -1, "PASS"),
        (1_200_000_000, "-3700", "-3500", -1, "CONTINUE"),
        (2_400_000_000, "-3700", "-3500", -1, "FAIL_RESOLUTION"),
        (1_200_000_000, "-3700", "-3501", -1, "PASS"),
        (1_200_000_000, "-3700.000001", "-3600", -1, "FAIL_RATE_BOUND"),
        (1_200_000_000, "-3400", "-3300", -1, "FAIL_MAXIMUM_SLEW"),
        (1_200_000_000, "-3400", "-3300.000001", -1, "PASS"),
        (1_200_000_000, "3400", "3500", -1, "FAIL_MAXIMUM_SLEW"),
    ],
)
def test_resolution_driven_duration_examples(elapsed, low, high, sign, expected):
    assert slew_decision(elapsed, Decimal(low), Decimal(high), sign) == expected


# A failed first exchange must leave actual readable evidence, even when it is
# EOF, malformed JSON, or the laptop's explicit reference-error response.
@pytest.mark.parametrize(
    "reply",
    [
        "",
        "{broken json\n",
        '{"version": 0, "error": "laptop uncertainty exceeds fixture ceiling"}\n',
        '{"version": 1, "source_independent": false, "utc_us": 1, "error_us": 0}\n',
        '{"version": 1, "source_independent": true, "utc_us": 1, "error_us": -1}\n',
        '{"version": 1, "source_independent": true, "utc_us": 1, "error_us": 100001}\n',
    ],
)
def test_failed_reference_evidence_is_retained(tmp_path, monkeypatch, reply):
    from tests.hardware.test_time_mutations import measure_slew
    from tests.support.fakes.os_clock import FakeOsClock
    from tests.support.fakes.chrony import FakeChronyControl

    monkeypatch.setattr(sys, "stdin", io.StringIO(reply))
    chrony = FakeChronyControl()
    with pytest.raises((AssertionError, ValueError)):
        measure_slew(
            {"offset": 120, "data": str(tmp_path)},
            FakeOsClock(monotonic_us=123),
            chrony,
        )
    result = json.loads((tmp_path / "maximum-slew.json").read_text())
    assert result["status"] == "ERROR" and result["error"]
    assert result["intervals"] == [] and chrony.calls == []
    assert len(result["samples"]) == 1
    sample = result["samples"][0]
    assert sample["raw_reply"] == reply and sample["valid"] is False
    assert sample["pi_before_us"] == sample["pi_after_us"] == 123


# With no reference error, a 4000 ppm clock must fail at the first eligible
# sample (1200 s). At 3500 ppm and 100 ms error at each endpoint, the width is
# still >200 ppm at 2000 s and first falls below it at the 2020 s sample.
@pytest.mark.parametrize(
    "rate,error_us,steps,expected",
    [(4000, 0, 60, "FAIL_RATE_BOUND"), (3500, 100_000, 101, "PASS")],
)
def test_driver_retains_intervals_and_stops_at_first_decision(
    tmp_path, monkeypatch, rate, error_us, steps, expected
):
    from cura_receiver.ports.chrony import ChronyTrackingResult, ChronyQueryStatus
    from tests.hardware import test_time_mutations as mutations
    from tests.support.fakes.os_clock import FakeOsClock
    from tests.support.fakes.chrony import FakeChronyControl

    clock = FakeOsClock()
    chrony = FakeChronyControl()
    chrony.tracking_results.extend(
        lambda: ChronyTrackingResult(
            ChronyQueryStatus.OK,
            clock.now_monotonic_us(),
            clock.now_monotonic_us(),
            True,
            True,
            120_000_000,
            0,
            0,
        )
        for _ in range(120)
    )
    replies = [
        {
            "version": 1,
            "source_independent": True,
            "utc_us": 1_000_000 + i * 20_000_000 * 1_000_000 // (1_000_000 + rate),
            "error_us": error_us,
        }
        for i in range(121)
    ]
    monkeypatch.setattr(
        sys, "stdin", io.StringIO("".join(json.dumps(r) + "\n" for r in replies))
    )
    monkeypatch.setattr(
        mutations.time,
        "sleep",
        lambda seconds: clock.advance_elapsed_us(int(seconds * 1_000_000)),
    )
    if expected == "PASS":
        mutations.measure_slew({"offset": 120, "data": str(tmp_path)}, clock, chrony)
    else:
        with pytest.raises(AssertionError, match=expected):
            mutations.measure_slew({"offset": 120, "data": str(tmp_path)}, clock, chrony)
    result = json.loads((tmp_path / "maximum-slew.json").read_text())
    assert result["status"] == expected
    assert len(chrony.calls) == len(result["intervals"]) == steps
    assert len(result["samples"]) == steps + 1
    assert all(s["valid"] for s in result["samples"])
    assert all(i["decision"] == "CONTINUE" for i in result["intervals"][:-1])
    assert result["intervals"][-1]["decision"] == expected
    assert result["intervals"][-1]["elapsed_monotonic_us"] == steps * 20_000_000
