"""Independently audit this run using exact rational arithmetic.

Usage: python3 verify_slew_evidence.py ARCHIVE_DIRECTORY
Reads retained evidence only; imports no receiver or fixture implementation.
"""

from decimal import Decimal, localcontext
from fractions import Fraction as Q
import json
from pathlib import Path
import sys


def rendered(value):
    with localcontext() as context:
        context.prec = 45
        return str(Decimal(value.numerator) / Decimal(value.denominator))


def ceil(value):
    return -(-value.numerator // value.denominator)


def audit(archive):
    postflight = json.loads((archive / "postflight.json").read_text())
    pi_addresses = next(
        c["stdout"].split() for c in postflight["commands"]
        if c["argv"] == ["hostname", "-I"] and c["exit"] == 0
    )
    cases = []
    paths = [archive / "positive.json", archive / "negative.json"]
    paths.sort(key=lambda p: json.loads(p.read_text())["samples"][0]["pi_before_us"])
    for path in paths:
        result = json.loads(path.read_text())
        samples = result["samples"]
        sign = 1 if result["offset"] > 0 else -1
        assert result["offset"] in (-120, 120)
        for sample in samples:
            reply = json.loads(sample["raw_reply"])
            assert reply == sample["reference_reply"]
            assert sample["valid"] is True
            assert sample["pi_before_us"] <= sample["pi_after_us"]
            assert reply["version"] == 1 and reply["source_independent"] is True
            csv = reply["tracking_csv"].split(",")
            assert len(csv) == 14 and csv[13] == "Normal"
            assert csv[1] not in pi_addresses
            assert csv[0] not in ("00000000", "7F7F0101")
            assert 1 <= int(csv[2]) <= 15
            span = reply["laptop_after_us"] - reply["laptop_before_us"]
            assert 0 <= span <= 100_000
            correction, delay, dispersion = (Q(csv[i]) for i in (4, 10, 11))
            assert min(delay, dispersion) >= 0
            error = ceil((abs(correction) + delay / 2 + dispersion) * 1_000_000)
            error += (span + 1) // 2 + 1
            assert error == reply["error_us"] == sample["error_us"]
            assert 0 <= error <= 100_000
            midpoint = (reply["laptop_after_us"] + reply["laptop_before_us"]) // 2
            assert midpoint == reply["utc_us"] == sample["utc_us"]
        assert len(result["intervals"]) == len(samples) - 1
        assert len(result["tracking"]) == len(result["intervals"])
        for tracking in result["tracking"]:
            assert tracking["status"] == "ChronyQueryStatus.OK"
            assert tracking["synchronized"] is True
            assert tracking["source_selected"] is True
            assert sign * tracking["remaining_correction_us"] > 100_000_000
        first = samples[0]
        decisions = []
        for last, recorded in zip(samples[1:], result["intervals"]):
            elapsed = last["utc_us"] - first["utc_us"]
            error = first["error_us"] + last["error_us"]
            assert elapsed > error
            low = Q(last["pi_before_us"] - first["pi_after_us"], elapsed + error)
            high = Q(last["pi_after_us"] - first["pi_before_us"], elapsed - error)
            low, high = (x * 1_000_000 - 1_000_000 for x in (low, high))
            width = high - low
            assert low <= high
            # The fixture uses 28-significant-digit Decimal calculations. Permit
            # only its sub-attoppm rounding when comparing exact rational results.
            for exact, saved in zip((low, high, width), (*recorded["rate_interval_ppm"], recorded["width_ppm"])):
                assert abs(exact - Q(saved)) < Q(1, 10**19), (exact, saved)
            duration = last["pi_after_us"] - first["pi_after_us"]
            assert duration == recorded["elapsed_monotonic_us"]
            if duration < 1_200_000_000:
                decision = "CONTINUE"
            elif width >= 200:
                decision = "CONTINUE" if duration < 2_400_000_000 else "FAIL_RESOLUTION"
            elif low < -3700 or high > 3700:
                decision = "FAIL_RATE_BOUND"
            elif (sign == 1 and low <= 3300) or (sign == -1 and high >= -3300):
                decision = "FAIL_MAXIMUM_SLEW"
            else:
                decision = "PASS"
            assert decision == recorded["decision"]
            decisions.append(decision)
        assert decisions and all(d == "CONTINUE" for d in decisions[:-1])
        assert decisions[-1] == result["status"] != "CONTINUE"
        assert result["rate_interval_ppm"] == result["intervals"][-1]["rate_interval_ppm"]
        # Postflight contains the original per-case restoration records.
        matches = [value for value in postflight["restorations"].values()
                   if value["offset"] == result["offset"]]
        assert len(matches) == 1
        restoration = matches[0]
        assert restoration["restored"] is True
        cases.append({
            "evidence": path.name,
            "offset_s": result["offset"],
            "status": result["status"],
            "samples": len(samples),
            "intervals": len(decisions),
            "elapsed_monotonic_us": duration,
            "elapsed_reference_us": elapsed,
            "rate_interval_ppm_exact_rational": [str(low), str(high)],
            "rate_interval_ppm_decimal_45_digits": [rendered(low), rendered(high)],
            "interval_width_ppm_decimal_45_digits": rendered(width),
            "extended_for_resolution": any(
                i["elapsed_monotonic_us"] >= 1_200_000_000 and i["decision"] == "CONTINUE"
                for i in result["intervals"]
            ),
            "all_intervals_recomputed": True,
            "first_eligible_resolved_result_stops": True,
            "laptop_uncertainty_recomputed_for_every_sample": True,
            "restored": True,
        })
    assert len(cases) == 2 and {c["offset_s"] for c in cases} == {-120, 120}
    print(json.dumps({"method": "exact Fraction arithmetic; no receiver/fixture imports", "pi_addresses": pi_addresses, "cases": cases}, indent=2))


if __name__ == "__main__":
    audit(Path(sys.argv[1]))
