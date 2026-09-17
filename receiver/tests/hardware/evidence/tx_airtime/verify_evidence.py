"""Check retained airtime measurements; removed databases were audited at curation."""

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text())


def passed(record, count):
    result = record["extracted"]
    assert result["tests"] == count
    assert result["failures"] == result["errors"] == result["skipped"] == 0
    assert all(case["outcome"] == "passed" for case in result["cases"])


def verify():
    results = read(ROOT / "results.json")
    manifest = ROOT / "source-manifest.json"
    manifest_hash = hashlib.sha256(manifest.read_bytes()).hexdigest()
    assert manifest_hash == results["source_manifest_sha256"]
    assert len(read(manifest)["files"]) == 327
    passed(results["component"], 17)
    path = ROOT / "grant-lifetime.json"
    assert hashlib.sha256(path.read_bytes()).hexdigest() == results["lifetime_capture"]["sha256"]
    life = read(path)
    deadline, samples = life["grant_deadline_us"], life["samples"]
    assert len(samples) == 1707
    assert all(reason == "ALLOWED" and start < deadline and start <= end
               for start, end, reason in samples[:-1])
    assert samples[-1][2] == "GRANT_EXPIRED"
    assert life["early_tolerance_us"] == 0
    assert 0 <= samples[-1][1] - deadline == 564 <= life["late_tolerance_us"] == 500000
    assert 0 <= deadline - samples[-2][0] == 632 <= life["last_allowed_tolerance_us"] == 50000
    assert 0 < deadline - life["acknowledged_us"] == 1952962 < 2000000
    assert deadline < life["seeded_monotonic_us"] + 2000000
    clock = results["lifetime_clock"]["data"]
    assert clock["fixture_maximum_network_skew_ppb"] == 10000 and clock["uid"] == 0
    sample = clock["correlation"]["sample"]
    end = life["expiration_utc_us"] - 3720000000
    offset = sample["utc_us"] - sample["monotonic_us"]
    lower = life["before_commit_us"] + ((end - offset - life["before_commit_us"]) * 9963) // 10000
    upper = life["acknowledged_us"] + ((end - offset - life["acknowledged_us"]) * 9963) // 10000
    assert lower <= deadline <= upper
    seed, replacement = [results["restart"][k]["data"] for k in ("seed", "replacement")]
    assert seed["boot"] == replacement["boot"] and seed["instance"] != replacement["instance"]
    assert seed["generation"] == 3 and replacement["generation"] == 4
    assert replacement["loaded_baseline_us"] == 1067866
    assert seed["buckets"][0]["expires_at_utc_us"] == replacement["buckets"][0]["expires_at_utc_us"]
    assert replacement["correlation"]["sample"]["monotonic_us"] >= replacement["started_at_monotonic_us"] > seed["correlation"]["sample"]["monotonic_us"]
    controller = results["controller"]["data"]
    assert controller["passed"] is True
    before, restored = controller["before"], controller["restored"]
    final = results["final_host"]["data"]
    assert before["config_sha256"] == restored["config_sha256"] == final["config_sha256"]
    for state in (before, restored, final):
        assert state["source_manifest_sha256"] == manifest_hash
        assert state["chrony"] == "active" and state["tracking"].split(",")[-1] == "Normal"
    assert final["boot"] == restored["boot"]
    for mode, run in results["reboot"].items():
        before, after = [run[k]["data"] for k in ("before", "after")]
        assert before["mode"] == after["mode"] == mode
        assert before["boot"] != after["boot"] and before["instance"] != after["instance"]
        assert before["state_sha256"] == after["state_sha256"]
        assert before["generation"] == after["generation"]
        assert before["total_used"] == 8000000 and after["allowance"] == 0
        assert before["buckets"] == after["buckets"]
        assert sum(b["charged_airtime_us"] for b in after["buckets"]) == 8000000
        if mode == "trusted":
            assert after["total_used"] == 8000000
            assert after["sample"]["monotonic_us"] >= after["instance_started_at_monotonic_us"]
            assert before["sample"]["utc_us"] < after["sample"]["utc_us"] < before["buckets"][0]["expires_at_utc_us"]
        else:
            assert after["total_used"] is None and after["sample"] is None
        passed(run["prepare"], 1)
        passed(run["verify"], 1)
        for record in run["clocks"]:
            clock = record["data"]
            assert clock["fixture_maximum_network_skew_ppb"] == 10000 and clock["uid"] == 0
            if clock["trusted_source_requested"]:
                sample = clock["correlation"]["sample"]
                assert sample["quality"] == "SystemTimeQuality.NETWORK_SYNCED"
                assert 0 < sample["error_bound_us"] < 40000000
                assert sample["monotonic_us"] >= clock["instance"]["started_at_monotonic_us"]
            else:
                assert clock["correlation"] is None
    assert results["reboot"]["trusted"]["after"]["data"]["boot"] == results["reboot"]["unavailable"]["before"]["data"]["boot"]
    assert results["first_controller_attempt"]["status"] == "failed"
    assert results["first_controller_attempt"]["unavailable_verify"] == "NOT RUN"
    assert results["database_audit"]["status"] == "passed_before_deletion"
    assert len(results["database_audit"]["databases"]) == 10
    return {"verified": True, "lifetime_samples": len(samples), "component_tests": 17,
            "reboot_phases": 4, "restart_baseline_us": replacement["loaded_baseline_us"],
            "limits": "Checks retained observations; database integrity was checked before curation deletion. No RF or power-loss claim."}


if __name__ == "__main__":
    print(json.dumps(verify(), indent=2))
