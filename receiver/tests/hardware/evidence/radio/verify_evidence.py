"""Check the curated 2026-09-17 radio results without a Pi or current sources."""

import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def passed(record, count):
    result = record["extracted"]
    assert result["tests"] == count
    assert result["failures"] == result["errors"] == result["skipped"] == 0
    assert all(case["outcome"] == "passed" for case in result["cases"])


def verify():
    results = read(ROOT / "results.json")
    assert digest(ROOT / "source.tar.gz") == results["source_archive_sha256"]
    assert digest(ROOT / "source-manifest.json") == results["source_manifest_sha256"]
    with tarfile.open(ROOT / "source.tar.gz") as archive:
        members = {m.name: hashlib.sha256(archive.extractfile(m).read()).hexdigest()
                   for m in archive if m.isfile()}
    manifest = read(ROOT / "source-manifest.json")
    assert len(members) == results["source_archive_files"] == 235
    assert len(manifest) == results["source_manifest_files"] == 189
    assert all(members.get(path) == expected for path, expected in manifest.items())
    for name, origin in results["original_files"].items():
        assert digest(ROOT.parent / name) == origin["sha256"], name

    nominal, held, restored = [results[k] for k in ("initial_nominal", "held", "restored")]
    for record, count in ((nominal, 3), (held, 1), (restored, 3)):
        passed(record["junit"], count)
    assert int(nominal["exit_status"]["text"]) == 0
    for record in (nominal, restored):
        assert len(record["teardowns"]) == 3
        assert all(t["data"]["state"] == "SHUTDOWN" and t["data"]["safe_shutdown"] is True
                   for t in record["teardowns"].values())
    initialization = held["initialization"]["data"]
    fault = initialization["result"]
    assert fault["state"] == "INITIALIZATION_FAILED" and fault["safe_shutdown"] is False
    assert [(e["operation"], e["error_code"], e["severity"], e["context"]["trigger_detail"]["stage"])
            for e in fault["episodes"]] == [
                ("INITIALIZE", "BUSY_TIMEOUT", "FATAL", "WAIT_BUSY"),
                ("CLEANUP", "BUSY_TIMEOUT", "FATAL", "WAIT_BUSY")]
    assert fault["busy"]["timeout_count"] == 2
    assert fault["tx"] is None and fault["t6_set_rx_issued_monotonic_us"] is None
    assert initialization["elapsed_us"] == 277477 <= 2050000
    assert held["observation"]["data"] == {
        "expected_fault_observed": True, "safe_shutdown_confirmed": False,
        "manual_restoration_required": True}
    assert held["teardown"]["data"]["safe_shutdown"] is False
    assert held["session"]["status"] == "aborted_safe_shutdown_unconfirmed"
    assert "radio safe-state teardown unconfirmed" in held["session"]["abort_line"]
    assert held["session"]["exact_command"] is held["session"]["numeric_exit"] is None
    assert restored["exact_command"] is restored["numeric_exit"] is None
    assert held["target"]["data"]["boot_id"] != restored["target"]["data"]["boot_id"]
    assert held["fixture"]["data"]["fixture_state"] == "radio_busy_held"
    assert restored["fixture"]["data"]["fixture_state"] == "radio_nominal"
    traces = {p.name: [json.loads(line) for line in p.read_text().splitlines()]
              for p in ROOT.glob("*.jsonl")}
    assert set(traces) == {"held-busy.jsonl", *restored["traces"].values()}
    busy = [e["result"] for e in traces["held-busy.jsonl"] if e["operation"] == "busy"]
    assert len(busy) == 809 and all(v is True for v in busy)
    assert not any(e["operation"] == "spi" or "error" in e for e in traces["held-busy.jsonl"])
    assert any(e["operation"] == "close" for e in traces["held-busy.jsonl"])
    assert not any(e["operation"] == "spi" and e["tx"].startswith("83")
                   for events in traces.values() for e in events)
    operator = read(ROOT / "operator-observations.json")
    assert operator["restoration"]["external_power_removed"] is True
    assert operator["restoration"]["selector_restored_to_nominal"] is True
    assert operator["restoration"]["independent_power_removal_measurement"] is False
    return {"verified": True, "source_files": len(members), "traces": len(traces),
            "held_high_samples": len(busy), "held_safe_shutdown": False,
            "restored_safe_teardowns": 3, "physical_tx_observed": False,
            "limits": results["limits"]}


if __name__ == "__main__":
    print(json.dumps(verify(), indent=2))
