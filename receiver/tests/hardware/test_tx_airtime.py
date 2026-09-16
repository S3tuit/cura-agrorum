"""Clock/SQLite component evidence; no radio operation or receiver service.

The read-only Chrony socket and RTC may require root on the bench. Timing allows
no early expiry, at most 500 ms late observation, and a final allowed sample
within 50 ms of expiry. These are measured scheduling tolerances, not RF bounds.
"""

from dataclasses import asdict
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import time

import pytest

from cura_receiver.platform.linux_boot_identity import read_linux_boot_id
from cura_receiver.tx_airtime import AirtimeReason as R, TxCertainty
from tests.hardware.airtime_component import component, record

pytestmark = pytest.mark.hardware


# A late-in-bucket grant freezes at its shortened original boundary on the actual Pi clock.
def test_target_shortened_grant_lifetime(tmp_path):
    samples = []
    with component(tmp_path, seed_remaining_us=2_000_000) as (
        airtime,
        clock,
        runtime,
        _,
        seeded,
    ):
        correlation = runtime.airtime_correlation()
        seeded_mono, seeded_utc, expiration = seeded
        before = clock.now_monotonic_us()
        assert (
            airtime.acquire_grant(deadline_monotonic_us=before + 5_000_000).reason
            is R.ALLOWED
        )
        acknowledged = clock.now_monotonic_us()
        spend = airtime.try_spend()
        assert spend.reason is R.ALLOWED
        airtime.report_tx(spend.token, TxCertainty.NOT_STARTED)
        deadline = spend.grant_deadline_monotonic_us
        try:
            assert spend.bucket_expiration_utc_us == expiration
            # Independent arithmetic brackets the internal call's acquisition timestamp.
            end = expiration - 3_720_000_000
            offset = correlation.sample.utc_us - correlation.sample.monotonic_us
            lower = before + ((end - offset - before) * 9963) // 10000
            upper = acknowledged + ((end - offset - acknowledged) * 9963) // 10000
            assert lower <= deadline <= upper
            assert acknowledged < deadline < seeded_mono + 2_000_000
            assert deadline - acknowledged < 2_000_000
            while clock.now_monotonic_us() <= deadline + 500_000:
                start = clock.now_monotonic_us()
                result = airtime.try_spend()
                finish = clock.now_monotonic_us()
                samples.append([start, finish, result.reason.name])
                if result.reason is R.GRANT_EXPIRED:
                    assert finish >= deadline
                    break
                assert result.reason is R.ALLOWED and start < deadline
                airtime.report_tx(result.token, TxCertainty.NOT_STARTED)
                time.sleep(0.001)
            assert samples[-1][2] == "GRANT_EXPIRED"
            assert samples[-1][1] <= deadline + 500_000
            assert 0 <= deadline - samples[-2][0] <= 50_000
            assert airtime.available_charge_us == 0
        finally:
            record(
                tmp_path,
                "grant-lifetime",
                {
                    "seeded_monotonic_us": seeded_mono,
                    "seeded_utc_us": seeded_utc,
                    "expiration_utc_us": expiration,
                    "before_commit_us": before,
                    "acknowledged_us": acknowledged,
                    "grant_deadline_us": deadline,
                    "early_tolerance_us": 0,
                    "late_tolerance_us": 500_000,
                    "last_allowed_tolerance_us": 50_000,
                    "samples": samples,
                },
            )


def restart_phase(root, phase):
    with component(root, seed_remaining_us=20_000_000 if phase == "seed" else None) as (
        airtime,
        clock,
        runtime,
        instance,
        _,
    ):
        assert airtime.available_charge_us == 0
        assert (
            airtime.recover(
                deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
            ).reason
            is R.STATE_READY
        )
        baseline = airtime.total_used
        generation = airtime.state.generation
        assert airtime.try_spend().reason is R.GRANT_REQUIRED
        assert (
            airtime.acquire_grant(
                deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
            ).reason
            is R.ALLOWED
        )
        assert airtime.state.generation == generation + 1
        assert airtime.available_charge_us == 8_000_000 - baseline
        if phase == "seed":
            spent = airtime.try_spend()
            airtime.report_tx(spent.token, TxCertainty.UNCERTAIN)
            assert (
                airtime.settle(
                    precharge=False,
                    deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000,
                ).reason
                is R.STATE_READY
            )
        result = dict(
            instance=instance.receiver_instance_id.hex(),
            boot=read_linux_boot_id().hex(),
            started_at_monotonic_us=instance.started_at_monotonic_us,
            correlation=asdict(runtime.airtime_correlation()),
            loaded_baseline_us=baseline,
            generation=airtime.state.generation,
            buckets=[asdict(b) for b in airtime.state.buckets if b.charged_airtime_us],
        )
        record(root, phase, result)
        return result


# Separate target processes retain the first process's charge without importing its spending allowance.
def test_target_process_restart_airtime_baseline(tmp_path):
    for phase in ("seed", "replacement"):
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "tests.hardware.test_tx_airtime",
                str(tmp_path),
                phase,
            ],
            capture_output=True,
            text=True,
            timeout=20,
            env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)},
        )
        (tmp_path / (phase + "-stdout.txt")).write_text(result.stdout)
        (tmp_path / (phase + "-stderr.txt")).write_text(result.stderr)
        assert result.returncode == 0, result.stderr
    first, second = [
        json.loads((tmp_path / (phase + ".json")).read_text())
        for phase in ("seed", "replacement")
    ]
    assert first["boot"] == second["boot"] == read_linux_boot_id().hex()
    assert first["instance"] != second["instance"]
    assert first["generation"] == 3 and second["generation"] == 4
    assert second["loaded_baseline_us"] == 1_067_866
    assert (
        first["buckets"][0]["expires_at_utc_us"]
        == second["buckets"][0]["expires_at_utc_us"]
    )
    assert (
        second["correlation"]["sample"]["monotonic_us"]
        >= second["started_at_monotonic_us"]
    )
    assert (
        second["started_at_monotonic_us"]
        > first["correlation"]["sample"]["monotonic_us"]
    )
    with sqlite3.connect(tmp_path / "worker.db") as database:
        assert database.execute("PRAGMA integrity_check").fetchone() == ("ok",)
        assert database.execute("PRAGMA foreign_key_check").fetchall() == []


if __name__ == "__main__":
    restart_phase(Path(sys.argv[1]), sys.argv[2])
