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


# An existing short-lived reservation expires on the actual Pi clock.
def test_target_historical_entry_expiration(tmp_path):
    samples = []
    with component(tmp_path, seed_remaining_us=2_000_000) as (airtime, clock, runtime, _, seeded):
        before = clock.now_monotonic_us()
        assert airtime.recover(deadline_monotonic_us=before+5_000_000).reason is R.STATE_READY
        deadline = airtime._ledger.deadlines[0]
        acknowledged = clock.now_monotonic_us()
        assert acknowledged < deadline <= acknowledged+2_007_400
        while clock.now_monotonic_us() <= deadline+500_000:
            start = clock.now_monotonic_us()
            total = airtime.total_used
            finish = clock.now_monotonic_us()
            samples.append([start, finish, total])
            if total == 2_000_000:
                assert finish >= deadline
                break
            assert total == 4_000_000 and start < deadline
            time.sleep(.001)
        assert samples[-1][2] == 2_000_000
        assert samples[-1][1] <= deadline+500_000
        assert 0 <= deadline-samples[-2][0] <= 50_000
        record(tmp_path, 'historical-entry-expiration', dict(
            seeded=seeded, deadline=deadline, acknowledged=acknowledged,
            early_tolerance_us=0, late_tolerance_us=500_000, samples=samples))


def restart_phase(root, phase):
    with component(root, seed_remaining_us=3_600_000_000 if phase == "seed" else None) as (
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
        assert airtime._ledger.current_entry is None
        assert (
            airtime.maintain(
                deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
            ).reason
            is R.ALLOWED
        )
        assert airtime.state.generation == generation
        assert airtime.available_charge_us == 2_000_000
        if phase == "seed":
            spent = airtime.try_spend()
            airtime.report_tx(spent.token, TxCertainty.UNCERTAIN)
            assert (
                airtime.save(
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
            entries=[asdict(b) for b in airtime.state.entries],
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
    assert first["loaded_baseline_us"] == 4_000_000
    assert second["loaded_baseline_us"] == 8_000_000
    assert sum(e["remaining_us"] > 0 for e in first["entries"]) == 3
    assert sum(e["remaining_us"] > 0 for e in second["entries"]) == 4
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
