"""Two explicit phases driven across an actual reboot by the external controller."""

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sqlite3

import pytest

from cura_receiver.generated.receiver_entities_generated import (
    encode_communicator_state_v1,
)
from cura_receiver.platform.linux_boot_identity import read_linux_boot_id
from cura_receiver.tx_airtime import AirtimeReason as R
from tests.hardware.airtime_component import component, preserve, record
from tests.hardware.conftest import _validated_destructive_test_root

pytestmark = [pytest.mark.hardware, pytest.mark.destructive]


# A real reboot preserves the UTC ledger; only fresh trusted evidence can reconstruct and admit a new grant.
def test_target_reboot_reconstruction(request):
    config = request.config
    root = _validated_destructive_test_root(config.getoption("receiver_test_root"))
    phase = config.getoption("airtime_reboot_phase")
    mode = config.getoption("airtime_reboot_mode")
    supplied = config.getoption("airtime_reboot_session")
    assert (
        phase and mode and supplied
    ), "use the external airtime reboot controller with explicit phase/mode/session"
    session = Path(supplied).resolve(strict=True)
    assert session.is_relative_to(root) and session != root
    marker = json.loads((session / "controller.json").read_text())
    assert marker == {"workload": "receiver_tx_airtime", "mode": mode}
    before_file = session / "before.json"
    trusted = phase == "prepare" or mode == "trusted"
    if phase == "prepare":
        assert not before_file.exists() and not (session / "worker.db").exists()
    else:
        assert before_file.is_file() and (session / "worker.db").is_file()
        before = json.loads(before_file.read_text())
        assert (
            before["boot"] != read_linux_boot_id().hex()
        ), "Linux boot identity did not change"
        preserve(session, "after-reboot-before-open")
    with component(
        session,
        trusted=trusted,
        seed_remaining_us=20_000_000 if phase == "prepare" else None,
    ) as (airtime, clock, runtime, instance, _):
        if phase == "prepare":
            assert (
                airtime.acquire_grant(
                    deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
                ).reason
                is R.ALLOWED
            )
            assert airtime.total_used == 8_000_000
        else:
            assert airtime.available_charge_us == 0
            assert (
                hashlib.sha256(encode_communicator_state_v1(airtime.state)).hexdigest()
                == before["state_sha256"]
            )
            assert instance.receiver_instance_id.hex() != before["instance"]
            assert airtime.try_spend().reason is (
                R.GRANT_REQUIRED if trusted else R.UNTRUSTED_TIME
            )
            result = airtime.recover(
                deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
            )
            assert result.reason is (R.STATE_READY if trusted else R.UNTRUSTED_TIME)
            assert airtime.total_used == (8_000_000 if trusted else None)
            if trusted:
                correlation = runtime.airtime_correlation()
                assert (
                    correlation.sample.monotonic_us >= instance.started_at_monotonic_us
                )
                assert correlation.sample.utc_us > before["sample"]["utc_us"]
                assert (
                    correlation.sample.utc_us
                    < before["buckets"][0]["expires_at_utc_us"]
                )
            else:
                assert runtime.airtime_correlation() is None
                assert (
                    airtime.acquire_grant(
                        deadline_monotonic_us=clock.now_monotonic_us() + 5_000_000
                    ).reason
                    is R.UNTRUSTED_TIME
                )
                assert airtime.state.generation == before["generation"]
        correlation = runtime.airtime_correlation()
        record(
            session,
            "before" if phase == "prepare" else "after",
            {
                "boot": read_linux_boot_id().hex(),
                "instance": instance.receiver_instance_id.hex(),
                "instance_started_at_monotonic_us": instance.started_at_monotonic_us,
                "sample": asdict(correlation.sample) if correlation else None,
                "generation": airtime.state.generation,
                "total_used": airtime.total_used,
                "allowance": airtime.available_charge_us,
                "state_sha256": hashlib.sha256(
                    encode_communicator_state_v1(airtime.state)
                ).hexdigest(),
                "buckets": [
                    asdict(b) for b in airtime.state.buckets if b.charged_airtime_us
                ],
                "mode": mode,
            },
        )
        preserve(session, phase + "-evidence")
    with sqlite3.connect(session / "worker.db") as database:
        assert database.execute("PRAGMA integrity_check").fetchone() == ("ok",)
        assert database.execute("PRAGMA foreign_key_check").fetchall() == []
