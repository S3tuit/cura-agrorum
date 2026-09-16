"""Independent sustained-availability obligations; no reference-model decisions."""

from collections import deque
from pathlib import Path
import json
import shutil
import sqlite3

from cura_receiver.airtime_ledger import AirtimeCorrelation
from cura_receiver.generated.receiver_entities_generated import (
    decode_communicator_state_v1,
)
from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth as RH,
    SystemTimeQuality as Q,
)
from cura_receiver.time_observations import TrustedTimeSample
from cura_receiver.tx_airtime import AirtimeReason as R, TxCertainty
from tests.support.builders.persistence_control import state


# F-001: steady fresh time must not turn safe per-minute traffic into an uptime-dependent TX outage.
def test_eight_hour_airtime_availability(airtime_component, tmp_path):
    policy, _, database, clock, _ = airtime_component(initial_state=state())
    pending_since = None
    previous_sent = None
    recent = deque()
    events = []
    max_gap = max_deferral = sent = denied = 0
    try:
        for second in range(8 * 3600):
            now = second * 1_000_000 + 100
            clock.advance_elapsed_us(now - clock.now_monotonic_us())
            policy.update_time(
                AirtimeCorrelation(
                    TrustedTimeSample(now, now - 100, 1, Q.NETWORK_SYNCED, second + 1),
                    second + 1,
                    now + 10_000_000,
                ),
                rtc_health=RH.PRESENT,
            )
            if second % 60 != 30 and pending_since is None:
                continue
            if pending_since is None:
                pending_since = second
            result = policy.acquire_grant(deadline_monotonic_us=now + 5_000_000)
            events.append([second, result.reason.name])
            if result.reason is R.ALLOWED:
                spend = policy.try_spend()
                assert spend.reason is R.ALLOWED and spend.token is not None
                assert now < spend.grant_deadline_monotonic_us
                policy.report_tx(spend.token, TxCertainty.STARTED)
                recent.append(second)
                while recent[0] < second - 3600:
                    recent.popleft()
                # Literal protocol charge and physical-time observations are independent of the ledger.
                assert len(recent) * 67_866 <= 36_000_000
                with sqlite3.connect(database) as connection:
                    blob = connection.execute(
                        "SELECT state_blob FROM communicator_state"
                    ).fetchone()[0]
                durable = decode_communicator_state_v1(blob)
                assert durable == policy.state
                assert (
                    sum(b.charged_airtime_us for b in durable.buckets)
                    == policy.total_used
                )
                if previous_sent is not None:
                    max_gap = max(max_gap, second - previous_sent)
                previous_sent = second
                max_deferral = max(max_deferral, second - pending_since)
                pending_since = None
                sent += 1
            else:
                assert result.reason is R.SNAPSHOT_DEFERRED
                assert policy.available_charge_us == 0
                denied += 1
        assert pending_since is None
        assert sent == 480
        assert denied > 0  # Preserve conservative deferral at the safety boundary.
        assert max_deferral <= 14
        assert max_gap <= 74
    except BaseException:
        evidence = tmp_path / "availability-failure"
        evidence.mkdir()
        for suffix in ("", "-wal", "-shm"):
            source = Path(str(database) + suffix)
            if source.exists():
                shutil.copy2(source, evidence / source.name)
        raise
    finally:
        (tmp_path / "availability.json").write_text(
            json.dumps(
                {
                    "seconds": 8 * 3600,
                    "sent": sent,
                    "denied": denied,
                    "max_gap_seconds": max_gap,
                    "max_deferral_seconds": max_deferral,
                    "events": events,
                },
                indent=2,
            )
            + "\n"
        )
