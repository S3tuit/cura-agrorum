"""Differential event sequences and independent physical-time retention bounds."""

from dataclasses import replace
from fractions import Fraction
from math import ceil
from pathlib import Path
import shutil
import sqlite3
import tempfile
import uuid

from hypothesis import example, given, settings, strategies as st, HealthCheck

from cura_receiver.airtime_ledger import AirtimeCorrelation, AirtimeLedger
from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.communicator_state_persistence import CommunicatorStatePolicy
from cura_receiver.generated.receiver_entities_generated import (
    TxAirtimeBucketV1 as Bucket,
    communicator_state_v1_parameters,
    decode_communicator_state_v1,
)
from cura_receiver.generated.receiver_enums_generated import (
    DiagnosticOperation as Op,
    RtcHealth as RH,
    SystemTimeQuality as Q,
)
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as CD,
    CommunicatorStateCommitFailureKind as CF,
)
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.time_observations import TrustedTimeSample
from cura_receiver.tx_airtime import TxAirtimePolicy, TxCertainty, AirtimeReason as R
from tests.support.builders.persistence_control import state
from tests.support.coordination.persistence_worker import (
    CheckedPersistenceWorker,
    prepare_worker_files,
)
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.models.tx_airtime import LedgerModel


class CommitFaults:
    """Inject reply uncertainty at the real worker channel, never in SQL behavior."""

    def __init__(self, control):
        self.control = control
        self.mode = "committed"
        self.fail_load = False

    def commit_communicator_state(self, value, **kwargs):
        mode, self.mode = self.mode, "committed"
        if mode in ("not_installed", "unknown_absent"):
            kwargs["deadline_monotonic_us"] = 0
        result = self.control.commit_communicator_state(value, **kwargs)
        assert result.disposition is (
            CD.NOT_INSTALLED
            if mode in ("not_installed", "unknown_absent")
            else CD.COMMITTED
        )
        if mode.startswith("unknown"):
            return replace(
                result,
                disposition=CD.OUTCOME_UNKNOWN,
                failure_kind=CF.DEADLINE_EXCEEDED,
                operation=Op.WRITE,
            )
        return result

    def load_communicator_state(self, **kwargs):
        if self.fail_load:
            kwargs["deadline_monotonic_us"] = 0
        return self.control.load_communicator_state(**kwargs)


OUTCOMES = st.sampled_from(
    ["committed", "not_installed", "unknown_installed", "unknown_absent"]
)
ACTIONS = st.one_of(
    st.tuples(st.just("grant"), OUTCOMES),
    st.tuples(st.just("settle"), st.booleans(), OUTCOMES),
    st.tuples(
        st.just("spend"),
        st.sampled_from(["NOT_STARTED", "STARTED", "UNCERTAIN", "MISSING"]),
    ),
    st.tuples(
        st.just("burst"),
        st.integers(1, 130),
        st.sampled_from(["NOT_STARTED", "STARTED", "UNCERTAIN", "MISSING"]),
    ),
    st.tuples(
        st.just("advance"),
        st.sampled_from(
            [0, 1, 1_000_000, 59_777_999, 60_000_000, 60_222_000, 61_000_000, 3_780_000_000]
        ),
    ),
    st.tuples(
        st.just("trust"), st.booleans(), st.sampled_from([-34_000_000, 0, 34_000_000])
    ),
    st.tuples(st.just("reconcile"), st.booleans()),
    st.tuples(st.just("restart")),
)


def projection(value):
    evidence = value.airtime_snapshot
    return (value.generation, None if evidence is None else
            (evidence.utc_us, evidence.error_bound_us),
            tuple(b.charged_airtime_us for b in value.buckets))


# Every generated policy decision and durable ledger is compared with a separate list/rational-time oracle.
@settings(
    max_examples=70,
    deadline=None,
    derandomize=True,
    suppress_health_check=[HealthCheck.function_scoped_fixture],
)
@given(
    charges=st.lists(st.integers(0, 8_000_000), max_size=4),
    actions=st.lists(ACTIONS, min_size=1, max_size=70),
)
@example(
    charges=[],
    actions=[
        action
        for _ in range(7)
        for action in (
            ("grant", "committed"),
            ("burst", 130, "STARTED"),
            ("advance", 60_000_000),
        )
    ],
)
@example(
    charges=[],
    actions=[
        action
        for _ in range(7)
        for action in (
            ("grant", "unknown_installed"),
            ("restart",),
            ("burst", 130, "MISSING"),
            ("advance", 60_000_000),
        )
    ],
)
def test_generated_airtime_sequences_against_independent_model(
    tmp_path, charges, actions
):
    root = Path(tempfile.mkdtemp(prefix="ledger-sequence-", dir=tmp_path))
    database, config, boot = prepare_worker_files(root)
    initial = state(buckets=(Bucket(0),) * (62 - len(charges)) +
                    tuple(Bucket(c) for c in charges))
    with sqlite3.connect(database) as connection:
        connection.execute(
            "INSERT INTO communicator_state VALUES (?,?,?,?,?)",
            communicator_state_v1_parameters(initial),
        )
    clock = FakeOsClock(monotonic_us=100)
    model = LedgerModel(charges)
    enabled, bias = True, 0
    worker = policy = faults = None
    generation = 0

    def supply_time():
        nonlocal generation
        generation += 1
        now = clock.now_monotonic_us()
        correlation = (
            AirtimeCorrelation(
                TrustedTimeSample(
                    now, now - 100 + bias, abs(bias) + 1, Q.NETWORK_SYNCED, generation
                ),
                generation,
                now + 10_000_000_000,
            )
            if enabled
            else None
        )
        policy.update_time(correlation, rtc_health=RH.PRESENT)

    def start():
        nonlocal worker, policy, faults
        now = clock.now_monotonic_us()
        worker = CheckedPersistenceWorker(
            instance=ReceiverInstanceStart(uuid.uuid4().bytes, now),
            database_path=database,
            configuration_path=config,
            boot_id_path=boot,
            clock=clock,
        )
        worker.start()
        startup = worker.wait_started(deadline_monotonic_us=now + 5_000_000)
        assert startup is not None and startup.database_failure is None
        faults = CommitFaults(worker.control)
        owner = CommunicatorStateOwner.from_load(
            control=faults, loaded=startup.state_load
        )
        policy = TxAirtimePolicy(state_owner=owner, clock=clock)
        supply_time()
        policy.recover(deadline_monotonic_us=now + 5_000_000)

    start()
    try:
        for index, action in enumerate(actions):
            faults.mode, faults.fail_load = "committed", False
            deadline = clock.now_monotonic_us() + 5_000_000
            tag = action[0]
            actual = expected = None
            if tag == "grant":
                faults.mode = action[1]
                actual = policy.acquire_grant(
                    deadline_monotonic_us=deadline
                ).reason.name
                expected = model.acquire(action[1])
            elif tag == "settle":
                faults.mode = action[2]
                actual = policy.settle(
                    precharge=action[1], deadline_monotonic_us=deadline
                ).reason.name
                expected = model.settle(action[1], action[2])
            elif tag in ("spend", "burst"):
                count, certainty = (1, action[1]) if tag == "spend" else action[1:]
                for _ in range(count):
                    result = policy.try_spend()
                    actual = result.reason.name
                    if result.token is not None and certainty != "MISSING":
                        policy.report_tx(result.token, TxCertainty[certainty])
                    expected = model.spend(certainty)
                    assert actual == expected, (index, action, actual, expected)
            elif tag == "advance":
                clock.advance_elapsed_us(action[1])
                model.advance(action[1])
            elif tag == "trust":
                enabled, bias = action[1:]
                supply_time()
                model.set_trust(enabled, bias)
            elif tag == "reconcile":
                faults.fail_load = action[1]
                actual = policy.reconcile(deadline_monotonic_us=deadline).reason.name
                expected = model.reconcile(action[1])
            elif tag == "restart":
                worker.finish_test()
                start()
                model.restart(enabled, bias)
            assert actual == expected, (index, action, actual, expected)
            assert policy.available_charge_us == model.available, (
                index,
                action,
                "allowance",
            )
            assert policy.total_used == model.total, (index, action, "cached total")
            # Decode observed output only; the model never sees this production representation.
            with sqlite3.connect(database) as connection:
                blob = connection.execute(
                    "SELECT state_blob FROM communicator_state"
                ).fetchone()[0]
            assert projection(decode_communicator_state_v1(blob)) == model.durable, (
                index,
                action,
                "durable ledger",
            )
    except BaseException:
        evidence = root / "failure-evidence"
        evidence.mkdir(exist_ok=True)
        for suffix in ("", "-wal", "-shm"):
            path = Path(str(database) + suffix)
            if path.exists():
                shutil.copy2(path, evidence / path.name)
        raise
    finally:
        if worker is not None:
            worker.finish_test()


# UTC errors cannot create excess elapsed credit; retain any possible use still in a physical hour.
@settings(max_examples=150, deadline=None, derandomize=True)
@given(old_error=st.integers(-39_999_999, 39_999_999),
       new_error=st.integers(-39_999_999, 39_999_999),
       elapsed=st.integers(0, 3_599_999_999),
       distance=st.integers(0, 60),
       rate=st.sampled_from([-3700, 0, 3700]))
def test_snapshot_error_reconstruction_preserves_physical_window(
    old_error, new_error, elapsed, distance, rate
):
    credit = max(0, elapsed + new_error - old_error - abs(new_error) - abs(old_error))
    assert credit <= elapsed
    charges = [Bucket(0)] * 62
    charges[61 - distance] = Bucket(67_866)
    ledger = AirtimeLedger(CommunicatorStatePolicy(), tuple(charges),
                          monotonic_us=0, elapsed_us=credit)
    latest_physical_tx = min(elapsed, 60_250_000 - distance * 60_000_000)
    physical_until_end = 3_600_000_000 + latest_physical_tx - elapsed
    if physical_until_end > 0:
        observed = ((physical_until_end - 1) * (1_000_000 + rate)) // 1_000_000
        ledger.advance(observed)
        assert ledger.total_used == 67_866


# New-bucket retention is bounded independently of uptime, and covers the latest authorized TX.
@settings(max_examples=100, deadline=None, derandomize=True)
@given(uptime=st.integers(0, 7 * 24 * 3_600_000_000),
       rate=st.sampled_from([-3700, 0, 3700]))
def test_new_bucket_physical_retention_independent_of_uptime(uptime, rate):
    now = uptime + 100
    ledger = AirtimeLedger(CommunicatorStatePolicy(), (Bucket(0),) * 62,
                          monotonic_us=now)
    ledger.set_charge(ledger.current_start, 67_866)
    deadline = ledger.retention_deadline(ledger.current_start)
    assert deadline == now + 3_673_792_925
    observed_hour = (3_600_000_000 * (1_000_000 + rate)) // 1_000_000
    ledger.advance(ledger.grant_deadline + observed_hour)
    assert ledger.total_used == 67_866
    ledger.advance(deadline - 1)
    assert ledger.total_used == 67_866
    ledger.advance(deadline)
    assert ledger.total_used == 0


# Near-end snapshots may extend restored lifetime, but cannot move the live deadline.
def test_near_end_snapshot_and_repeated_unknown_time_restarts():
    original = AirtimeLedger(CommunicatorStatePolicy(), (Bucket(0),) * 62,
                            monotonic_us=0)
    original.set_charge(0, 67_866)
    deadline = original.retention_deadline(0)
    for _ in range(4):
        original.advance(60_221_999)
        snapshot = original.snapshot_buckets()
        assert original.retention_deadline(original.current_start) == deadline
        restored = AirtimeLedger(CommunicatorStatePolicy(), snapshot, monotonic_us=0)
        assert restored.total_used == 67_866
        assert restored.retention_deadline(0) >= deadline - 60_221_999
        original = restored
        deadline = original.retention_deadline(0)
    original.advance(deadline - 1)
    assert original.total_used == 67_866
    original.advance(deadline)
    assert original.total_used == 0
