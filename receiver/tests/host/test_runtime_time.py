from dataclasses import replace

import pytest

from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.persist_queue import PersistQueue, PersistenceAdmissionSnapshot
from cura_receiver.ports.chrony import ChronyTrackingResult, ChronyQueryStatus as Q
from cura_receiver.ports.ds3231 import Ds3231ReadResult, Ds3231ReadStatus as R
from cura_receiver.ports.kernel_clock import KernelClockResult, KernelSampleStatus as K
from cura_receiver.runtime_time import RuntimeTime
from cura_receiver.time_policy import TimePolicy, advance_clock_state
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.fakes.kernel_clock import FakeKernelClock

UTC = 1_789_200_000_500_000


def runtime(*, capacity=100, health=R.OK):
    clock = FakeOsClock(monotonic_us=1000, realtime_us=UTC)
    kernel = FakeKernelClock()
    queue = PersistQueue(capacity_entities=capacity)
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(1, E.PersistenceAdmissionState.AVAILABLE, 0)
    )
    rtc = Ds3231ReadResult(health, 0, 100, UTC // 1_000_000 if health is R.OK else None)
    value = RuntimeTime(
        receiver_instance_id=bytes.fromhex("00112233445546778899aabbccddeeff"),
        clock=clock,
        kernel=kernel,
        queue=queue,
        policy=TimePolicy(maximum_network_skew_ppb=1000),
        startup_rtc_result=rtc,
    )
    return value, clock, kernel, queue


def tracking(rt, *, correction=0, distance=0, status=Q.OK):
    now = rt.clock.now_monotonic_us()
    return ChronyTrackingResult(
        status, now, now, status is Q.OK, status is Q.OK, correction, distance, 0
    )


def sample(rt, kernel, *, status=K.OK, hook=None):
    def result():
        now = rt.clock.now_monotonic_us()
        if hook:
            hook()
        return KernelClockResult(
            status, now, now, UTC if status is K.OK else None, 5, 0x2040
        )

    kernel.results.append(result)


# Fresh kernel and tracking observations establish network time even with a missing RTC.
@pytest.mark.parametrize("health", [R.OK, R.MISSING, R.INVALID])
def test_network_independent_of_rtc(health):
    rt, _, kernel, _ = runtime(health=health)
    sample(rt, kernel)
    update = rt.sample_network(tracking(rt))
    assert update.observation.system_time_quality is E.SystemTimeQuality.NETWORK_SYNCED
    assert update.failure is None and rt.state.generation == 1
    assert rt.tracking_poll_deadline == 60_001_000


# Failed publication suppresses trust and retains a boundary ahead of every later ordinary entity.
def test_full_queue_retains_boundary():
    rt, _, kernel, queue = runtime(capacity=1)
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    sample(rt, kernel)
    update = rt.sample_network(tracking(rt))
    assert update.admission_result is E.AdmissionResult.QUEUE_FULL
    assert (
        rt.state.quality is E.SystemTimeQuality.UNTRUSTED and rt.state.generation == 2
    )
    assert rt.ordinary_admission_blocked and rt.observation_sequence == 2


# Raw kernel interference fails closed and identical repeated failures latch until success.
def test_kernel_failure_latch():
    rt, _, kernel, _ = runtime()
    sample(rt, kernel, status=K.CLOCK_INTERFERENCE)
    first = rt.sample_network(tracking(rt))
    assert first.failure.error_code is E.TimeDiagnosticErrorCode.CLOCK_INTERFERENCE
    assert first.failure.context.kernel_status_bits == 0x2040
    sample(rt, kernel, status=K.CLOCK_INTERFERENCE)
    assert rt.sample_network(tracking(rt)).failure is None
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    sample(rt, kernel, status=K.CLOCK_INTERFERENCE)
    assert rt.sample_network(tracking(rt)).failure is not None


# The public polling path must not mistake a suppressed failure for a successful read.
def test_poll_preserves_kernel_failure_latch():
    rt, _, kernel, _ = runtime()
    chrony = FakeChronyControl()
    emitted = []
    for status in (K.IO_ERROR, K.IO_ERROR, K.IO_ERROR, K.OK, K.IO_ERROR):
        sample(rt, kernel, status=status)
        chrony.tracking_results.append(tracking(rt))
        emitted.append(rt.poll_chrony(chrony).failure is not None)
    assert emitted == [True, False, False, False, True]


# Equal final quality cannot hide a generation change during the real sampling boundary.
def test_kernel_generation_aba():
    rt, _, kernel, _ = runtime()

    def aba():
        rt.state = advance_clock_state(
            rt.state,
            quality=E.SystemTimeQuality.NETWORK_SYNCED,
            rtc_health=E.RtcHealth.PRESENT,
        )
        rt.state = advance_clock_state(
            rt.state,
            quality=E.SystemTimeQuality.UNTRUSTED,
            rtc_health=E.RtcHealth.PRESENT,
        )

    sample(rt, kernel, hook=aba)
    update = rt.sample_network(tracking(rt))
    assert rt.state.quality is E.SystemTimeQuality.UNTRUSTED
    assert update.observation.sampled_at_utc_us is None and rt.state.generation == 3


# Missing the poll boundary expires trust at its exact monotonic instant without a diagnostic.
def test_poll_expiry_and_realtime_immunity():
    rt, clock, kernel, _ = runtime()
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    clock.step_realtime_us(-86_400_000_000)
    clock.advance_elapsed_us(59_999_999)
    assert rt.expire_due().observation is None
    clock.advance_elapsed_us(1)
    update = rt.expire_due()
    assert update.failure is None and not update.observation.step_discontinuity_boundary
    assert update.observation.sampled_at_monotonic_us == 60_001_000


# Valid scalar inputs whose combined error overflows produce a calculation diagnostic, never trust.
def test_calculation_range():
    rt, _, _, _ = runtime()
    result = rt.sample_network(tracking(rt, distance=(1 << 64) - 1))
    assert result.failure.error_code is E.TimeDiagnosticErrorCode.CALCULATION_RANGE


from cura_receiver.runtime_time import ChronyStepState as SS
from cura_receiver.ports.chrony import ChronyStepResult, ChronyStepDisposition as SD
from tests.support.fakes.chrony import FakeChronyControl


# A completed FIFO boundary precedes the sole command and the first later trusted observation closes its gap.
@pytest.mark.parametrize("disposition", [SD.SUBMITTED, SD.OUTCOME_UNKNOWN])
def test_step_boundary_then_wait(disposition):
    rt, clock, kernel, queue = runtime()
    chrony = FakeChronyControl()
    chrony.tracking_results.append(tracking(rt, correction=40_000_000))
    boundary = rt.poll_chrony(chrony)
    assert boundary.observation.step_discontinuity_boundary
    assert rt.step_state is SS.STEP_COMMAND_PENDING and len(chrony.calls) == 1
    chrony.step_results.append(ChronyStepResult(disposition, 1000, 1000))
    assert rt.poll_chrony(chrony).step_result.disposition is disposition
    assert rt.step_state is SS.WAITING_FOR_STABLE_TIME
    sample(rt, kernel)
    chrony.tracking_results.append(tracking(rt))
    stable = rt.poll_chrony(chrony)
    assert (
        rt.step_state is SS.IDLE
        and rt.state.quality is E.SystemTimeQuality.NETWORK_SYNCED
    )
    assert (
        stable.observation.observation_sequence
        > boundary.observation.observation_sequence
    )
    assert (stable.failure is not None) == (disposition is SD.OUTCOME_UNKNOWN)
    assert sum(kind == "step" for kind, _ in chrony.calls) == 1


# A full FIFO cannot authorize a step, and expiry of its original tracking authority prevents late submission.
def test_step_boundary_full_then_stale():
    rt, clock, kernel, queue = runtime(capacity=1)
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    chrony = FakeChronyControl()
    chrony.tracking_results.append(tracking(rt, correction=40_000_000))
    update = rt.poll_chrony(chrony)
    assert update.admission_result is E.AdmissionResult.QUEUE_FULL
    assert rt.ordinary_admission_blocked
    clock.advance_elapsed_us(1_000_000)
    rt.poll_chrony(chrony)
    assert not any(kind == "step" for kind, _ in chrony.calls)


# Unknown submission plus stable-time deadline creates one root diagnostic and requires fresh status after backoff.
def test_unknown_step_timeout_no_blind_retry():
    rt, clock, _, _ = runtime()
    chrony = FakeChronyControl()
    chrony.tracking_results.append(tracking(rt, correction=40_000_000))
    rt.poll_chrony(chrony)
    chrony.step_results.append(ChronyStepResult(SD.OUTCOME_UNKNOWN, 1000, 1000))
    rt.poll_chrony(chrony)
    clock.advance_elapsed_us(rt.step_deadline_monotonic_us - clock.now_monotonic_us())
    result = rt.poll_chrony(chrony)
    assert result.failure.error_code is E.TimeDiagnosticErrorCode.OUTCOME_UNKNOWN
    assert rt.step_state is SS.RETRY_BACKOFF
    rt.poll_chrony(chrony)
    assert sum(kind == "step" for kind, _ in chrony.calls) == 1
    clock.advance_elapsed_us(
        rt.retry_not_before_monotonic_us - clock.now_monotonic_us()
    )
    chrony.tracking_results.append(tracking(rt, status=Q.UNAVAILABLE))
    rt.poll_chrony(chrony)
    assert (
        rt.step_state is SS.RETRY_BACKOFF
        and sum(kind == "step" for kind, _ in chrony.calls) == 1
    )


# Definite rejection retains the gap and returns a single exceptional result before bounded backoff.
def test_step_rejection():
    rt, _, _, _ = runtime()
    chrony = FakeChronyControl()
    chrony.tracking_results.append(tracking(rt, correction=40_000_000))
    rt.poll_chrony(chrony)
    chrony.step_results.append(ChronyStepResult(SD.NOT_SUBMITTED, 1000, 1000))
    result = rt.poll_chrony(chrony)
    assert result.failure.error_code is E.TimeDiagnosticErrorCode.COMMAND_REJECTED
    assert (
        rt.step_state is SS.RETRY_BACKOFF
        and rt.state.quality is E.SystemTimeQuality.UNTRUSTED
    )


# Offline results are ordinary; deadline and malformed replies use distinct latched failure signatures.
def test_tracking_failure_classification():
    rt, _, _, _ = runtime()
    chrony = FakeChronyControl()
    for status, expected in [
        (Q.UNAVAILABLE, None),
        (Q.DEADLINE_EXCEEDED, E.TimeDiagnosticErrorCode.DEADLINE),
        (Q.DEADLINE_EXCEEDED, None),
        (Q.INVALID_RESPONSE, E.TimeDiagnosticErrorCode.INVALID_RESPONSE),
    ]:
        chrony.tracking_results.append(tracking(rt, status=status))
        result = rt.poll_chrony(chrony)
        assert (result.failure.error_code if result.failure else None) is expected


from cura_receiver.runtime_time import RtcRefreshStatus as RS
from cura_receiver.ports.ds3231 import (
    Ds3231WriteResult,
    Ds3231WriteDisposition as W,
    Ds3231Failure as RF,
)
from cura_receiver.generated.receiver_entities_generated import RtcProvenanceV1
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as CD,
)
from tests.support.fakes.ds3231 import FakeDs3231Control
from tests.support.builders.persistence_control import synthetic
from tests.support.coordination.persistence_worker import (
    CheckedPersistenceWorker,
    prepare_worker_files,
)


def prepared_rtc_runtime(tmp_path):
    rt, clock, kernel, _ = runtime()
    database, configuration, boot = prepare_worker_files(tmp_path)
    worker = CheckedPersistenceWorker(
        instance=ReceiverInstanceStart(rt.instance, 0),
        database_path=database,
        configuration_path=configuration,
        boot_id_path=boot,
        clock=clock,
        minimum_free_bytes=0,
    )
    worker.start()
    assert worker.wait_started(deadline_monotonic_us=5_001_000) is not None
    # A reviewed existing synthetic ledger is fixture input, not runtime time policy.
    initial = synthetic()
    initial = replace(
        initial,
        airtime_snapshot_utc_us=UTC,
        buckets=tuple(
            (
                replace(b, expires_at_utc_us=b.expires_at_utc_us + UTC)
                if b.charged_airtime_us
                else b
            )
            for b in initial.buckets
        ),
        rtc_provenance=RtcProvenanceV1(rt.instance, UTC, UTC, 3_000_000, 10),
    )
    assert (
        worker.control.commit_communicator_state(
            initial, deadline_monotonic_us=5_001_000
        ).disposition
        is CD.COMMITTED
    )
    rt.durable_state = worker.control.load_communicator_state(
        deadline_monotonic_us=5_001_000
    ).state
    rt.rtc_provenance = rt.durable_state.rtc_provenance
    rt.queue = worker.queue
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    rtc = FakeDs3231Control()
    # Each refresh starts with a successful communication check before invalidation.
    rtc.read_results.append(Ds3231ReadResult(R.OK, 1000, 1000, UTC // 1_000_000))

    def snapshot(**values):
        return replace(
            values["previous_state"],
            generation=values["previous_state"].generation + 1,
            rtc_provenance=values["provenance"],
            airtime_snapshot_utc_us=values["snapshot_utc_us"],
        )

    return rt, clock, kernel, rtc, worker, snapshot, database


@pytest.fixture
def rtc_runtime(tmp_path):
    resources = prepared_rtc_runtime(tmp_path)
    try:
        yield resources
    finally:
        resources[4].finish_test()


# A new write observes durable invalidation; matching read-back and exact state acknowledgement restore proof.
@pytest.mark.parametrize("disposition", [W.COMPLETED, W.OUTCOME_UNKNOWN])
def test_rtc_refresh_durable_order(rtc_runtime, disposition):
    rt, clock, _, rtc, worker, snapshot, _ = rtc_runtime

    def write():
        loaded = worker.control.load_communicator_state(deadline_monotonic_us=5_001_000)
        assert loaded.state.generation == 2 and loaded.state.rtc_provenance is None
        return Ds3231WriteResult(
            disposition,
            RF.NONE if disposition is W.COMPLETED else RF.IO_ERROR,
            1000,
            1000,
        )

    rtc.write_results.append(write)
    rtc.read_results.append(Ds3231ReadResult(R.OK, 1000, 1000, UTC // 1_000_000))
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is RS.VERIFIED
    assert rt.durable_state.generation == 3 and rt.rtc_provenance is not None
    assert (result.failure is not None) == (disposition is W.OUTCOME_UNKNOWN)
    assert (
        worker.control.load_communicator_state(deadline_monotonic_us=5_001_000).state
        == rt.durable_state
    )
    assert [call[0] for call in rtc.calls] == ["read", "write", "read"]


# Every unverified read-back leaves the acknowledged invalidation in SQLite and cannot resurrect old proof.
@pytest.mark.parametrize(
    "status", [R.MISSING, R.INVALID, R.IO_ERROR, R.DEADLINE_EXCEEDED]
)
def test_rtc_failed_readback(rtc_runtime, status):
    rt, clock, _, rtc, worker, snapshot, _ = rtc_runtime
    rtc.write_results.append(
        Ds3231WriteResult(W.OUTCOME_UNKNOWN, RF.IO_ERROR, 1000, 1000)
    )

    def failed_read():
        if status is not R.INVALID:
            clock.advance_elapsed_us(3_000_000)
        return Ds3231ReadResult(status, 1000, clock.now_monotonic_us())

    rtc.read_results.append(failed_read)
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is RS.READBACK_FAILED
    assert result.failure.error_code is E.TimeDiagnosticErrorCode.OUTCOME_UNKNOWN
    assert (
        result.failure.context.secondary_status.value
        == E.Ds3231ReadStatus[
            "INVALID" if status is R.INVALID else "DEADLINE_EXCEEDED"
        ].value
    )
    assert (
        worker.control.load_communicator_state(
            deadline_monotonic_us=5_001_000
        ).state.rtc_provenance
        is None
    )


# A mismatching whole-second read-back has one diagnostic and never installs provenance.
def test_rtc_readback_mismatch(rtc_runtime):
    rt, _, _, rtc, worker, snapshot, _ = rtc_runtime
    rtc.write_results.append(Ds3231WriteResult(W.COMPLETED, RF.NONE, 1000, 1000))
    rtc.read_results.append(Ds3231ReadResult(R.OK, 1000, 1000, UTC // 1_000_000 + 2))
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is RS.READBACK_MISMATCH
    assert result.failure.error_code is E.TimeDiagnosticErrorCode.RTC_READBACK_MISMATCH
    assert rt.rtc_provenance is None and rt.durable_state.generation == 2


# A generation change after writing still requires read-back and suppresses the final provenance commit.
def test_rtc_generation_invalidated_during_write(rtc_runtime):
    rt, _, _, rtc, worker, snapshot, _ = rtc_runtime

    def changed():
        rt.state = advance_clock_state(
            rt.state,
            quality=rt.state.quality,
            rtc_health=rt.state.rtc_health,
            tracking_processed=True,
        )
        return Ds3231WriteResult(W.COMPLETED, RF.NONE, 1000, 1000)

    rtc.write_results.append(changed)
    rtc.read_results.append(Ds3231ReadResult(R.OK, 1000, 1000, UTC // 1_000_000))
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is RS.TRUST_INVALIDATED and result.failure is None
    assert [call[0] for call in rtc.calls] == ["read", "write", "read"]
    assert rt.durable_state.generation == 2 and rt.rtc_provenance is None


# Offline startup uses fresh direct RTC evidence and old durable proof, independent of plausible Linux UTC.
def test_offline_direct_holdover_and_unproven_startup():
    for proven in (False, True):
        rt, clock, _, _ = runtime()
        rt.rtc_provenance = (
            RtcProvenanceV1(
                rt.instance, UTC - 3_600_000_000, UTC - 3_600_000_000, 3_000_000, 10
            )
            if proven
            else None
        )
        rtc = FakeDs3231Control()
        rtc.read_results.append(Ds3231ReadResult(R.OK, 1000, 1000, UTC // 1_000_000))
        clock.step_realtime_us(86400_000_000)
        result = rt.observe_rtc(rtc)
        assert (rt.state.quality is E.SystemTimeQuality.RTC_HOLDOVER) == proven
        assert result.failure is None and all(call[0] == "read" for call in rtc.calls)


# A valid network clock remains valid after an RTC failure while its health-observation handoff blocks admission.
def test_online_rtc_missing_preserves_network():
    rt, clock, kernel, _ = runtime()
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    rtc = FakeDs3231Control()

    def missing():
        clock.advance_elapsed_us(3_000_000)
        return Ds3231ReadResult(R.MISSING, 1000, clock.now_monotonic_us())

    rtc.read_results.append(missing)
    result = rt.observe_rtc(rtc)
    assert (
        result.failure.error_code is E.TimeDiagnosticErrorCode.DEADLINE
        and rt.state.quality is E.SystemTimeQuality.NETWORK_SYNCED
    )
    assert rt.state.rtc_health is E.RtcHealth.MISSING and rt.ordinary_admission_blocked
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    assert not rt.ordinary_admission_blocked


from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitFailureKind as CF,
)


# Unknown invalidation/final commits reconcile through real serialized loads before any next action.
@pytest.mark.parametrize("unknown_commit", [1, 2])
def test_rtc_unknown_commit_reconciliation(rtc_runtime, unknown_commit):
    rt, _, _, rtc, worker, snapshot, _ = rtc_runtime

    class LostReply:
        count = 0

        def commit_communicator_state(self, *args, **kwargs):
            self.count += 1
            result = worker.control.commit_communicator_state(*args, **kwargs)
            if self.count == unknown_commit:
                return replace(
                    result,
                    disposition=CD.OUTCOME_UNKNOWN,
                    failure_kind=CF.DEADLINE_EXCEEDED,
                    operation=E.DiagnosticOperation.WRITE,
                )
            return result

        def load_communicator_state(self, **kwargs):
            return worker.control.load_communicator_state(**kwargs)

    rtc.write_results.append(Ds3231WriteResult(W.COMPLETED, RF.NONE, 1000, 1000))
    rtc.read_results.append(Ds3231ReadResult(R.OK, 1000, 1000, UTC // 1_000_000))
    result = rt.refresh_rtc(rtc, LostReply(), snapshot)
    assert result.status is RS.VERIFIED and rt.durable_state.generation == 3
    assert result.failure is None


# Loss of a complete-state owner defers refresh after preflight, without a write or changing old proof.
def test_state_handoff_unavailable(rtc_runtime):
    rt, _, _, rtc, worker, _, _ = rtc_runtime
    result = rt.refresh_rtc(rtc, worker.control, lambda **kw: None)
    assert result.status is RS.DEFERRED and [c[0] for c in rtc.calls] == ["read"]
    assert rt.durable_state.generation == 1 and rt.rtc_provenance is not None


# Before the final submission a callback generation change cancels the new proof despite a matching read-back.
def test_generation_change_before_provenance_commit(rtc_runtime):
    rt, _, _, rtc, worker, snapshot, _ = rtc_runtime

    def changed(**values):
        if values["provenance"] is not None:
            rt.state = advance_clock_state(
                rt.state,
                quality=rt.state.quality,
                rtc_health=rt.state.rtc_health,
                tracking_processed=True,
            )
        return snapshot(**values)

    rtc.write_results.append(Ds3231WriteResult(W.COMPLETED, RF.NONE, 1000, 1000))
    rtc.read_results.append(Ds3231ReadResult(R.OK, 1000, 1000, UTC // 1_000_000))
    result = rt.refresh_rtc(rtc, worker.control, changed)
    assert result.status is RS.TRUST_INVALIDATED
    assert rt.durable_state.generation == 2 and rt.rtc_provenance is None


def rtc_crash_child(root, milestone, pipe):
    from pathlib import Path

    rt, _, _, rtc, worker, snapshot, _ = prepared_rtc_runtime(Path(root))

    def ready(point):
        if point == milestone:
            pipe.send(point)
            pipe.recv()  # Named boundary: parent kills this actual runtime process.

    class Control:
        def commit_communicator_state(self, state, **kwargs):
            result = worker.control.commit_communicator_state(state, **kwargs)
            if state.rtc_provenance is not None:
                ready("verified_commit")
            return result

        def load_communicator_state(self, **kwargs):
            return worker.control.load_communicator_state(**kwargs)

    def build(**values):
        ready(
            "before_invalidation"
            if values["provenance"] is None
            else "before_verification_commit"
        )
        return snapshot(**values)

    def write():
        ready("invalidated")
        return Ds3231WriteResult(W.COMPLETED, RF.NONE, 1000, 1000)

    def read():
        ready("written")
        return Ds3231ReadResult(R.OK, 1000, 1000, UTC // 1_000_000)

    rtc.write_results.append(write)
    rtc.read_results.append(read)
    rt.refresh_rtc(rtc, Control(), build)
    ready("acknowledged")
    worker.finish_test()


# SIGKILL at real runtime/SQLite boundaries leaves old, absent or new proof exactly as durable ordering requires.
@pytest.mark.parametrize(
    "milestone,generation,proven",
    [
        ("before_invalidation", 1, True),
        ("invalidated", 2, False),
        ("written", 2, False),
        ("before_verification_commit", 2, False),
        ("verified_commit", 3, True),
        ("acknowledged", 3, True),
    ],
)
def test_rtc_process_crash_restart(tmp_path, milestone, generation, proven):
    import multiprocessing
    import shutil

    ctx = multiprocessing.get_context("spawn")
    parent, child = ctx.Pipe()
    process = ctx.Process(
        target=rtc_crash_child, args=(str(tmp_path), milestone, child)
    )
    process.start()
    child.close()
    try:
        assert parent.poll(10), "runtime child did not reach the named boundary"
        assert parent.recv() == milestone
        process.kill()
        process.join(5)
        assert not process.is_alive() and process.exitcode == -9
        evidence = tmp_path / "pre-recovery"
        evidence.mkdir()
        for name in ("worker.db", "worker.db-wal", "worker.db-shm"):
            path = tmp_path / name
            if path.exists():
                shutil.copy2(path, evidence / name)
        worker = CheckedPersistenceWorker(
            instance=ReceiverInstanceStart(
                bytes.fromhex("10112233445546778899aabbccddeeff"), 2000
            ),
            database_path=tmp_path / "worker.db",
            configuration_path=tmp_path / "test-group.json",
            boot_id_path=tmp_path / "boot-id",
            clock=FakeOsClock(monotonic_us=2000),
            minimum_free_bytes=0,
        )
        worker.start()
        try:
            assert (
                worker.wait_started(deadline_monotonic_us=5_002_000).database_failure
                is None
            )
            state = worker.control.load_communicator_state(
                deadline_monotonic_us=5_002_000
            ).state
            assert (
                state.generation == generation
                and (state.rtc_provenance is not None) == proven
            )
        finally:
            worker.finish_test()
    finally:
        if process.is_alive():
            process.kill()
        process.join(5)
        parent.close()


# The strict RTC source threshold is rechecked after durable invalidation, with one microsecond sufficient to expire equality.
def test_source_expires_after_invalidation(rtc_runtime):
    rt, clock, kernel, rtc, worker, snapshot, _ = rtc_runtime
    sample(rt, kernel)
    rt.sample_network(tracking(rt, correction=4_000_000))

    class DelayedAcknowledgement:
        def commit_communicator_state(self, *args, **kwargs):
            result = worker.control.commit_communicator_state(*args, **kwargs)
            clock.advance_elapsed_us(1)
            return result

    result = rt.refresh_rtc(rtc, DelayedAcknowledgement(), snapshot)
    assert result.status is RS.TRUST_INVALIDATED and [c[0] for c in rtc.calls] == ["read"]
    assert rt.durable_state.rtc_provenance is None
    assert rt.rtc_trust_invalidated_count == 0  # No physical write occurred.


# The exact one-second read-back tolerance is inclusive on both sides; the adjacent microsecond is rejected.
@pytest.mark.parametrize(
    "difference,expected",
    [
        (1_000_000, RS.VERIFIED),
        (-1_000_000, RS.VERIFIED),
        (1_000_001, RS.READBACK_MISMATCH),
        (-1_000_001, RS.READBACK_MISMATCH),
    ],
)
def test_readback_exact_margin(rtc_runtime, difference, expected):
    rt, _, kernel, rtc, worker, snapshot, _ = rtc_runtime
    kernel.results.append(
        KernelClockResult(K.OK, 1000, 1000, UTC - difference, 5, 0x2040)
    )
    rt.sample_network(tracking(rt))
    rtc.write_results.append(Ds3231WriteResult(W.COMPLETED, RF.NONE, 1000, 1000))
    rtc.read_results.append(Ds3231ReadResult(R.OK, 1000, 1000, UTC // 1_000_000))
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is expected
    assert rt.rtc_readback_verified_count == int(expected is RS.VERIFIED)


# A read-back health recovery may advance its own generation and still finish the same verified episode.
def test_refresh_adopts_own_health_transition(rtc_runtime):
    rt, _, _, rtc, worker, snapshot, _ = rtc_runtime
    rt.state = advance_clock_state(
        rt.state, quality=rt.state.quality, rtc_health=E.RtcHealth.MISSING
    )
    rt.sample = replace(rt.sample, generation=rt.state.generation)
    rtc.write_results.append(Ds3231WriteResult(W.COMPLETED, RF.NONE, 1000, 1000))
    rtc.read_results.append(Ds3231ReadResult(R.OK, 1000, 1000, UTC // 1_000_000))
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is RS.VERIFIED and rt.state.rtc_health is E.RtcHealth.PRESENT
    assert rt.ordinary_admission_blocked


# Generation changes during read-back cannot be adopted as the episode's own health update.
def test_generation_change_during_readback(rtc_runtime):
    rt, _, _, rtc, worker, snapshot, _ = rtc_runtime
    rtc.write_results.append(Ds3231WriteResult(W.COMPLETED, RF.NONE, 1000, 1000))

    def changed():
        rt.state = advance_clock_state(
            rt.state,
            quality=rt.state.quality,
            rtc_health=rt.state.rtc_health,
            tracking_processed=True,
        )
        return Ds3231ReadResult(R.OK, 1000, 1000, UTC // 1_000_000)

    rtc.read_results.append(changed)
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is RS.TRUST_INVALIDATED and rt.rtc_trust_invalidated_count == 1
    assert rt.durable_state.rtc_provenance is None


# Every definitely-not-applied failure ends without read-back or new provenance.
@pytest.mark.parametrize("failure", [RF.MISSING, RF.IO_ERROR, RF.DEADLINE_EXCEEDED])
def test_not_applied_never_reads_back(rtc_runtime, failure):
    rt, _, _, rtc, worker, snapshot, _ = rtc_runtime
    rtc.write_results.append(Ds3231WriteResult(W.NOT_APPLIED, failure, 1000, 1000))
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is RS.WRITE_FAILED and [call[0] for call in rtc.calls] == [
        "read", "write"
    ]
    assert rt.rtc_write_counts[W.NOT_APPLIED] == 1 and rt.rtc_provenance is None
    assert (result.failure is None) == (failure is RF.MISSING)


# A missing durable state never causes a time-only component to invent a ledger or perform a write.
def test_missing_state_defers_refresh():
    rt, _, kernel, _ = runtime()
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    rtc = FakeDs3231Control()
    result = rt.refresh_rtc(
        rtc, None, lambda **kw: pytest.fail("unexpected state construction")
    )
    assert result.status is RS.DEFERRED and not rtc.calls


# The closed diagnostic factory has no queue access; failed caller admission cannot undo the prior time transition.
def test_diagnostic_admission_independent():
    from cura_receiver.time_diagnostics import time_diagnostic
    from cura_receiver.persist_queue_entities import DIAGNOSTIC_V1_SPEC

    rt, _, kernel, queue = runtime(capacity=1)
    sample(rt, kernel, status=K.CLOCK_INTERFERENCE)
    result = rt.sample_network(tracking(rt))
    diagnostic = time_diagnostic(
        result.failure, receiver_instance_id=rt.instance, diagnostic_sequence=1
    )
    assert (
        queue.try_reserve_one(DIAGNOSTIC_V1_SPEC).status is E.AdmissionResult.QUEUE_FULL
    )
    assert (
        diagnostic.context_length == 80
        and rt.state.quality is E.SystemTimeQuality.UNTRUSTED
    )
    assert result.observation.sampled_at_utc_us is None


from hypothesis import given, settings, strategies as st


def network_oracle(previous, error, available):
    # Primitive reviewed model: the strict UTC budget separately overrides retention at forty seconds.
    if not available or error >= 40_000_000:
        return "UNTRUSTED"
    if error <= 35_000_000:
        return "NETWORK_SYNCED"
    return previous


# Reviewed primitive model examples establish entry, retention, loss and ordinary budget expiry.
def test_network_oracle_examples():
    assert network_oracle("UNTRUSTED", 35_000_000, True) == "NETWORK_SYNCED"
    assert network_oracle("UNTRUSTED", 35_000_001, True) == "UNTRUSTED"
    assert network_oracle("NETWORK_SYNCED", 39_999_999, True) == "NETWORK_SYNCED"
    assert network_oracle("NETWORK_SYNCED", 40_000_000, True) == "UNTRUSTED"


# Generated tracking/clock schedules compare live policy with primitive expectations and prohibit accidental steps.
@given(
    st.lists(
        st.tuples(
            st.sampled_from(
                [1_000_000, 35_000_000, 35_000_001, 39_999_999, 40_000_000]
            ),
            st.booleans(),
        ),
        min_size=1,
        max_size=40,
    )
)
@settings(max_examples=40, derandomize=True, deadline=None)
def test_runtime_network_sequences(inputs):
    rt, _, kernel, _ = runtime()
    expected = "UNTRUSTED"
    for error, available in inputs:
        next_expected = network_oracle(expected, error, available)
        if available and (error <= 35_000_000 or expected == "NETWORK_SYNCED"):
            sample(rt, kernel)
        result = rt.sample_network(
            tracking(
                rt,
                correction=error - 1_000_000 if available else 0,
                status=Q.OK if available else Q.UNAVAILABLE,
            )
        )
        assert rt.state.quality.name == next_expected
        assert (
            not result.observation.step_discontinuity_boundary
            and result.failure is None
        )
        expected = next_expected


from cura_receiver.runtime_time import recover_rtc_read


def queue_timed_rtc_read(rtc, clock, status, elapsed_us, *, os_errno=None, seconds=None):
    def read():
        start = clock.now_monotonic_us()
        clock.advance_elapsed_us(elapsed_us)
        return Ds3231ReadResult(
            status, start, clock.now_monotonic_us(),
            (UTC // 1_000_000 if seconds is None else seconds) if status is R.OK else None,
            os_errno,
        )
    rtc.read_results.append(read)


# The observed NACK/timeout/EIO/success sequence recovers using only the final read bracket.
def test_rtc_read_recovery_transport_sequence():
    clock = FakeOsClock(monotonic_us=0)
    rtc = FakeDs3231Control()
    queue_timed_rtc_read(rtc, clock, R.MISSING, 3_000, os_errno=121)
    queue_timed_rtc_read(rtc, clock, R.IO_ERROR, 1_000_000, os_errno=110)
    queue_timed_rtc_read(rtc, clock, R.IO_ERROR, 2_000, os_errno=5)
    queue_timed_rtc_read(rtc, clock, R.OK, 4_000)
    result = recover_rtc_read(
        rtc, clock=clock, deadline_monotonic_us=3_000_000, attempt_budget_us=5_000_000
    )
    assert result.attempts == 4 and result.result.status is R.OK
    assert result.operation_started_at_monotonic_us == 0
    assert result.operation_finished_at_monotonic_us == 1_009_000
    assert result.result.operation_started_at_monotonic_us == 1_005_000
    assert result.result.operation_finished_at_monotonic_us == 1_009_000
    assert result.first_failure.os_errno == 110
    assert rtc.calls == [
        ('read', 5_000_000), ('read', 5_003_000),
        ('read', 6_003_000), ('read', 6_005_000),
    ]


# Equality and an overrun reject an otherwise valid final read; no fifth-second retry is smuggled in.
@pytest.mark.parametrize('last_duration,status', [
    (99_999, R.OK), (100_000, R.DEADLINE_EXCEEDED), (1_000_000, R.DEADLINE_EXCEEDED),
])
def test_rtc_recovery_window_final_inflight(last_duration, status):
    clock = FakeOsClock(monotonic_us=0)
    rtc = FakeDs3231Control()
    queue_timed_rtc_read(rtc, clock, R.IO_ERROR, 2_900_000, os_errno=110)
    queue_timed_rtc_read(rtc, clock, R.OK, last_duration)
    result = recover_rtc_read(
        rtc, clock=clock, deadline_monotonic_us=3_000_000, attempt_budget_us=5_000_000
    )
    assert result.result.status is status and result.attempts == 2
    assert (result.result.rtc_utc_s is not None) == (status is R.OK)
    assert result.operation_finished_at_monotonic_us == 2_900_000 + last_duration
    assert result.first_failure.os_errno == 110


# An invalid RTC is operator-recovery input, while expiry before entry performs no I/O.
@pytest.mark.parametrize('expired', [False, True])
def test_rtc_recovery_invalid_and_expired(expired):
    clock = FakeOsClock(monotonic_us=0)
    rtc = FakeDs3231Control()
    queue_timed_rtc_read(rtc, clock, R.INVALID, 1_000, os_errno=22)
    result = recover_rtc_read(
        rtc, clock=clock, deadline_monotonic_us=0 if expired else 3_000_000,
        attempt_budget_us=5_000_000,
    )
    assert result.result.status is (R.DEADLINE_EXCEEDED if expired else R.INVALID)
    assert result.attempts == (0 if expired else 1)


# A failed pre-write read leaves the prior durable proof intact and submits no helper or commit.
@pytest.mark.parametrize('status,duration', [(R.INVALID, 1_000), (R.IO_ERROR, 3_000_000)])
def test_prewrite_read_failure_preserves_provenance(rtc_runtime, status, duration):
    rt, clock, _, rtc, worker, snapshot, _ = rtc_runtime
    rtc.read_results.clear()
    before = rt.durable_state
    queue_timed_rtc_read(rtc, clock, status, duration)
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is RS.PREWRITE_READ_FAILED
    assert result.write_result is None and result.commit_result is None
    assert result.prewrite_read_result.status is (
        R.INVALID if status is R.INVALID else R.DEADLINE_EXCEEDED
    )
    assert [call[0] for call in rtc.calls] == ['read']
    assert rt.durable_state == before and rt.rtc_provenance == before.rtc_provenance
    assert rt.state.quality is E.SystemTimeQuality.NETWORK_SYNCED


# Recovery advances the write's UTC derivation and still makes exactly one write after durable invalidation.
def test_prewrite_recovery_derives_fresh_time(rtc_runtime):
    rt, clock, _, rtc, worker, snapshot, _ = rtc_runtime
    rtc.read_results.clear()
    queue_timed_rtc_read(rtc, clock, R.IO_ERROR, 1_000_000, os_errno=110)
    queue_timed_rtc_read(rtc, clock, R.OK, 500_000)
    queue_timed_rtc_read(rtc, clock, R.OK, 1_000, seconds=UTC // 1_000_000 + 2)

    def write():
        assert rt.durable_state.rtc_provenance is None
        now = clock.now_monotonic_us()
        return Ds3231WriteResult(W.COMPLETED, RF.NONE, now, now)

    rtc.write_results.append(write)
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is RS.VERIFIED
    assert [call[0] for call in rtc.calls] == ['read', 'read', 'write', 'read']
    assert rtc.calls[2][1] == UTC // 1_000_000 + 2
    assert result.failure.error_code is E.TimeDiagnosticErrorCode.IO
    assert result.failure.context.os_errno == 110
    assert result.failure.context.secondary_status.value == E.Ds3231ReadStatus.OK.value


# Source expiry or an ABA generation change during successful preflight cancels before invalidation/write.
@pytest.mark.parametrize('change', ['source', 'generation'])
def test_prewrite_recovery_rechecks_authority(rtc_runtime, change):
    rt, clock, kernel, rtc, worker, snapshot, _ = rtc_runtime
    rtc.read_results.clear()
    if change == 'source':
        sample(rt, kernel)
        rt.sample_network(tracking(rt, correction=4_000_000))

    def read():
        clock.advance_elapsed_us(1)
        if change == 'generation':
            rt.state = advance_clock_state(
                rt.state, quality=rt.state.quality,
                rtc_health=rt.state.rtc_health, tracking_processed=True,
            )
        return Ds3231ReadResult(R.OK, 1000, 1001, UTC // 1_000_000)

    rtc.read_results.append(read)
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is RS.TRUST_INVALIDATED
    assert rt.durable_state.generation == 1 and rt.rtc_provenance is not None
    assert [call[0] for call in rtc.calls] == ['read']


# A recovered direct read reports its first exception and resets the latch for the next independent failure.
def test_observe_rtc_recovered_failure_latch():
    rt, clock, _, _ = runtime()
    for _ in range(2):
        rtc = FakeDs3231Control()
        queue_timed_rtc_read(rtc, clock, R.IO_ERROR, 1_000, os_errno=5)
        queue_timed_rtc_read(rtc, clock, R.OK, 1_000)
        update = rt.observe_rtc(rtc)
        assert update.failure.error_code is E.TimeDiagnosticErrorCode.IO
        assert update.failure.context.secondary_status.value == E.Ds3231ReadStatus.OK.value
        assert rt.state.rtc_health is E.RtcHealth.PRESENT


# An uncertain write may be verified after read recovery, but it is never submitted a second time.
def test_unknown_write_recovers_readback_without_rewrite(rtc_runtime):
    from cura_receiver.time_diagnostics import TimeFlags

    rt, clock, _, rtc, worker, snapshot, _ = rtc_runtime
    rtc.write_results.append(Ds3231WriteResult(W.OUTCOME_UNKNOWN, RF.DEADLINE_EXCEEDED, 1000, 1000))
    queue_timed_rtc_read(rtc, clock, R.IO_ERROR, 1_000_000, os_errno=110)
    queue_timed_rtc_read(rtc, clock, R.OK, 1_000, seconds=UTC // 1_000_000 + 1)
    result = rt.refresh_rtc(rtc, worker.control, snapshot)
    assert result.status is RS.VERIFIED and rt.rtc_provenance is not None
    assert [call[0] for call in rtc.calls] == ['read', 'write', 'read', 'read']
    assert result.failure.error_code is E.TimeDiagnosticErrorCode.OUTCOME_UNKNOWN
    assert result.failure.context.flags & int(TimeFlags.COMMAND_MAY_HAVE_APPLIED)
    assert result.failure.context.secondary_status.value == E.Ds3231ReadStatus.OK.value


# Realtime steps during a recovery attempt cannot change either monotonic deadline.
def test_recovery_ignores_realtime_steps():
    clock = FakeOsClock(monotonic_us=0, realtime_us=UTC)
    rtc = FakeDs3231Control()

    def failed():
        clock.step_realtime_us(-86_400_000_000)
        clock.advance_elapsed_us(1_000_000)
        return Ds3231ReadResult(R.IO_ERROR, 0, 1_000_000, os_errno=5)

    rtc.read_results.append(failed)
    queue_timed_rtc_read(rtc, clock, R.OK, 1_000)
    result = recover_rtc_read(rtc, clock=clock, deadline_monotonic_us=3_000_000, attempt_budget_us=5_000_000)
    assert result.result.status is R.OK
    assert rtc.calls == [('read', 5_000_000), ('read', 6_000_000)]
    assert result.operation_finished_at_monotonic_us == 1_001_000
