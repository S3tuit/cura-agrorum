"""Production ingress, time policy, state ownership and SQLite component boundaries."""

import pytest

from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    ProcessingResult,
    RadioState,
    RtcHealth as RH,
)
from cura_receiver.persist_queue_entities import MeasurementProfileUnitV1
from cura_receiver.ports.chrony import ChronyTrackingResult, ChronyQueryStatus as CQ
from cura_receiver.ports.ds3231 import (
    Ds3231ReadResult,
    Ds3231ReadStatus as DR,
    Ds3231WriteResult,
    Ds3231WriteDisposition as DW,
    Ds3231Failure as DF,
)
from cura_receiver.ports.kernel_clock import KernelClockResult, KernelSampleStatus as KS
from cura_receiver.protocol_ingress import ProtocolIngress, ProtocolIngressTerminalV1
from cura_receiver.runtime_time import RuntimeTime, RtcRefreshStatus
from cura_receiver.time_policy import TimePolicy
from cura_receiver.tx_airtime import AirtimeReason as R, TxCertainty
from tests.support.builders.persistence import INSTANCE
from tests.support.builders.persistence_control import state
from tests.support.builders.protocol_ingress import (
    REVIEWED_NODE_ID,
    REVIEWED_NODE_KEY,
    authenticated_frame,
    ingress_packet,
)
from tests.support.coordination.state_commit import LostStateReply
from tests.support.fakes.ds3231 import FakeDs3231Control
from tests.support.fakes.kernel_clock import FakeKernelClock


# Real airtime exhaustion preserves ingress acceptance and its original queue reservation through publication.
def test_budget_suppression_preserves_real_ingress_acceptance(airtime_component):
    policy, worker, _, clock, _ = airtime_component()
    assert policy.recover(deadline_monotonic_us=5_000_100).reason is R.STATE_READY
    assert (
        policy.acquire_grant(deadline_monotonic_us=5_000_100).reason
        is R.BUDGET_EXHAUSTED
    )
    ingress = ProtocolIngress(
        queue=worker.queue,
        monotonic_clock=clock,
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
    )
    occurrence = ingress.begin(ingress_packet(frame=authenticated_frame()))
    assert occurrence.pre_tx_profile.processing_result is ProcessingResult.ACCEPTED
    assert occurrence.pre_tx_profile.ack_selected is AckSelection.ACCEPTED
    assert policy.try_spend().token is None
    finalized = ingress.finalize(
        occurrence,
        ProtocolIngressTerminalV1(
            AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
            None,
            None,
            clock.now_monotonic_us(),
            radio_state=RadioState.RX_SINGLE,
        ),
    )
    entity = finalized.published_entity
    assert finalized.admission is None and isinstance(entity, MeasurementProfileUnitV1)
    assert entity.candidate is occurrence.candidate
    assert entity.profile.processing_result is ProcessingResult.ACCEPTED
    assert entity.profile.ack_tx_result is AckTxResult.SUPPRESSED_AIRTIME_BUDGET


# The RTC writer uses the airtime component's complete snapshot without losing or reopening its grant.
@pytest.mark.parametrize("lost_reply", [False, True])
def test_runtime_rtc_commit_preserves_airtime_owner_and_allowance(
    airtime_component, lost_reply
):
    policy, worker, _, clock, _ = airtime_component(
        initial_state=state(), utc=1_800_000_000_500_000
    )
    kernel = FakeKernelClock()
    runtime = RuntimeTime(
        receiver_instance_id=INSTANCE,
        clock=clock,
        kernel=kernel,
        queue=worker.queue,
        policy=TimePolicy(maximum_network_skew_ppb=1000),
        startup_rtc_result=Ds3231ReadResult(DR.OK, 100, 100, 1_800_000_000),
        state_owner=policy.owner,
    )
    assert runtime.airtime_correlation() is None
    kernel.results.append(
        KernelClockResult(KS.OK, 100, 100, 1_800_000_000_500_000, 5, 0x2040)
    )
    runtime.sample_network(ChronyTrackingResult(CQ.OK, 100, 100, True, True, 0, 0, 0))
    policy.update_time(
        runtime.airtime_correlation(), rtc_health=runtime.state.rtc_health
    )
    assert policy.acquire_grant(deadline_monotonic_us=5_000_100).reason is R.ALLOWED
    policy.report_tx(policy.try_spend().token, TxCertainty.UNCERTAIN)
    before = policy.state
    if lost_reply:
        channel = LostStateReply(worker.control, installed=True)
        channel.fail_load = True
        owner = CommunicatorStateOwner(control=channel, initial_state=before)
        runtime.state_owner = policy.owner = owner
    rtc = FakeDs3231Control()
    rtc.read_results.extend([Ds3231ReadResult(DR.OK, 100, 100, 1_800_000_000)] * 2)
    rtc.write_results.append(Ds3231WriteResult(DW.COMPLETED, DF.NONE, 100, 100))
    result = runtime.refresh_rtc(rtc, policy.snapshot)
    if lost_reply:
        assert result.status is RtcRefreshStatus.PERSISTENCE_FAILED
        assert policy.available_charge_us == 0 and runtime.rtc_provenance is None
        channel.fail_load = False
        assert policy.reconcile(deadline_monotonic_us=5_000_100).reason is R.ALLOWED
    else:
        assert result.status is RtcRefreshStatus.VERIFIED
    assert runtime.state_owner is policy.owner
    assert policy.state.generation == 3 and policy.state.buckets == before.buckets
    assert runtime.rtc_provenance is policy.state.rtc_provenance
    assert runtime.rtc_provenance is not None
    assert policy.available_charge_us == 7_932_134
    assert [call[0] for call in rtc.calls] == ["read", "write", "read"]
    clock.advance_elapsed_us(60_000_000)
    assert runtime.airtime_correlation() is None
    policy.update_time(runtime.airtime_correlation(), rtc_health=RH.PRESENT)
    assert policy.available_charge_us == 0
