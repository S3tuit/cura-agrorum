from __future__ import annotations

import pytest

from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    PersistenceAdmissionState,
    ProcessingResult,
)
from cura_receiver.persist_queue import (
    PersistQueue,
    PersistQueueBatchDisposition,
    PersistenceAdmissionSnapshot,
)
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.protocol_ingress import (
    ProtocolIngress,
    ProtocolIngressTerminalV1,
)
from tests.support.builders.protocol_ingress import (
    REVIEWED_ACCEPTED_ACK,
    REVIEWED_BACKLOG_FRAME,
    REVIEWED_CURRENT_FRAME,
    REVIEWED_NODE_ID,
    REVIEWED_NODE_KEY,
    REVIEWED_SAMPLE_ID,
    ingress_packet,
)


pytestmark = pytest.mark.hardware


def _context() -> tuple[PersistQueue, ProtocolIngress, LinuxOsClock]:
    queue = PersistQueue(capacity_entities=1)
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=1,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=1,
        )
    )
    clock = LinuxOsClock()
    ingress = ProtocolIngress(
        queue=queue,
        monotonic_clock=clock,
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
    )
    return queue, ingress, clock


def _packet(frame: bytes, clock: LinuxOsClock):
    copied_at = clock.now_monotonic_us()
    return ingress_packet(
        frame=frame,
        received_at_monotonic_us=copied_at,
        t1_handler_started_monotonic_us=copied_at,
        t2_packet_copied_monotonic_us=copied_at,
    )


# Processes both reviewed reading domains using the target Python and crypto stack.
@pytest.mark.parametrize("frame", (REVIEWED_CURRENT_FRAME, REVIEWED_BACKLOG_FRAME))
def test_target_runtime_processes_reviewed_readings_without_radio(frame: bytes) -> None:
    queue, ingress, clock = _context()

    decision = ingress.begin(_packet(frame, clock))

    assert decision.pre_tx_profile.processing_result is ProcessingResult.ACCEPTED
    assert decision.pre_tx_profile.ack_selected is AckSelection.ACCEPTED
    assert decision.pre_tx_profile.ack_frame == REVIEWED_ACCEPTED_ACK
    assert decision.pre_tx_profile.decoded_sample_id == REVIEWED_SAMPLE_ID
    finalized = ingress.finalize(
        decision,
        ProtocolIngressTerminalV1(
            ack_tx_result=AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
            t4_set_tx_attempted_monotonic_us=None,
            t5_tx_done_monotonic_us=None,
            t6_set_rx_issued_monotonic_us=clock.now_monotonic_us(),
        ),
    )
    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    assert lease.entries[0].entity is finalized.published_entity
    lease.acknowledge_durable((PersistQueueBatchDisposition.SQLITE_COMMITTED,))


# Rejects a reviewed-frame tag mutation on target without touching radio hardware.
def test_target_runtime_rejects_invalid_authenticated_frame_without_radio() -> None:
    queue, ingress, clock = _context()
    invalid = bytearray(REVIEWED_CURRENT_FRAME)
    invalid[-1] ^= 1

    decision = ingress.begin(_packet(bytes(invalid), clock))

    assert decision.pre_tx_profile.processing_result is (
        ProcessingResult.AUTHENTICATION_FAILED
    )
    assert decision.pre_tx_profile.ack_selected is AckSelection.NONE
    assert decision.pre_tx_profile.ack_frame is None
    assert decision.candidate is None
    finalized = ingress.finalize(
        decision,
        ProtocolIngressTerminalV1(
            ack_tx_result=AckTxResult.NOT_APPLICABLE,
            t4_set_tx_attempted_monotonic_us=None,
            t5_tx_done_monotonic_us=None,
            t6_set_rx_issued_monotonic_us=clock.now_monotonic_us(),
        ),
    )
    assert finalized.published_entity is not None
    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    assert lease.entries[0].entity is finalized.published_entity
