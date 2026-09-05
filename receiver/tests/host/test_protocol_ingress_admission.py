from __future__ import annotations

import pytest

from cura_receiver.generated import protocol_v2_lora_generated as protocol
from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    AdmissionResult,
    PersistenceAdmissionState,
    PersistQueueEntityKind,
    ProcessingResult,
    RadioState,
)
from cura_receiver.persist_queue import (
    PersistQueue,
    PersistQueueBatchDisposition,
    PersistenceAdmissionSnapshot,
)
from cura_receiver.persist_queue_entities import (
    MEASUREMENT_PROFILE_V1_SPEC,
    PROFILE_ONLY_V1_SPEC,
    MeasurementProfileUnitV1,
    ProfileOnlyUnitV1,
)
from cura_receiver.protocol_ingress import (
    ProtocolIngress,
    ProtocolIngressPacketV1,
    ProtocolIngressTerminalV1,
)
from tests.support.builders.protocol_ingress import (
    REVIEWED_NODE_ID as NODE_ID,
    REVIEWED_NODE_KEY as NODE_KEY,
    REVIEWED_RETRY_LATER_ACK as EXPECTED_RETRY_ACK,
    authenticated_frame as _frame,
    ingress_packet as _packet,
)
from tests.support.fakes.os_clock import FakeOsClock


def _publish_state(
    queue: PersistQueue,
    state: PersistenceAdmissionState,
    *,
    generation: int = 1,
) -> None:
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=generation,
            state=state,
            changed_at_monotonic_us=generation,
        )
    )


def _ingress(queue: PersistQueue) -> ProtocolIngress:
    return ProtocolIngress(
        queue=queue,
        monotonic_clock=FakeOsClock(monotonic_us=20, realtime_us=0),
        auth_node_keys={NODE_ID: NODE_KEY},
    )


def _terminal(result: AckTxResult) -> ProtocolIngressTerminalV1:
    if result is AckTxResult.NOT_APPLICABLE:
        return ProtocolIngressTerminalV1(result, None, None, 23, radio_state=RadioState.RX_SINGLE)
    if result is AckTxResult.SUPPRESSED_AIRTIME_BUDGET:
        return ProtocolIngressTerminalV1(result, None, None, 23, radio_state=RadioState.RX_SINGLE)
    return ProtocolIngressTerminalV1(result, 21, 22, 23, radio_state=RadioState.RX_SINGLE)


# Reserves the exact pair before acceptance and publishes it despite later unavailability.
def test_accepted_reading_owns_one_slot_before_ack_and_publishes_without_readmission() -> None:
    queue = PersistQueue(capacity_entities=1)
    _publish_state(queue, PersistenceAdmissionState.AVAILABLE)
    ingress = _ingress(queue)

    decision = ingress.begin(_packet(frame=_frame()))

    assert decision.pre_tx_profile.processing_result is ProcessingResult.ACCEPTED
    assert decision.pre_tx_profile.ack_selected is AckSelection.ACCEPTED
    assert decision.admission is not None
    assert decision.admission.entity_kind is PersistQueueEntityKind.MEASUREMENT_PROFILE
    assert decision.admission.result is AdmissionResult.RESERVED
    assert queue.snapshot().reserved_entities == 1
    assert queue.snapshot().published_entities == 0

    _publish_state(
        queue,
        PersistenceAdmissionState.UNAVAILABLE_IO,
        generation=2,
    )
    finalized = ingress.finalize(
        decision,
        _terminal(AckTxResult.SUPPRESSED_AIRTIME_BUDGET),
    )

    assert finalized.admission is None
    assert isinstance(finalized.published_entity, MeasurementProfileUnitV1)
    assert finalized.published_entity.candidate is decision.candidate
    assert finalized.published_entity.profile.ack_tx_result is (
        AckTxResult.SUPPRESSED_AIRTIME_BUDGET
    )
    assert queue.snapshot().reserved_entities == 0
    assert queue.snapshot().published_entities == 1
    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    assert lease.entries[0].spec is MEASUREMENT_PROFILE_V1_SPEC
    assert lease.entries[0].entity is finalized.published_entity
    lease.acknowledge_durable((PersistQueueBatchDisposition.SQLITE_COMMITTED,))


# Reserves a profile-only slot before a permanent authenticated rejection response.
def test_authenticated_rejection_reserves_profile_only_before_ack() -> None:
    queue = PersistQueue(capacity_entities=1)
    _publish_state(queue, PersistenceAdmissionState.AVAILABLE)
    ingress = _ingress(queue)

    decision = ingress.begin(_packet(frame=_frame(control=0x30)))

    assert decision.pre_tx_profile.processing_result is (
        ProcessingResult.REJECTED_UNSUPPORTED_CONTROL
    )
    assert decision.pre_tx_profile.ack_selected is AckSelection.REJECTED_UNSUPPORTED
    assert decision.candidate is None
    assert decision.admission is not None
    assert decision.admission.entity_kind is PersistQueueEntityKind.PROFILE_ONLY
    assert decision.admission.result is AdmissionResult.RESERVED

    finalized = ingress.finalize(decision, _terminal(AckTxResult.TX_DONE))
    assert isinstance(finalized.published_entity, ProfileOnlyUnitV1)
    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    assert lease.entries[0].spec is PROFILE_ONLY_V1_SPEC
    assert lease.entries[0].entity is finalized.published_entity


# Maps each operational admission failure to retry-later without a second attempt.
@pytest.mark.parametrize(
    ("queue_state", "prefill", "expected_processing", "expected_admission"),
    (
        (
            PersistenceAdmissionState.UNAVAILABLE_IO,
            False,
            ProcessingResult.RETRY_LATER_PERSISTENCE_UNAVAILABLE,
            AdmissionResult.PERSISTENCE_UNAVAILABLE,
        ),
        (
            PersistenceAdmissionState.AVAILABLE,
            True,
            ProcessingResult.RETRY_LATER_QUEUE_FULL,
            AdmissionResult.QUEUE_FULL,
        ),
    ),
)
def test_response_eligible_admission_failure_selects_retry_later_once(
    queue_state: PersistenceAdmissionState,
    prefill: bool,
    expected_processing: ProcessingResult,
    expected_admission: AdmissionResult,
) -> None:
    queue = PersistQueue(capacity_entities=1)
    _publish_state(queue, PersistenceAdmissionState.AVAILABLE)
    if prefill:
        occupied = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert occupied.reservation is not None
        occupied.reservation.publish(object())
    if queue_state is not PersistenceAdmissionState.AVAILABLE:
        _publish_state(queue, queue_state, generation=2)
    ingress = _ingress(queue)

    decision = ingress.begin(_packet(frame=_frame()))

    assert decision.pre_tx_profile.processing_result is expected_processing
    assert decision.pre_tx_profile.ack_selected is AckSelection.RETRY_LATER
    assert decision.pre_tx_profile.ack_frame == EXPECTED_RETRY_ACK
    assert decision.candidate is None
    assert decision.admission is not None
    assert decision.admission.entity_kind is PersistQueueEntityKind.MEASUREMENT_PROFILE
    assert decision.admission.result is expected_admission
    published_before = queue.snapshot().published_entities

    finalized = ingress.finalize(decision, _terminal(AckTxResult.TX_DONE))

    assert finalized.admission is None
    assert finalized.published_entity is None
    assert queue.snapshot().published_entities == published_before


# Defers silent occurrence admission until terminal rearm and never invents a response.
@pytest.mark.parametrize(
    ("state", "expected_admission", "published"),
    (
        (
            PersistenceAdmissionState.AVAILABLE,
            AdmissionResult.RESERVED,
            True,
        ),
        (
            PersistenceAdmissionState.UNAVAILABLE_IO,
            AdmissionResult.PERSISTENCE_UNAVAILABLE,
            False,
        ),
    ),
)
def test_wrong_direction_attempts_profile_only_admission_after_rearm_without_response(
    state: PersistenceAdmissionState,
    expected_admission: AdmissionResult,
    published: bool,
) -> None:
    queue = PersistQueue(capacity_entities=1)
    _publish_state(queue, state)
    ingress = _ingress(queue)
    reflected_ack = _frame(
        domain=protocol.Domain.ACK_ACCEPTED_DOWNLINK,
        body=protocol.encode_ack(protocol.Ack(status=0)),
    )

    decision = ingress.begin(_packet(frame=reflected_ack))

    assert decision.pre_tx_profile.processing_result is ProcessingResult.WRONG_DIRECTION
    assert decision.pre_tx_profile.ack_selected is AckSelection.NONE
    assert decision.pre_tx_profile.ack_frame is None
    assert decision.admission is None
    assert queue.snapshot().reserved_entities == 0

    finalized = ingress.finalize(
        decision,
        _terminal(AckTxResult.NOT_APPLICABLE),
    )

    assert finalized.admission is not None
    assert finalized.admission.entity_kind is PersistQueueEntityKind.PROFILE_ONLY
    assert finalized.admission.result is expected_admission
    assert (finalized.published_entity is not None) is published
    assert decision.pre_tx_profile.processing_result is ProcessingResult.WRONG_DIRECTION
