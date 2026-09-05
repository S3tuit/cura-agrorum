from __future__ import annotations

from dataclasses import FrozenInstanceError, fields

import pytest

from cura_receiver.generated import protocol_v2_lora_generated as protocol
from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    AdmissionResult,
    PersistenceAdmissionState,
    PersistQueueEntityKind,
    ProcessingResult,
)
from cura_receiver.persist_queue import (
    PersistQueue,
    PersistQueueBatchDisposition,
    PersistenceAdmissionSnapshot,
)
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC
from cura_receiver.protocol_ingress import (
    ProtocolIngress,
    ProtocolIngressInterfaceError,
    ProtocolIngressTerminalV1,
)
from tests.support.builders.protocol_ingress import (
    REVIEWED_CURRENT_FRAME,
    REVIEWED_NODE_ID,
    REVIEWED_NODE_KEY,
    REVIEWED_RETRY_LATER_ACK,
    authenticated_frame,
    ingress_packet,
)
from tests.support.fakes.os_clock import FakeOsClock


def _malformed_body() -> bytes:
    body = bytearray(protocol.READING_BODY_SIZE)
    body[6] = 1
    return bytes(body)


RESPONSE_ELIGIBLE_CASES = (
    (
        "accepted",
        REVIEWED_CURRENT_FRAME,
        PersistQueueEntityKind.MEASUREMENT_PROFILE,
    ),
    (
        "unsupported_control",
        authenticated_frame(control=0x30),
        PersistQueueEntityKind.PROFILE_ONLY,
    ),
    (
        "unsupported_domain",
        authenticated_frame(domain=0x7F, body=bytes(1)),
        PersistQueueEntityKind.PROFILE_ONLY,
    ),
    (
        "malformed_length",
        authenticated_frame(body=bytes(31)),
        PersistQueueEntityKind.PROFILE_ONLY,
    ),
    (
        "malformed_body",
        authenticated_frame(body=_malformed_body()),
        PersistQueueEntityKind.PROFILE_ONLY,
    ),
)


def _queue(
    *,
    state: PersistenceAdmissionState,
    full: bool = False,
) -> PersistQueue:
    queue = PersistQueue(capacity_entities=1)
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=1,
            state=state,
            changed_at_monotonic_us=1,
        )
    )
    if full:
        occupied = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert occupied.reservation is not None
        occupied.reservation.publish(object())
    return queue


def _ingress(queue: PersistQueue) -> ProtocolIngress:
    return ProtocolIngress(
        queue=queue,
        monotonic_clock=FakeOsClock(monotonic_us=20, realtime_us=0),
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
    )


def _tx_done_terminal() -> ProtocolIngressTerminalV1:
    return ProtocolIngressTerminalV1(
        ack_tx_result=AckTxResult.TX_DONE,
        t4_set_tx_attempted_monotonic_us=21,
        t5_tx_done_monotonic_us=22,
        t6_set_rx_issued_monotonic_us=23,
    )


# Converts every response-eligible admission failure to the exact retry outcome once.
@pytest.mark.parametrize(
    ("failure_state", "full", "expected_result", "admission_result"),
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
@pytest.mark.parametrize(
    ("case_name", "frame", "expected_kind"),
    RESPONSE_ELIGIBLE_CASES,
    ids=tuple(case[0] for case in RESPONSE_ELIGIBLE_CASES),
)
def test_every_response_eligible_packet_uses_retry_later_on_admission_failure(
    failure_state: PersistenceAdmissionState,
    full: bool,
    expected_result: ProcessingResult,
    admission_result: AdmissionResult,
    case_name: str,
    frame: bytes,
    expected_kind: PersistQueueEntityKind,
) -> None:
    del case_name
    queue = _queue(state=failure_state, full=full)
    ingress = _ingress(queue)
    published_before = queue.snapshot().published_entities

    decision = ingress.begin(ingress_packet(frame=frame))

    assert decision.pre_tx_profile.processing_result is expected_result
    assert decision.pre_tx_profile.ack_selected is AckSelection.RETRY_LATER
    assert decision.pre_tx_profile.ack_frame == REVIEWED_RETRY_LATER_ACK
    assert decision.candidate is None
    assert decision.admission is not None
    assert decision.admission.entity_kind is expected_kind
    assert decision.admission.result is admission_result
    assert queue.snapshot().reserved_entities == 0

    finalized = ingress.finalize(decision, _tx_done_terminal())
    assert finalized.admission is None
    assert finalized.published_entity is None
    assert queue.snapshot().published_entities == published_before


# Reprocesses the same occurrence after availability returns instead of caching retry-later.
def test_retry_later_is_not_cached_after_persistence_recovers() -> None:
    queue = _queue(state=PersistenceAdmissionState.UNAVAILABLE_IO)
    ingress = _ingress(queue)

    retry = ingress.begin(
        ingress_packet(frame=REVIEWED_CURRENT_FRAME, occurrence_sequence=1)
    )
    assert retry.pre_tx_profile.processing_result is (
        ProcessingResult.RETRY_LATER_PERSISTENCE_UNAVAILABLE
    )
    ingress.finalize(retry, _tx_done_terminal())

    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=2,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=2,
        )
    )
    accepted = ingress.begin(
        ingress_packet(frame=REVIEWED_CURRENT_FRAME, occurrence_sequence=2)
    )

    assert accepted.pre_tx_profile.processing_result is ProcessingResult.ACCEPTED
    assert accepted.pre_tx_profile.ack_selected is AckSelection.ACCEPTED
    assert accepted.admission is not None
    assert accepted.admission.result is AdmissionResult.RESERVED


# Reprocesses a queue-full occurrence after capacity release without cached rejection.
def test_retry_later_is_not_cached_after_queue_capacity_returns() -> None:
    queue = _queue(state=PersistenceAdmissionState.AVAILABLE, full=True)
    ingress = _ingress(queue)

    retry = ingress.begin(
        ingress_packet(frame=REVIEWED_CURRENT_FRAME, occurrence_sequence=1)
    )
    assert retry.pre_tx_profile.processing_result is (
        ProcessingResult.RETRY_LATER_QUEUE_FULL
    )
    ingress.finalize(retry, _tx_done_terminal())
    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    lease.acknowledge_durable((PersistQueueBatchDisposition.SQLITE_COMMITTED,))

    accepted = ingress.begin(
        ingress_packet(frame=REVIEWED_CURRENT_FRAME, occurrence_sequence=2)
    )

    assert accepted.pre_tx_profile.processing_result is ProcessingResult.ACCEPTED
    assert accepted.admission is not None
    assert accepted.admission.result is AdmissionResult.RESERVED


TERMINAL_CASES = (
    ProtocolIngressTerminalV1(
        AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
        None,
        None,
        23,
    ),
    ProtocolIngressTerminalV1(AckTxResult.SET_TX_FAILED, 21, None, 23),
    ProtocolIngressTerminalV1(AckTxResult.TX_TIMEOUT, 21, None, 23),
    ProtocolIngressTerminalV1(AckTxResult.TX_DONE, 21, 22, 23),
    ProtocolIngressTerminalV1(AckTxResult.UNKNOWN_INTERRUPTED, 21, None, None),
)


# Publishes every terminal ACK outcome as one complete immutable pre-reserved pair.
@pytest.mark.parametrize(
    "terminal",
    TERMINAL_CASES,
    ids=tuple(value.ack_tx_result.name.lower() for value in TERMINAL_CASES),
)
def test_terminal_completion_preserves_pre_tx_facts_and_identical_object_reference(
    terminal: ProtocolIngressTerminalV1,
) -> None:
    queue = _queue(state=PersistenceAdmissionState.AVAILABLE)
    ingress = _ingress(queue)
    packet = ingress_packet(
        frame=REVIEWED_CURRENT_FRAME,
        busy_wait_total_us=9,
        busy_wait_max_us=5,
        busy_wait_count=2,
        busy_timeout_count=1,
        last_busy_timeout_opcode=0x1D,
    )
    decision = ingress.begin(packet)
    pre_tx_values = {
        field.name: getattr(decision.pre_tx_profile, field.name)
        for field in fields(decision.pre_tx_profile)
    }

    finalized = ingress.finalize(decision, terminal)

    assert finalized.admission is None
    assert finalized.published_entity is not None
    profile = finalized.published_entity.profile
    for field_name, expected in pre_tx_values.items():
        assert getattr(profile, field_name) == expected
    assert profile.ack_tx_result is terminal.ack_tx_result
    assert profile.t4_set_tx_attempted_monotonic_us == (
        terminal.t4_set_tx_attempted_monotonic_us
    )
    assert profile.t5_tx_done_monotonic_us == terminal.t5_tx_done_monotonic_us
    assert profile.t6_set_rx_issued_monotonic_us == (
        terminal.t6_set_rx_issued_monotonic_us
    )
    assert not hasattr(profile, "__dict__")
    with pytest.raises(FrozenInstanceError):
        profile.ack_tx_result = AckTxResult.TX_DONE  # type: ignore[misc]

    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    assert lease.entries[0].entity is finalized.published_entity
    with pytest.raises(ProtocolIngressInterfaceError):
        ingress.finalize(decision, terminal)


# Rejects terminal combinations that cannot occur in the receive-to-ACK flow.
@pytest.mark.parametrize(
    "terminal",
    (
        ProtocolIngressTerminalV1(AckTxResult.NOT_APPLICABLE, None, None, 23),
        ProtocolIngressTerminalV1(
            AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
            21,
            None,
            23,
        ),
        ProtocolIngressTerminalV1(AckTxResult.TX_DONE, 21, None, 23),
        ProtocolIngressTerminalV1(AckTxResult.TX_TIMEOUT, 19, None, 23),
        ProtocolIngressTerminalV1(AckTxResult.TX_DONE, 22, 21, 23),
    ),
)
def test_selected_ack_rejects_impossible_terminal_radio_facts(
    terminal: ProtocolIngressTerminalV1,
) -> None:
    queue = _queue(state=PersistenceAdmissionState.AVAILABLE)
    ingress = _ingress(queue)
    decision = ingress.begin(ingress_packet())

    with pytest.raises(ProtocolIngressInterfaceError):
        ingress.finalize(decision, terminal)

    assert queue.snapshot().reserved_entities == 1
    valid = ingress.finalize(
        decision,
        ProtocolIngressTerminalV1(
            AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
            None,
            None,
            23,
        ),
    )
    assert valid.published_entity is not None


# Prevents a decision and its live queue reservation from crossing ingress owners.
def test_foreign_ingress_cannot_finalize_a_live_decision() -> None:
    first_queue = _queue(state=PersistenceAdmissionState.AVAILABLE)
    second_queue = _queue(state=PersistenceAdmissionState.AVAILABLE)
    first = _ingress(first_queue)
    second = _ingress(second_queue)
    decision = first.begin(ingress_packet())

    with pytest.raises(ProtocolIngressInterfaceError):
        second.finalize(
            decision,
            ProtocolIngressTerminalV1(
                AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
                None,
                None,
                23,
            ),
        )

    assert first_queue.snapshot().reserved_entities == 1
    assert second_queue.snapshot().reserved_entities == 0
