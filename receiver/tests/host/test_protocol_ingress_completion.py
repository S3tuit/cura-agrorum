from __future__ import annotations

from dataclasses import FrozenInstanceError, fields, replace

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
        radio_state=RadioState.RX_SINGLE,
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
        radio_state=RadioState.RX_SINGLE,
    ),
    ProtocolIngressTerminalV1(
        AckTxResult.SET_TX_FAILED, 21, None, 23, radio_state=RadioState.RX_SINGLE,
    ),
    ProtocolIngressTerminalV1(
        AckTxResult.TX_TIMEOUT, 21, None, 23, radio_state=RadioState.RX_SINGLE,
    ),
    ProtocolIngressTerminalV1(
        AckTxResult.TX_DONE, 21, 22, 23, radio_state=RadioState.RX_SINGLE,
    ),
    ProtocolIngressTerminalV1(
        AckTxResult.UNKNOWN_INTERRUPTED, 21, None, None, radio_state=RadioState.SHUTDOWN,
    ),
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
        ProtocolIngressTerminalV1(
            AckTxResult.NOT_APPLICABLE, None, None, 23, radio_state=RadioState.RX_SINGLE,
        ),
        ProtocolIngressTerminalV1(
            AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
            21,
            None,
            23,
            radio_state=RadioState.RX_SINGLE,
        ),
        ProtocolIngressTerminalV1(
            AckTxResult.TX_DONE, 21, None, 23, radio_state=RadioState.RX_SINGLE,
        ),
        ProtocolIngressTerminalV1(
            AckTxResult.TX_TIMEOUT, 19, None, 23, radio_state=RadioState.RX_SINGLE,
        ),
        ProtocolIngressTerminalV1(
            AckTxResult.TX_DONE, 22, 21, 23, radio_state=RadioState.RX_SINGLE,
        ),
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
            radio_state=RadioState.RX_SINGLE,
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
                radio_state=RadioState.RX_SINGLE,
            ),
        )

    assert first_queue.snapshot().reserved_entities == 1
    assert second_queue.snapshot().reserved_entities == 0


SILENT_FRAME = authenticated_frame(node_id=b"unknown!")
SILENT_TERMINAL = ProtocolIngressTerminalV1(
    AckTxResult.NOT_APPLICABLE, None, None, 23, radio_state=RadioState.RX_SINGLE,
)


# F-002: Rejects missing rearm evidence without consuming silent or accepted occurrences.
@pytest.mark.parametrize("frame", (SILENT_FRAME, REVIEWED_CURRENT_FRAME), ids=("silent", "suppressed"))
@pytest.mark.parametrize("radio_state", (RadioState.RX_SINGLE, RadioState.RX_EVENT_PENDING))
def test_rearmed_completion_requires_t6_and_preserves_the_original_occurrence(
    frame: bytes,
    radio_state: RadioState,
) -> None:
    queue = _queue(state=PersistenceAdmissionState.AVAILABLE)
    ingress = _ingress(queue)
    occurrence = ingress.begin(ingress_packet(frame=frame))
    terminal = ProtocolIngressTerminalV1(
        AckTxResult.NOT_APPLICABLE if frame == SILENT_FRAME else AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
        None, None, None, radio_state=radio_state,
    )
    before = queue.snapshot()

    with pytest.raises(ProtocolIngressInterfaceError, match="requires T6"):
        ingress.finalize(occurrence, terminal)

    assert queue.snapshot() == before
    assert ingress._active_occurrence is occurrence
    assert queue.claim_batch(max_entities=1) is None
    finalized = ingress.finalize(
        occurrence, replace(terminal, t6_set_rx_issued_monotonic_us=23),
    )
    assert finalized.published_entity is not None
    assert finalized.published_entity.profile.t6_set_rx_issued_monotonic_us == 23
    assert ingress._active_occurrence is None


# F-002: An unfinished or initialization-only state cannot finalize even with T6 present.
@pytest.mark.parametrize(
    "radio_state",
    (RadioState.INITIALIZING, RadioState.INITIALIZATION_FAILED, RadioState.TX_ACTIVE, RadioState.RECOVERING),
)
@pytest.mark.parametrize("silent", (False, True), ids=("selected_ack", "silent"))
def test_incomplete_radio_state_cannot_publish_an_occurrence(
    radio_state: RadioState,
    silent: bool,
) -> None:
    queue = _queue(state=PersistenceAdmissionState.AVAILABLE)
    ingress = _ingress(queue)
    occurrence = ingress.begin(
        ingress_packet(frame=SILENT_FRAME if silent else REVIEWED_CURRENT_FRAME)
    )
    completed = SILENT_TERMINAL if silent else _tx_done_terminal()
    before = queue.snapshot()

    with pytest.raises(ProtocolIngressInterfaceError, match="radio handling must reach"):
        ingress.finalize(occurrence, replace(completed, radio_state=radio_state))

    assert queue.snapshot() == before
    assert ingress._active_occurrence is occurrence
    assert ingress.finalize(occurrence, completed).published_entity is not None


# F-002: A terminal receiver failure preserves the exact best available ACK and RX evidence.
@pytest.mark.parametrize("terminal", TERMINAL_CASES + (SILENT_TERMINAL,))
@pytest.mark.parametrize(
    "radio_state",
    (RadioState.SHUTDOWN, RadioState.RECOVERY_EXHAUSTED, RadioState.HARDWARE_MISSING),
)
@pytest.mark.parametrize("t6", (None, 23), ids=("rx_not_issued", "rx_issued"))
def test_stopped_completion_preserves_missing_or_observed_error_path_timestamps(
    terminal: ProtocolIngressTerminalV1,
    radio_state: RadioState,
    t6: int | None,
) -> None:
    queue = _queue(state=PersistenceAdmissionState.AVAILABLE)
    ingress = _ingress(queue)
    silent = terminal.ack_tx_result is AckTxResult.NOT_APPLICABLE
    occurrence = ingress.begin(
        ingress_packet(frame=SILENT_FRAME if silent else REVIEWED_CURRENT_FRAME)
    )
    terminal = replace(terminal, radio_state=radio_state, t6_set_rx_issued_monotonic_us=t6)

    finalized = ingress.finalize(occurrence, terminal)

    if silent:
        assert isinstance(finalized.published_entity, ProfileOnlyUnitV1)
        expected_spec = PROFILE_ONLY_V1_SPEC
    else:
        assert isinstance(finalized.published_entity, MeasurementProfileUnitV1)
        assert finalized.published_entity.candidate is occurrence.candidate
        assert finalized.published_entity.profile.processing_result is ProcessingResult.ACCEPTED
        expected_spec = MEASUREMENT_PROFILE_V1_SPEC
    profile = finalized.published_entity.profile
    assert profile.ack_tx_result is terminal.ack_tx_result
    assert profile.t4_set_tx_attempted_monotonic_us == terminal.t4_set_tx_attempted_monotonic_us
    assert profile.t5_tx_done_monotonic_us == terminal.t5_tx_done_monotonic_us
    assert profile.t6_set_rx_issued_monotonic_us == t6
    assert not hasattr(profile, "radio_state")
    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    assert lease.entries[0].spec is expected_spec
    assert lease.entries[0].entity is finalized.published_entity
    assert ingress._active_occurrence is None


# F-002: Handles a new DIO1 event immediately following confirmed receive rearm.
@pytest.mark.parametrize("terminal", TERMINAL_CASES[:4] + (SILENT_TERMINAL,))
def test_rearm_followed_by_a_new_rx_event_is_a_complete_previous_occurrence(
    terminal: ProtocolIngressTerminalV1,
) -> None:
    queue = _queue(state=PersistenceAdmissionState.AVAILABLE)
    ingress = _ingress(queue)
    silent = terminal.ack_tx_result is AckTxResult.NOT_APPLICABLE
    occurrence = ingress.begin(
        ingress_packet(frame=SILENT_FRAME if silent else REVIEWED_CURRENT_FRAME)
    )

    finalized = ingress.finalize(
        occurrence, replace(terminal, radio_state=RadioState.RX_EVENT_PENDING),
    )

    assert finalized.published_entity is not None
    assert finalized.published_entity.profile.t6_set_rx_issued_monotonic_us == 23
    assert ingress._active_occurrence is None


# F-002: The exceptional unknown ACK terminal must accompany receiver termination.
@pytest.mark.parametrize("radio_state", (RadioState.RX_SINGLE, RadioState.RX_EVENT_PENDING))
def test_unknown_interrupted_ack_cannot_claim_a_resumed_receiver(
    radio_state: RadioState,
) -> None:
    queue = _queue(state=PersistenceAdmissionState.AVAILABLE)
    ingress = _ingress(queue)
    occurrence = ingress.begin(ingress_packet())
    terminal = ProtocolIngressTerminalV1(
        AckTxResult.UNKNOWN_INTERRUPTED, 21, None, 23, radio_state=radio_state,
    )
    before = queue.snapshot()

    with pytest.raises(ProtocolIngressInterfaceError, match="UNKNOWN_INTERRUPTED requires"):
        ingress.finalize(occurrence, terminal)

    assert queue.snapshot() == before
    assert ingress.finalize(
        occurrence, replace(terminal, radio_state=RadioState.SHUTDOWN),
    ).published_entity is not None


# F-002: Private evolving facts become a frozen queue value only at valid completion.
def test_private_radio_construction_does_not_escape_before_completion() -> None:
    queue = _queue(state=PersistenceAdmissionState.AVAILABLE)
    ingress = _ingress(queue)
    occurrence = ingress.begin(ingress_packet())
    draft = {
        "ack_tx_result": AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
        "t4_set_tx_attempted_monotonic_us": None,
        "t5_tx_done_monotonic_us": None,
        "t6_set_rx_issued_monotonic_us": None,
        "radio_state": RadioState.RECOVERING,
    }
    incomplete = ProtocolIngressTerminalV1(**draft)  # type: ignore[arg-type]
    with pytest.raises(ProtocolIngressInterfaceError):
        ingress.finalize(occurrence, incomplete)
    assert queue.snapshot().reserved_entities == 1
    assert queue.claim_batch(max_entities=1) is None

    draft.update(radio_state=RadioState.RX_SINGLE, t6_set_rx_issued_monotonic_us=23)
    complete = ProtocolIngressTerminalV1(**draft)  # type: ignore[arg-type]
    finalized = ingress.finalize(occurrence, complete)
    draft.update(t6_set_rx_issued_monotonic_us=99)

    assert incomplete.radio_state is RadioState.RECOVERING
    assert incomplete.t6_set_rx_issued_monotonic_us is None
    assert isinstance(finalized.published_entity, MeasurementProfileUnitV1)
    assert finalized.published_entity.candidate is occurrence.candidate
    assert finalized.published_entity.profile.t6_set_rx_issued_monotonic_us == 23
    with pytest.raises(FrozenInstanceError):
        finalized.published_entity.profile.t6_set_rx_issued_monotonic_us = 99  # type: ignore[misc]
