from __future__ import annotations

import copy
import pickle
from dataclasses import replace

import pytest

from cura_receiver.generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    PersistenceAdmissionState,
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
)
from cura_receiver.protocol_ingress import (
    ProtocolIngress,
    ProtocolIngressInterfaceError,
    ProtocolIngressOccurrenceV1,
    ProtocolIngressTerminalV1,
)
from tests.support.builders.protocol_ingress import (
    REVIEWED_ACCEPTED_ACK,
    REVIEWED_NODE_ID,
    REVIEWED_NODE_KEY,
    ingress_packet,
)
from tests.support.fakes.os_clock import FakeOsClock


def _context(
    *,
    state: PersistenceAdmissionState = PersistenceAdmissionState.AVAILABLE,
    full: bool = False,
) -> tuple[PersistQueue, ProtocolIngress]:
    queue = PersistQueue(capacity_entities=1)
    queue.publish_admission_state(PersistenceAdmissionSnapshot(1, state, 1))
    if full:
        reserved = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert reserved.reservation is not None
        reserved.reservation.publish(object())
    ingress = ProtocolIngress(
        queue=queue,
        monotonic_clock=FakeOsClock(monotonic_us=20, realtime_us=0),
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
    )
    return queue, ingress


def _terminal(occurrence: ProtocolIngressOccurrenceV1) -> ProtocolIngressTerminalV1:
    return ProtocolIngressTerminalV1(
        ack_tx_result=(
            AckTxResult.NOT_APPLICABLE
            if occurrence.pre_tx_profile.ack_selected is AckSelection.NONE
            else AckTxResult.SUPPRESSED_AIRTIME_BUDGET
        ),
        t4_set_tx_attempted_monotonic_us=None,
        t5_tx_done_monotonic_us=None,
        t6_set_rx_issued_monotonic_us=23,
        radio_state=RadioState.RX_SINGLE,
    )


# F-001: Prevents reconstructing a completion capability while retaining its reservation.
@pytest.mark.parametrize("operation", ("construct", "replace", "copy", "deepcopy", "pickle"))
def test_occurrence_cannot_be_constructed_replaced_copied_or_serialized(
    operation: str,
) -> None:
    queue, ingress = _context()
    occurrence = ingress.begin(ingress_packet())
    before = queue.snapshot()

    with pytest.raises(TypeError):
        if operation == "construct":
            ProtocolIngressOccurrenceV1()
        elif operation == "replace":
            replace(occurrence, candidate=None)
        elif operation == "copy":
            copy.copy(occurrence)
        elif operation == "deepcopy":
            copy.deepcopy(occurrence)
        else:
            pickle.dumps(occurrence)

    assert queue.snapshot() == before
    finalized = ingress.finalize(occurrence, _terminal(occurrence))
    assert isinstance(finalized.published_entity, MeasurementProfileUnitV1)
    assert finalized.published_entity.candidate is occurrence.candidate
    assert finalized.published_entity.profile.processing_result is ProcessingResult.ACCEPTED
    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    assert lease.entries[0].spec is MEASUREMENT_PROFILE_V1_SPEC
    assert lease.entries[0].entity is finalized.published_entity


# F-001: Exposes read-only facts without letting a copied profile replace the accepted one.
def test_occurrence_facts_cannot_be_replaced_through_the_public_boundary() -> None:
    queue, ingress = _context()
    occurrence = ingress.begin(ingress_packet())
    original = occurrence.pre_tx_profile
    changed = replace(original, occurrence_sequence=99)

    assert not hasattr(occurrence, "__dict__")
    assert occurrence.ack_frame is original.ack_frame
    assert occurrence.ack_frame == REVIEWED_ACCEPTED_ACK
    for name, value in (
        ("pre_tx_profile", changed),
        ("candidate", None),
        ("admission", None),
        ("ack_frame", None),
    ):
        with pytest.raises(AttributeError):
            setattr(occurrence, name, value)
    with pytest.raises(ProtocolIngressInterfaceError):
        ingress.finalize(changed, _terminal(occurrence))  # type: ignore[arg-type]

    assert occurrence.pre_tx_profile is original
    assert queue.snapshot().reserved_entities == 1
    finalized = ingress.finalize(occurrence, _terminal(occurrence))
    assert isinstance(finalized.published_entity, MeasurementProfileUnitV1)
    assert finalized.published_entity.candidate is occurrence.candidate
    assert finalized.published_entity.profile.occurrence_sequence == original.occurrence_sequence


# F-001: Rejects even an instance of the exact handle class without minting authority.
def test_reconstructed_handle_cannot_consume_the_active_occurrence() -> None:
    queue, ingress = _context()
    occurrence = ingress.begin(ingress_packet())
    reconstructed = object.__new__(ProtocolIngressOccurrenceV1)
    before = queue.snapshot()

    with pytest.raises(ProtocolIngressInterfaceError, match="foreign or stale"):
        ingress.finalize(reconstructed, _terminal(occurrence))

    assert queue.snapshot() == before
    assert ingress.finalize(occurrence, _terminal(occurrence)).published_entity is not None


# F-001: A copied stateful owner must not inherit authority over the live handle.
@pytest.mark.parametrize("copy_owner", (copy.copy, copy.deepcopy), ids=("shallow", "deep"))
def test_ingress_owner_cannot_be_copied_with_a_live_occurrence(copy_owner) -> None:
    queue, ingress = _context()
    occurrence = ingress.begin(ingress_packet())
    before = queue.snapshot()

    with pytest.raises(TypeError, match="owners are not copyable"):
        copy_owner(ingress)

    assert queue.snapshot() == before
    assert ingress._active_occurrence is occurrence
    assert ingress.finalize(occurrence, _terminal(occurrence)).published_entity is not None


# F-001: Protects every active occurrence, including paths with no pre-TX reservation.
@pytest.mark.parametrize(
    ("silent", "state", "full"),
    (
        (False, PersistenceAdmissionState.AVAILABLE, False),
        (True, PersistenceAdmissionState.AVAILABLE, False),
        (False, PersistenceAdmissionState.UNAVAILABLE_IO, False),
        (False, PersistenceAdmissionState.AVAILABLE, True),
    ),
    ids=("accepted", "silent", "unavailable", "full"),
)
def test_active_occurrence_blocks_begin_until_completion(
    silent: bool,
    state: PersistenceAdmissionState,
    full: bool,
) -> None:
    queue, ingress = _context(state=state, full=full)
    packet = ingress_packet(frame=bytes(23)) if silent else ingress_packet()
    occurrence = ingress.begin(packet)
    before = queue.snapshot()

    with pytest.raises(ProtocolIngressInterfaceError, match="must be finalized first"):
        ingress.begin(ingress_packet(occurrence_sequence=2))
    assert queue.snapshot() == before
    assert ingress._active_occurrence is occurrence

    ingress.finalize(occurrence, _terminal(occurrence))
    assert ingress._active_occurrence is None
    lease = queue.claim_batch(max_entities=1)
    if lease is not None:
        lease.acknowledge_durable((PersistQueueBatchDisposition.SQLITE_COMMITTED,))
    following = ingress.begin(ingress_packet(occurrence_sequence=2))
    before = queue.snapshot()
    with pytest.raises(ProtocolIngressInterfaceError, match="foreign or stale"):
        ingress.finalize(occurrence, _terminal(occurrence))
    assert queue.snapshot() == before
    assert ingress._active_occurrence is following
    ingress.finalize(following, _terminal(following))
