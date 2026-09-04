from __future__ import annotations

import copy
from dataclasses import FrozenInstanceError, fields
from threading import Event

from hypothesis import given, settings, strategies as st
import pytest

from cura_receiver.generated.receiver_enums_generated import (
    AdmissionResult,
    PersistenceAdmissionState,
    PersistQueueEntityKind,
)
from cura_receiver.persist_queue import (
    PERSIST_QUEUE_MAX_ENTITIES,
    PersistQueue,
    PersistQueueBatchDisposition,
    PersistQueueConfigurationError,
    PersistQueueInterfaceError,
    PersistQueueSnapshot,
    PersistenceAdmissionSnapshot,
)
from cura_receiver.persist_queue_entities import (
    CLOCK_OBSERVATION_V1_SPEC,
    DIAGNOSTIC_V1_SPEC,
    MEASUREMENT_PROFILE_V1_SPEC,
    PROFILE_ONLY_V1_SPEC,
    RECEIVER_HEALTH_REQUEST_V1_SPEC,
    PersistQueueEntitySpec,
    ProfileOnlyUnitV1,
)
from cura_receiver.quarantine_evidence import (
    QuarantineEvidenceEncodeError,
    encode_quarantine_evidence_v1,
)
from tests.support.models.persist_queue import (
    ReferencePersistQueue,
    ReferenceQueueViolation,
)


ALL_SPECS = (
    MEASUREMENT_PROFILE_V1_SPEC,
    PROFILE_ONLY_V1_SPEC,
    RECEIVER_HEALTH_REQUEST_V1_SPEC,
    DIAGNOSTIC_V1_SPEC,
    CLOCK_OBSERVATION_V1_SPEC,
)


def _make_available(queue: PersistQueue, generation: int = 1) -> None:
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=generation,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=generation,
        )
    )


def _publish(queue: PersistQueue, entity: object, *, spec=PROFILE_ONLY_V1_SPEC):
    result = queue.try_reserve_one(spec)
    assert result.status is AdmissionResult.RESERVED
    assert result.reservation is not None
    result.reservation.publish(entity)
    return result.reservation.token


# Construction rejects non-integer and out-of-pilot capacity values.
@pytest.mark.parametrize("capacity", (-1, 0, True, 501, 10**100))
def test_constructor_rejects_capacity_outside_the_pilot_bound(
    capacity: object,
) -> None:
    with pytest.raises(PersistQueueConfigurationError):
        PersistQueue(capacity_entities=capacity)  # type: ignore[arg-type]


# Both capacity boundaries retain the exact original backing arrays after a full cycle.
@pytest.mark.parametrize("capacity", (1, PERSIST_QUEUE_MAX_ENTITIES))
def test_constructor_preallocates_fixed_parallel_backing_arrays(capacity: int) -> None:
    queue = PersistQueue(capacity_entities=capacity)

    assert PERSIST_QUEUE_MAX_ENTITIES == 500
    assert len(queue._payload_slots) == capacity
    assert len(queue._spec_slots) == capacity
    assert len(queue._token_slots) == capacity
    with pytest.raises(AttributeError):
        queue.capacity_entities = 1  # type: ignore[misc]
    identities = tuple(
        id(storage)
        for storage in (
            queue._payload_slots,
            queue._spec_slots,
            queue._token_slots,
        )
    )

    _make_available(queue)
    for index in range(capacity):
        _publish(queue, index, spec=ALL_SPECS[index % len(ALL_SPECS)])
    batch = queue.claim_batch(max_entities=capacity)
    assert batch is not None
    batch.acknowledge_durable(
        (PersistQueueBatchDisposition.SQLITE_COMMITTED,) * capacity
    )

    assert tuple(
        id(storage)
        for storage in (
            queue._payload_slots,
            queue._spec_slots,
            queue._token_slots,
        )
    ) == identities
    assert tuple(map(len, (queue._payload_slots, queue._spec_slots, queue._token_slots))) == (
        capacity,
        capacity,
        capacity,
    )


# Public accounting exposes entity counts and contains no residual byte budget.
def test_snapshot_and_reservation_result_expose_entity_counts_only() -> None:
    queue = PersistQueue(capacity_entities=2)
    snapshot = queue.snapshot()

    assert [field.name for field in fields(PersistQueueSnapshot)] == [
        "admission_snapshot",
        "capacity_entities",
        "reserved_entities",
        "published_entities",
        "claimed_entities",
        "closed",
        "closed_and_drained",
    ]
    assert not any("byte" in field.name for field in fields(type(snapshot)))

    result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert result.status is AdmissionResult.PERSISTENCE_UNAVAILABLE
    assert result.reservation is None
    assert result.used_entities_before == 0
    assert result.capacity_entities == 2
    assert not any("byte" in field.name for field in fields(type(result)))


# Each admission result carries the state and occupancy observed by its atomic decision.
def test_admission_snapshot_and_decision_are_atomic_observations() -> None:
    queue = PersistQueue(capacity_entities=1)

    unavailable = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert unavailable.admission_snapshot == PersistenceAdmissionSnapshot(
        generation=0,
        state=PersistenceAdmissionState.UNAVAILABLE_STARTING,
        changed_at_monotonic_us=0,
    )

    _make_available(queue)
    reserved = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert reserved.status is AdmissionResult.RESERVED
    assert reserved.used_entities_before == 0
    assert reserved.admission_snapshot.generation == 1
    assert reserved.reservation is not None
    reserved.reservation.publish(object())

    full = queue.try_reserve_one(DIAGNOSTIC_V1_SPEC)
    assert full.status is AdmissionResult.QUEUE_FULL
    assert full.used_entities_before == 1
    assert full.admission_snapshot.generation == 1


# Every non-available persistence state produces the same non-mutating gate result.
@pytest.mark.parametrize(
    "state",
    tuple(
        state
        for state in PersistenceAdmissionState
        if state is not PersistenceAdmissionState.AVAILABLE
    ),
)
def test_every_unavailable_admission_state_rejects_without_touching_owned_work(
    state: PersistenceAdmissionState,
) -> None:
    queue = PersistQueue(capacity_entities=2)
    _make_available(queue)
    payload = object()
    _publish(queue, payload)
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=2,
            state=state,
            changed_at_monotonic_us=2,
        )
    )

    result = queue.try_reserve_one(DIAGNOSTIC_V1_SPEC)

    assert result.status is AdmissionResult.PERSISTENCE_UNAVAILABLE
    assert result.reservation is None
    assert result.used_entities_before == 1
    assert result.admission_snapshot.state is state
    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    assert lease.entries[0].entity is payload
    lease.release_for_retry()


# Invalid generations and value types leave the initial admission snapshot untouched.
@pytest.mark.parametrize(
    "invalid_snapshot",
    (
        PersistenceAdmissionSnapshot(
            generation=0,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=1,
        ),
        PersistenceAdmissionSnapshot(
            generation=2,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=2,
        ),
        PersistenceAdmissionSnapshot(
            generation=1,
            state="AVAILABLE",  # type: ignore[arg-type]
            changed_at_monotonic_us=1,
        ),
        PersistenceAdmissionSnapshot(
            generation=1,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=-1,
        ),
    ),
)
def test_invalid_admission_publication_preserves_the_initial_snapshot(
    invalid_snapshot: PersistenceAdmissionSnapshot,
) -> None:
    queue = PersistQueue(capacity_entities=1)

    with pytest.raises(PersistQueueInterfaceError):
        queue.publish_admission_state(invalid_snapshot)

    assert queue.snapshot().admission_snapshot.generation == 0


# A valid next generation still cannot move its monotonic change time backward.
def test_admission_publication_rejects_time_regression() -> None:
    queue = PersistQueue(capacity_entities=1)
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=1,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=10,
        )
    )

    with pytest.raises(PersistQueueInterfaceError):
        queue.publish_admission_state(
            PersistenceAdmissionSnapshot(
                generation=2,
                state=PersistenceAdmissionState.UNAVAILABLE_IO,
                changed_at_monotonic_us=9,
            )
        )

    assert queue.snapshot().admission_snapshot.generation == 1


# A reserved tail owns capacity without becoming consumer-visible and can be cancelled once.
def test_outstanding_reservation_counts_but_is_invisible_and_cancellable() -> None:
    queue = PersistQueue(capacity_entities=2)
    _make_available(queue)
    result = queue.try_reserve_one(MEASUREMENT_PROFILE_V1_SPEC)
    assert result.reservation is not None

    assert queue.snapshot().reserved_entities == 1
    assert queue.snapshot().published_entities == 0
    assert queue.claim_batch(max_entities=1) is None
    with pytest.raises(PersistQueueInterfaceError, match="outstanding"):
        queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)

    result.reservation.cancel()
    assert queue.snapshot().reserved_entities == 0
    assert queue.snapshot().closed_and_drained is False
    with pytest.raises(PersistQueueInterfaceError):
        result.reservation.cancel()
    with pytest.raises(PersistQueueInterfaceError):
        result.reservation.publish(object())


class _OpaquePayload:
    def __len__(self) -> int:
        raise AssertionError("queue inspected payload length")

    def __copy__(self):
        raise AssertionError("queue copied payload")

    def __deepcopy__(self, memo: object):
        raise AssertionError("queue deep-copied payload")

    def __reduce__(self):
        raise AssertionError("queue serialized payload")


# Publication stores the exact payload reference without size inspection, copying or encoding.
def test_publication_transfers_the_identical_opaque_reference_under_pressure() -> None:
    queue = PersistQueue(capacity_entities=1)
    _make_available(queue)
    result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert result.reservation is not None
    payload = _OpaquePayload()

    result.reservation.publish(payload)

    batch = queue.claim_batch(max_entities=1)
    assert batch is not None
    assert batch.entries[0].entity is payload
    assert batch.entries[0].spec is PROFILE_ONLY_V1_SPEC
    assert batch.entries[0].token is result.reservation.token


# Every specification costs one slot and wraparound preserves global FIFO identity.
def test_every_kind_costs_one_slot_and_fifo_wraparound_preserves_identity() -> None:
    queue = PersistQueue(capacity_entities=3)
    _make_available(queue)
    objects = [object() for _ in range(5)]
    _publish(queue, objects[0], spec=ALL_SPECS[0])
    _publish(queue, objects[1], spec=ALL_SPECS[1])
    _publish(queue, objects[2], spec=ALL_SPECS[2])

    assert queue.try_reserve_one(ALL_SPECS[3]).status is AdmissionResult.QUEUE_FULL
    first = queue.claim_batch(max_entities=2)
    assert first is not None
    assert [entry.entity for entry in first.entries] == objects[:2]
    first.acknowledge_durable(
        (
            PersistQueueBatchDisposition.SQLITE_COMMITTED,
            PersistQueueBatchDisposition.SQLITE_COMMITTED,
        )
    )

    _publish(queue, objects[3], spec=ALL_SPECS[3])
    _publish(queue, objects[4], spec=ALL_SPECS[4])
    wrapped = queue.claim_batch(max_entities=10)
    assert wrapped is not None
    assert [entry.entity for entry in wrapped.entries] == objects[2:]
    assert [entry.spec for entry in wrapped.entries] == list(ALL_SPECS[2:])


# Unknown or malformed kind/version pairs fail without claiming a slot.
@pytest.mark.parametrize(
    "spec",
    (
        PersistQueueEntitySpec(PersistQueueEntityKind.PROFILE_ONLY, 0),
        PersistQueueEntitySpec(PersistQueueEntityKind.PROFILE_ONLY, 2),
        PersistQueueEntitySpec(PersistQueueEntityKind.PROFILE_ONLY, True),
        PersistQueueEntitySpec(PersistQueueEntityKind.PROFILE_ONLY, []),
        PersistQueueEntitySpec([], 1),
        PersistQueueEntitySpec(99, 1),  # type: ignore[arg-type]
        object(),
    ),
)
def test_unknown_entity_spec_is_an_interface_violation(spec: object) -> None:
    queue = PersistQueue(capacity_entities=1)
    _make_available(queue)

    with pytest.raises(PersistQueueInterfaceError, match="spec"):
        queue.try_reserve_one(spec)  # type: ignore[arg-type]

    assert queue.snapshot().reserved_entities == 0


# A structurally equivalent supported descriptor need not be a module singleton.
def test_equivalent_supported_entity_spec_is_accepted_without_normalization() -> None:
    queue = PersistQueue(capacity_entities=1)
    _make_available(queue)
    spec = PersistQueueEntitySpec(PersistQueueEntityKind.PROFILE_ONLY, 1)

    result = queue.try_reserve_one(spec)

    assert result.status is AdmissionResult.RESERVED
    assert result.reservation is not None
    assert result.reservation.spec is spec
    result.reservation.cancel()


# Reservation handles cannot be copied, reused or applied to another queue.
def test_reservation_handles_are_noncopyable_and_reject_stale_or_foreign_use() -> None:
    first_queue = PersistQueue(capacity_entities=1)
    second_queue = PersistQueue(capacity_entities=1)
    _make_available(first_queue)
    _make_available(second_queue)
    result = first_queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert result.reservation is not None
    reservation = result.reservation

    with pytest.raises(TypeError):
        copy.copy(reservation)
    with pytest.raises(TypeError):
        copy.deepcopy(reservation)
    with pytest.raises(TypeError):
        copy.copy(reservation.token)
    with pytest.raises(TypeError):
        copy.deepcopy(reservation.token)
    with pytest.raises(PersistQueueInterfaceError, match="foreign"):
        second_queue._publish_reservation(reservation, object())

    reservation.publish(object())
    with pytest.raises(PersistQueueInterfaceError, match="stale"):
        first_queue._publish_reservation(reservation, object())


# A basic mixed FIFO sequence agrees with the independent logical-list model.
def test_production_queue_matches_independent_model_for_basic_sequence() -> None:
    queue = PersistQueue(capacity_entities=2)
    model = ReferencePersistQueue(capacity_entities=2)
    _make_available(queue)
    model.set_available(True)

    production_reservations = []
    model_tokens = []
    for entity, spec in (("a", ALL_SPECS[0]), ("b", ALL_SPECS[4])):
        result = queue.try_reserve_one(spec)
        modeled = model.reserve(spec)
        assert result.status is modeled.status
        assert result.used_entities_before == modeled.used_entities_before
        assert result.reservation is not None and modeled.token is not None
        production_reservations.append(result.reservation)
        model_tokens.append(modeled.token)
        result.reservation.publish(entity)
        model.publish(modeled.token, entity)

    assert queue.try_reserve_one(ALL_SPECS[2]).status is model.reserve(
        ALL_SPECS[2]
    ).status

    batch = queue.claim_batch(max_entities=1)
    modeled_batch = model.claim(1)
    assert batch is not None and modeled_batch is not None
    assert [entry.entity for entry in batch.entries] == [
        entry.entity for entry in modeled_batch
    ]
    batch.acknowledge_durable((PersistQueueBatchDisposition.SQLITE_COMMITTED,))
    model.acknowledge((model_tokens[0],))

    snapshot = queue.snapshot()
    modeled_snapshot = model.snapshot()
    assert (
        snapshot.capacity_entities,
        snapshot.reserved_entities,
        snapshot.published_entities,
        snapshot.claimed_entities,
        snapshot.closed,
        snapshot.closed_and_drained,
    ) == (
        modeled_snapshot.capacity_entities,
        modeled_snapshot.reserved_entities,
        modeled_snapshot.published_entities,
        modeled_snapshot.claimed_entities,
        modeled_snapshot.closed,
        modeled_snapshot.closed_and_drained,
    )


# The queue public surface contains no normal-path codec or entity builder.
def test_queue_module_has_no_normal_entity_codec_or_builder_api() -> None:
    import cura_receiver.persist_queue as queue_module

    public_names = set(queue_module.__all__)
    assert not any("encode" in name.lower() or "decode" in name.lower() for name in public_names)
    assert not any("builder" in name.lower() for name in public_names)
    assert "Event" not in public_names


# Keeps one borrowed FIFO prefix stable while tail publication continues.
def test_batch_claim_is_bounded_frozen_and_exclusive() -> None:
    queue = PersistQueue(capacity_entities=5)
    _make_available(queue)
    payloads = [object() for _ in range(5)]
    tokens = [_publish(queue, payload) for payload in payloads[:4]]

    lease = queue.claim_batch(max_entities=2)
    assert lease is not None
    assert [entry.entity for entry in lease.entries] == payloads[:2]
    assert [entry.token for entry in lease.entries] == tokens[:2]
    assert queue.snapshot().claimed_entities == 2
    assert queue.snapshot().published_entities == 4
    with pytest.raises(TypeError):
        copy.copy(lease)
    with pytest.raises(TypeError):
        copy.deepcopy(lease)
    with pytest.raises(FrozenInstanceError):
        lease.entries[0].entity = object()  # type: ignore[misc]
    with pytest.raises(PersistQueueInterfaceError, match="active batch"):
        queue.claim_batch(max_entities=1)

    _publish(queue, payloads[4], spec=DIAGNOSTIC_V1_SPEC)
    assert [entry.entity for entry in lease.entries] == payloads[:2]
    assert queue.snapshot().published_entities == 5

    lease.release_for_retry()
    retry = queue.claim_batch(max_entities=5)
    assert retry is not None
    assert [entry.entity for entry in retry.entries] == payloads
    assert [entry.token for entry in retry.entries[:2]] == tokens[:2]


# Context exit releases an unacknowledged prefix without changing FIFO order.
def test_batch_context_releases_for_retry_after_exception() -> None:
    queue = PersistQueue(capacity_entities=2)
    _make_available(queue)
    payloads = (object(), object())
    tokens = tuple(_publish(queue, payload) for payload in payloads)

    with pytest.raises(RuntimeError, match="injected"):
        with queue.claim_batch(max_entities=2) as lease:
            assert lease is not None
            raise RuntimeError("injected")

    assert queue.snapshot().claimed_entities == 0
    retry = queue.claim_batch(max_entities=2)
    assert retry is not None
    assert tuple(entry.token for entry in retry.entries) == tokens
    assert tuple(entry.entity for entry in retry.entries) == payloads


# Invalid dispositions retain the complete active prefix; mixed durable results remove it.
def test_durable_acknowledgement_is_all_or_nothing() -> None:
    queue = PersistQueue(capacity_entities=2)
    _make_available(queue)
    _publish(queue, object())
    _publish(queue, object(), spec=DIAGNOSTIC_V1_SPEC)
    lease = queue.claim_batch(max_entities=2)
    assert lease is not None

    invalid_dispositions = (
        (PersistQueueBatchDisposition.SQLITE_COMMITTED,),
        [
            PersistQueueBatchDisposition.SQLITE_COMMITTED,
            PersistQueueBatchDisposition.QUARANTINED,
        ],
        (1, 2),
    )
    for invalid in invalid_dispositions:
        with pytest.raises(PersistQueueInterfaceError):
            lease.acknowledge_durable(invalid)  # type: ignore[arg-type]
        assert queue.snapshot().published_entities == 2
        assert queue.snapshot().claimed_entities == 2

    lease.acknowledge_durable(
        (
            PersistQueueBatchDisposition.SQLITE_COMMITTED,
            PersistQueueBatchDisposition.QUARANTINED,
        )
    )
    assert queue.snapshot().published_entities == 0
    assert queue.snapshot().claimed_entities == 0
    with pytest.raises(PersistQueueInterfaceError, match="stale"):
        lease.release_for_retry()
    with pytest.raises(PersistQueueInterfaceError, match="stale"):
        lease.acknowledge_durable(())


# A foreign lease and a corrupted head identity cannot release queue ownership.
def test_batch_lease_rejects_foreign_and_stale_prefix_transitions() -> None:
    first_queue = PersistQueue(capacity_entities=1)
    second_queue = PersistQueue(capacity_entities=1)
    _make_available(first_queue)
    _make_available(second_queue)
    _publish(first_queue, object())
    _publish(second_queue, object())
    first_lease = first_queue.claim_batch(max_entities=1)
    second_lease = second_queue.claim_batch(max_entities=1)
    assert first_lease is not None and second_lease is not None

    with pytest.raises(PersistQueueInterfaceError, match="foreign"):
        second_queue._release_lease(first_lease)
    assert first_queue.snapshot().claimed_entities == 1
    assert second_queue.snapshot().claimed_entities == 1

    original_token = first_queue._token_slots[first_queue._head]
    first_queue._token_slots[first_queue._head] = second_lease.entries[0].token
    with pytest.raises(PersistQueueInterfaceError, match="queue-head prefix"):
        first_lease.acknowledge_durable(
            (PersistQueueBatchDisposition.SQLITE_COMMITTED,)
        )
    assert first_queue.snapshot().claimed_entities == 1
    first_queue._token_slots[first_queue._head] = original_token
    first_lease.release_for_retry()


# Evidence failure is not a queue disposition and therefore retains the poisoned head.
def test_unencodable_poison_retains_the_claimed_head_and_following_entries() -> None:
    queue = PersistQueue(capacity_entities=2)
    _make_available(queue)
    poisoned = ProfileOnlyUnitV1(profile=["mutable"])  # type: ignore[arg-type]
    follower = object()
    poisoned_token = _publish(queue, poisoned)
    follower_token = _publish(queue, follower, spec=DIAGNOSTIC_V1_SPEC)
    lease = queue.claim_batch(max_entities=2)
    assert lease is not None

    with pytest.raises(QuarantineEvidenceEncodeError, match="unsupported"):
        encode_quarantine_evidence_v1(
            lease.entries[0].entity,
            spec=lease.entries[0].spec,
        )

    assert queue.snapshot().published_entities == 2
    assert queue.snapshot().claimed_entities == 2
    with pytest.raises(PersistQueueInterfaceError, match="active batch"):
        queue.claim_batch(max_entities=1)
    lease.release_for_retry()
    retained = queue.claim_batch(max_entities=2)
    assert retained is not None
    assert tuple(entry.token for entry in retained.entries) == (
        poisoned_token,
        follower_token,
    )


# A non-quarantinable incompatible head remains owned until an explicit durable action.
def test_incompatible_clock_head_has_no_bypass_or_arbitrary_removal_api() -> None:
    queue = PersistQueue(capacity_entities=2)
    _make_available(queue)
    clock_token = _publish(queue, object(), spec=CLOCK_OBSERVATION_V1_SPEC)
    follower_token = _publish(queue, object(), spec=PROFILE_ONLY_V1_SPEC)
    lease = queue.claim_batch(max_entities=2)
    assert lease is not None

    assert tuple(entry.token for entry in lease.entries) == (
        clock_token,
        follower_token,
    )
    assert not any(
        hasattr(queue, name)
        for name in ("pop", "popleft", "remove", "evict", "requeue", "skip")
    )
    assert queue.snapshot().claimed_entities == 2
    lease.release_for_retry()


# Invalid batch limits leave all published ownership unchanged.
@pytest.mark.parametrize("max_entities", (-1, 0, True, 1.5, "1"))
def test_claim_rejects_invalid_entity_limits_without_changing_state(
    max_entities: object,
) -> None:
    queue = PersistQueue(capacity_entities=1)
    _make_available(queue)
    _publish(queue, object())

    with pytest.raises(PersistQueueConfigurationError):
        queue.claim_batch(max_entities=max_entities)  # type: ignore[arg-type]
    assert queue.snapshot().published_entities == 1
    assert queue.snapshot().claimed_entities == 0


# Closure is idempotent, wakes waiters and drains only after reserved/published ownership ends.
def test_close_and_drain_cover_empty_reserved_and_published_states() -> None:
    empty_event = Event()
    empty = PersistQueue(capacity_entities=1, wake_event=empty_event)
    empty.close()
    assert empty_event.is_set()
    assert empty.snapshot().closed_and_drained
    empty_event.clear()
    empty.close()
    assert not empty_event.is_set()
    with pytest.raises(PersistQueueInterfaceError, match="closed"):
        empty.try_reserve_one(PROFILE_ONLY_V1_SPEC)

    published_event = Event()
    published = PersistQueue(capacity_entities=1, wake_event=published_event)
    _make_available(published)
    _publish(published, object())
    published_event.clear()
    published.close()
    assert published_event.is_set()
    assert not published.snapshot().closed_and_drained
    lease = published.claim_batch(max_entities=1)
    assert lease is not None
    published_event.clear()
    lease.acknowledge_durable((PersistQueueBatchDisposition.SQLITE_COMMITTED,))
    assert published_event.is_set()
    assert published.snapshot().closed_and_drained

    reserved_event = Event()
    reserved = PersistQueue(capacity_entities=1, wake_event=reserved_event)
    _make_available(reserved)
    result = reserved.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert result.reservation is not None
    reserved.close()
    reserved_event.clear()
    assert not reserved.snapshot().closed_and_drained
    result.reservation.cancel()
    assert reserved_event.is_set()
    assert reserved.snapshot().closed_and_drained

    publish_after_close_event = Event()
    publish_after_close = PersistQueue(
        capacity_entities=1,
        wake_event=publish_after_close_event,
    )
    _make_available(publish_after_close)
    result = publish_after_close.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert result.reservation is not None
    payload = object()
    publish_after_close.close()
    publish_after_close_event.clear()
    result.reservation.publish(payload)
    assert publish_after_close_event.is_set()
    assert not publish_after_close.snapshot().closed_and_drained
    final_lease = publish_after_close.claim_batch(max_entities=1)
    assert final_lease is not None
    assert final_lease.entries[0].entity is payload
    publish_after_close_event.clear()
    final_lease.acknowledge_durable(
        (PersistQueueBatchDisposition.SQLITE_COMMITTED,)
    )
    assert publish_after_close_event.is_set()
    assert publish_after_close.snapshot().closed_and_drained


# The queue only sets its shared wakeup hint; snapshots and claims never clear it.
def test_shared_wakeup_event_is_level_triggered_queue_hint() -> None:
    wake_event = Event()
    queue = PersistQueue(capacity_entities=1, wake_event=wake_event)
    _make_available(queue)
    assert not wake_event.is_set()

    _publish(queue, object())
    assert wake_event.is_set()
    queue.snapshot()
    queue.claim_batch(max_entities=1).release_for_retry()  # type: ignore[union-attr]
    assert wake_event.is_set()

    wake_event.clear()
    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    lease.release_for_retry()
    assert wake_event.is_set()


def _assert_model_snapshot(queue: PersistQueue, model: ReferencePersistQueue) -> None:
    actual = queue.snapshot()
    expected = model.snapshot()
    assert (
        actual.capacity_entities,
        actual.reserved_entities,
        actual.published_entities,
        actual.claimed_entities,
        actual.closed,
        actual.closed_and_drained,
    ) == (
        expected.capacity_entities,
        expected.reserved_entities,
        expected.published_entities,
        expected.claimed_entities,
        expected.closed,
        expected.closed_and_drained,
    )


# Exercises long mixed operation sequences against the independent logical model.
@settings(max_examples=100, deadline=None)
@given(
    st.lists(
        st.tuples(
            st.sampled_from(("availability", "offer", "claim", "resolve", "close")),
            st.integers(min_value=1, max_value=7),
            st.booleans(),
        ),
        min_size=1,
        max_size=120,
    )
)
def test_queue_state_machine_matches_independent_model(
    operations: list[tuple[str, int, bool]],
) -> None:
    queue = PersistQueue(capacity_entities=3)
    model = ReferencePersistQueue(capacity_entities=3)
    generation = 0
    next_payload = 0
    active_lease = None
    modeled_claim = None

    for action, amount, choice in operations:
        if action == "availability":
            generation += 1
            state = (
                PersistenceAdmissionState.AVAILABLE
                if choice
                else PersistenceAdmissionState.UNAVAILABLE_IO
            )
            queue.publish_admission_state(
                PersistenceAdmissionSnapshot(generation, state, generation)
            )
            model.set_available(choice)
        elif action == "offer":
            spec = ALL_SPECS[amount % len(ALL_SPECS)]
            if model.closed:
                with pytest.raises(PersistQueueInterfaceError):
                    queue.try_reserve_one(spec)
                with pytest.raises(ReferenceQueueViolation):
                    model.reserve(spec)
            else:
                result = queue.try_reserve_one(spec)
                modeled = model.reserve(spec)
                assert result.status is modeled.status
                assert result.used_entities_before == modeled.used_entities_before
                if result.reservation is not None:
                    assert modeled.token is not None
                    payload = (next_payload, amount)
                    next_payload += 1
                    if choice:
                        result.reservation.publish(payload)
                        model.publish(modeled.token, payload)
                    else:
                        result.reservation.cancel()
                        model.cancel(modeled.token)
        elif action == "claim":
            if model.snapshot().claimed_entities:
                with pytest.raises(PersistQueueInterfaceError):
                    queue.claim_batch(max_entities=amount)
                with pytest.raises(ReferenceQueueViolation):
                    model.claim(amount)
            else:
                active_lease = queue.claim_batch(max_entities=amount)
                modeled_claim = model.claim(amount)
                if modeled_claim is None:
                    assert active_lease is None
                else:
                    assert active_lease is not None
                    assert tuple(entry.entity for entry in active_lease.entries) == tuple(
                        entry.entity for entry in modeled_claim
                    )
        elif action == "resolve" and model.snapshot().claimed_entities:
            assert active_lease is not None and modeled_claim is not None
            if choice:
                active_lease.acknowledge_durable(
                    (PersistQueueBatchDisposition.SQLITE_COMMITTED,)
                    * len(active_lease.entries)
                )
                model.acknowledge(tuple(entry.token for entry in modeled_claim))
            else:
                active_lease.release_for_retry()
                model.release()
            active_lease = None
            modeled_claim = None
        elif action == "close":
            queue.close()
            model.close()

        _assert_model_snapshot(queue, model)
