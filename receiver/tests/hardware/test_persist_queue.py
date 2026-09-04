from __future__ import annotations

from collections.abc import Callable
from threading import Event
from time import perf_counter_ns

import pytest

from cura_receiver.generated.receiver_enums_generated import (
    AdmissionResult,
    PersistenceAdmissionState,
)
from cura_receiver.persist_queue import (
    PERSIST_QUEUE_MAX_ENTITIES,
    PersistQueue,
    PersistQueueBatchDisposition,
    PersistenceAdmissionSnapshot,
)
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC
from tests.support.coordination.threads import (
    join_checked_threads,
    start_checked_threads,
)


pytestmark = pytest.mark.hardware
DEADLOCK_TIMEOUT_SECONDS = 10.0


def _wait(event: Event) -> None:
    assert event.wait(DEADLOCK_TIMEOUT_SECONDS), "target queue boundary timed out"


def _make_available(queue: PersistQueue) -> None:
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=1,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=1,
        )
    )


def _ack_all(queue: PersistQueue, max_entities: int) -> tuple[object, ...]:
    lease = queue.claim_batch(max_entities=max_entities)
    assert lease is not None
    entities = tuple(entry.entity for entry in lease.entries)
    lease.acknowledge_durable(
        (PersistQueueBatchDisposition.SQLITE_COMMITTED,) * len(lease.entries)
    )
    return entities


# Constructs and exhausts the exact production backing without inspecting object memory.
def test_production_queue_preallocates_and_uses_all_500_slots() -> None:
    queue = PersistQueue()
    _make_available(queue)

    assert queue.capacity_entities == PERSIST_QUEUE_MAX_ENTITIES == 500
    assert tuple(
        len(storage)
        for storage in (
            queue._payload_slots,
            queue._spec_slots,
            queue._token_slots,
        )
    ) == (500, 500, 500)
    backing_identities = (
        id(queue._payload_slots),
        id(queue._spec_slots),
        id(queue._token_slots),
    )

    for value in range(500):
        result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert result.status is AdmissionResult.RESERVED
        assert result.reservation is not None
        result.reservation.publish(value)
    assert queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).status is AdmissionResult.QUEUE_FULL
    assert _ack_all(queue, 500) == tuple(range(500))
    assert backing_identities == (
        id(queue._payload_slots),
        id(queue._spec_slots),
        id(queue._token_slots),
    )


# Releasing half a full target queue restores admission without eviction or reorder.
def test_target_queue_pressure_recovers_after_consumer_acknowledgement() -> None:
    queue = PersistQueue()
    _make_available(queue)
    for value in range(500):
        result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert result.reservation is not None
        result.reservation.publish(value)
    assert queue.try_reserve_one(PROFILE_ONLY_V1_SPEC).status is AdmissionResult.QUEUE_FULL

    consumer_ready = Event()
    allow_acknowledgement = Event()
    capacity_released = Event()

    def persistence() -> None:
        lease = queue.claim_batch(max_entities=250)
        assert lease is not None
        assert tuple(entry.entity for entry in lease.entries) == tuple(range(250))
        consumer_ready.set()
        _wait(allow_acknowledgement)
        lease.acknowledge_durable(
            (PersistQueueBatchDisposition.SQLITE_COMMITTED,) * 250
        )
        capacity_released.set()

    threads = start_checked_threads((("persistence-pressure", persistence),))
    _wait(consumer_ready)
    allow_acknowledgement.set()
    _wait(capacity_released)

    result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert result.status is AdmissionResult.RESERVED
    assert result.reservation is not None
    result.reservation.publish(500)
    join_checked_threads(threads, timeout_seconds=DEADLOCK_TIMEOUT_SECONDS)
    assert _ack_all(queue, 500) == tuple(range(250, 501))


def _percentile(samples: list[int], numerator: int, denominator: int) -> int:
    ordered = sorted(samples)
    index = min(len(ordered) - 1, (len(ordered) * numerator) // denominator)
    return ordered[index]


# Sustained target contention characterizes lock latency while forcing ring wrap and drain.
def test_target_sustained_handoff_wraparound_latency_and_close_drain(
    record_property: Callable[[str, object], None],
) -> None:
    item_count = 20_000
    wake_event = Event()
    capacity_released = Event()
    queue = PersistQueue(wake_event=wake_event)
    _make_available(queue)
    received: list[int] = []
    reservation_call_ns: list[int] = []
    publication_call_ns: list[int] = []
    claim_call_ns: list[int] = []
    acknowledgement_call_ns: list[int] = []

    def communicator() -> None:
        for value in range(item_count):
            while True:
                started = perf_counter_ns()
                result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
                reservation_call_ns.append(perf_counter_ns() - started)
                if result.status is AdmissionResult.RESERVED:
                    assert result.reservation is not None
                    started = perf_counter_ns()
                    result.reservation.publish(value)
                    publication_call_ns.append(perf_counter_ns() - started)
                    break
                assert result.status is AdmissionResult.QUEUE_FULL
                capacity_released.clear()
                started = perf_counter_ns()
                retry = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
                reservation_call_ns.append(perf_counter_ns() - started)
                if retry.status is AdmissionResult.RESERVED:
                    assert retry.reservation is not None
                    started = perf_counter_ns()
                    retry.reservation.publish(value)
                    publication_call_ns.append(perf_counter_ns() - started)
                    break
                assert retry.status is AdmissionResult.QUEUE_FULL
                _wait(capacity_released)
        queue.close()

    def persistence() -> None:
        next_expected = 0
        while True:
            started = perf_counter_ns()
            lease = queue.claim_batch(max_entities=64)
            claim_call_ns.append(perf_counter_ns() - started)
            if lease is None:
                if queue.snapshot().closed_and_drained:
                    break
                wake_event.clear()
                lease = queue.claim_batch(max_entities=64)
                if lease is None:
                    if queue.snapshot().closed_and_drained:
                        break
                    _wait(wake_event)
                    continue
            for entry in lease.entries:
                assert entry.entity == next_expected
                received.append(entry.entity)  # type: ignore[arg-type]
                next_expected += 1
            started = perf_counter_ns()
            lease.acknowledge_durable(
                (PersistQueueBatchDisposition.SQLITE_COMMITTED,)
                * len(lease.entries)
            )
            acknowledgement_call_ns.append(perf_counter_ns() - started)
            capacity_released.set()
        assert next_expected == item_count

    threads = start_checked_threads(
        (("communicator-target", communicator), ("persistence-target", persistence))
    )
    join_checked_threads(threads, timeout_seconds=30.0)

    assert received == list(range(item_count))
    assert queue.snapshot().closed_and_drained
    for label, samples in (
        ("reservation_call_ns", reservation_call_ns),
        ("publication_call_ns", publication_call_ns),
        ("claim_call_ns", claim_call_ns),
        ("acknowledgement_call_ns", acknowledgement_call_ns),
    ):
        assert samples
        record_property(f"{label}_p50", _percentile(samples, 50, 100))
        record_property(f"{label}_p95", _percentile(samples, 95, 100))
        record_property(f"{label}_p99", _percentile(samples, 99, 100))
        record_property(f"{label}_max", max(samples))
