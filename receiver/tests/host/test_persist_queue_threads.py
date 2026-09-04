from __future__ import annotations

from threading import Barrier, Event

import pytest

from cura_receiver.generated.receiver_enums_generated import (
    AdmissionResult,
    PersistenceAdmissionState,
)
from cura_receiver.persist_queue import (
    PersistQueue,
    PersistQueueBatchDisposition,
    PersistenceAdmissionSnapshot,
)
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC
from tests.support.coordination.threads import (
    join_checked_threads,
    start_checked_threads,
)


DEADLOCK_TIMEOUT_SECONDS = 5.0


def _wait(event: Event) -> None:
    assert event.wait(DEADLOCK_TIMEOUT_SECONDS), "named test boundary timed out"


def _arrive(barrier: Barrier) -> None:
    barrier.wait(DEADLOCK_TIMEOUT_SECONDS)


def _make_available(queue: PersistQueue) -> None:
    queue.publish_admission_state(
        PersistenceAdmissionSnapshot(
            generation=1,
            state=PersistenceAdmissionState.AVAILABLE,
            changed_at_monotonic_us=1,
        )
    )


# Covers both lock serialization orders for admission publication and reservation.
@pytest.mark.parametrize("publication_first", (False, True))
def test_reservation_and_admission_publication_have_one_atomic_order(
    publication_first: bool,
) -> None:
    queue = PersistQueue(capacity_entities=1)
    both_workers_ready = Barrier(3)
    first_operation_completed = Event()
    results = []

    def communicator() -> None:
        _arrive(both_workers_ready)
        if publication_first:
            _wait(first_operation_completed)
        results.append(queue.try_reserve_one(PROFILE_ONLY_V1_SPEC))
        if not publication_first:
            first_operation_completed.set()

    def persistence() -> None:
        _arrive(both_workers_ready)
        if not publication_first:
            _wait(first_operation_completed)
        _make_available(queue)
        if publication_first:
            first_operation_completed.set()

    threads = start_checked_threads(
        (("communicator-admission", communicator), ("persistence-state", persistence))
    )
    _arrive(both_workers_ready)
    join_checked_threads(threads)

    assert len(results) == 1
    result = results[0]
    if publication_first:
        assert result.status is AdmissionResult.RESERVED
        assert result.admission_snapshot.generation == 1
        assert result.reservation is not None
        result.reservation.cancel()
    else:
        assert result.status is AdmissionResult.PERSISTENCE_UNAVAILABLE
        assert result.admission_snapshot.generation == 0
        assert result.reservation is None
    assert queue.snapshot().admission_snapshot.generation == 1


# Covers publication before and after the consumer's first nonblocking claim.
@pytest.mark.parametrize("publication_first", (False, True))
def test_publication_and_claim_have_one_visibility_order(publication_first: bool) -> None:
    queue = PersistQueue(capacity_entities=1)
    _make_available(queue)
    result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert result.reservation is not None
    payload = object()
    both_workers_ready = Barrier(3)
    first_operation_completed = Event()
    claimed = []

    def communicator() -> None:
        _arrive(both_workers_ready)
        if not publication_first:
            _wait(first_operation_completed)
        result.reservation.publish(payload)
        if publication_first:
            first_operation_completed.set()

    def persistence() -> None:
        _arrive(both_workers_ready)
        if publication_first:
            _wait(first_operation_completed)
        claimed.append(queue.claim_batch(max_entities=1))
        if not publication_first:
            first_operation_completed.set()

    threads = start_checked_threads(
        (("communicator-publish", communicator), ("persistence-claim", persistence))
    )
    _arrive(both_workers_ready)
    join_checked_threads(threads)

    lease = claimed[0]
    if publication_first:
        assert lease is not None
    else:
        assert lease is None
        lease = queue.claim_batch(max_entities=1)
        assert lease is not None
    assert lease.entries[0].entity is payload
    lease.acknowledge_durable((PersistQueueBatchDisposition.SQLITE_COMMITTED,))


# Covers capacity observation immediately before and after durable acknowledgement.
@pytest.mark.parametrize("acknowledgement_first", (False, True))
def test_acknowledgement_and_new_reservation_have_one_capacity_order(
    acknowledgement_first: bool,
) -> None:
    queue = PersistQueue(capacity_entities=1)
    _make_available(queue)
    first = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert first.reservation is not None
    first.reservation.publish(object())
    lease = queue.claim_batch(max_entities=1)
    assert lease is not None
    both_workers_ready = Barrier(3)
    first_operation_completed = Event()
    results = []

    def communicator() -> None:
        _arrive(both_workers_ready)
        if acknowledgement_first:
            _wait(first_operation_completed)
        results.append(queue.try_reserve_one(PROFILE_ONLY_V1_SPEC))
        if not acknowledgement_first:
            first_operation_completed.set()

    def persistence() -> None:
        _arrive(both_workers_ready)
        if not acknowledgement_first:
            _wait(first_operation_completed)
        lease.acknowledge_durable((PersistQueueBatchDisposition.SQLITE_COMMITTED,))
        if acknowledgement_first:
            first_operation_completed.set()

    threads = start_checked_threads(
        (("communicator-reserve", communicator), ("persistence-ack", persistence))
    )
    _arrive(both_workers_ready)
    join_checked_threads(threads)

    result = results[0]
    if acknowledgement_first:
        assert result.status is AdmissionResult.RESERVED
        assert result.reservation is not None
        result.reservation.cancel()
    else:
        assert result.status is AdmissionResult.QUEUE_FULL
        retry = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert retry.status is AdmissionResult.RESERVED
        assert retry.reservation is not None
        retry.reservation.cancel()


# Retry release preserves the head while the producer publishes at the tail.
def test_release_for_retry_and_tail_publication_preserve_fifo() -> None:
    queue = PersistQueue(capacity_entities=3)
    _make_available(queue)
    for value in (0, 1):
        result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert result.reservation is not None
        result.reservation.publish(value)
    first_lease = queue.claim_batch(max_entities=2)
    assert first_lease is not None
    both_workers_ready = Barrier(3)
    prefix_released = Event()
    tail_published = Event()
    retried = []

    def communicator() -> None:
        _arrive(both_workers_ready)
        _wait(prefix_released)
        result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert result.reservation is not None
        result.reservation.publish(2)
        tail_published.set()

    def persistence() -> None:
        _arrive(both_workers_ready)
        first_lease.release_for_retry()
        prefix_released.set()
        _wait(tail_published)
        retried.append(queue.claim_batch(max_entities=3))

    threads = start_checked_threads(
        (("communicator-tail", communicator), ("persistence-retry", persistence))
    )
    _arrive(both_workers_ready)
    join_checked_threads(threads)

    retry_lease = retried[0]
    assert retry_lease is not None
    assert tuple(entry.entity for entry in retry_lease.entries) == (0, 1, 2)
    retry_lease.acknowledge_durable(
        (PersistQueueBatchDisposition.SQLITE_COMMITTED,) * 3
    )


# Closure permits the producer's existing reservation to publish and the consumer to drain it.
def test_closure_publication_and_drain_do_not_lose_the_reserved_entity() -> None:
    queue = PersistQueue(capacity_entities=1)
    _make_available(queue)
    result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert result.reservation is not None
    producer_closed = Event()
    entity_published = Event()

    def communicator() -> None:
        queue.close()
        producer_closed.set()
        result.reservation.publish("final")
        entity_published.set()

    def persistence() -> None:
        _wait(producer_closed)
        assert not queue.snapshot().closed_and_drained
        _wait(entity_published)
        lease = queue.claim_batch(max_entities=1)
        assert lease is not None
        assert lease.entries[0].entity == "final"
        lease.acknowledge_durable(
            (PersistQueueBatchDisposition.SQLITE_COMMITTED,)
        )

    threads = start_checked_threads(
        (("communicator-close", communicator), ("persistence-drain", persistence))
    )
    join_checked_threads(threads)
    assert queue.snapshot().closed_and_drained


# Publication before event clearing may erase the hint, but the mandatory recheck sees work.
def test_clear_then_recheck_prevents_lost_wakeup_when_publication_precedes_clear() -> None:
    wake_event = Event()
    queue = PersistQueue(capacity_entities=1, wake_event=wake_event)
    _make_available(queue)
    initially_empty_observed = Event()
    entity_published = Event()

    def communicator() -> None:
        _wait(initially_empty_observed)
        result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert result.reservation is not None
        result.reservation.publish("work")
        entity_published.set()

    def persistence() -> None:
        assert queue.claim_batch(max_entities=1) is None
        initially_empty_observed.set()
        _wait(entity_published)
        assert wake_event.is_set()
        wake_event.clear()
        lease = queue.claim_batch(max_entities=1)
        assert lease is not None
        lease.acknowledge_durable(
            (PersistQueueBatchDisposition.SQLITE_COMMITTED,)
        )

    threads = start_checked_threads(
        (("communicator-wakeup", communicator), ("persistence-recheck", persistence))
    )
    join_checked_threads(threads)


# Publication after event clearing sets the hint and is also visible to the recheck.
def test_clear_then_recheck_prevents_lost_wakeup_when_publication_follows_clear() -> None:
    wake_event = Event()
    queue = PersistQueue(capacity_entities=1, wake_event=wake_event)
    _make_available(queue)
    wakeup_cleared = Event()
    entity_published = Event()

    def communicator() -> None:
        _wait(wakeup_cleared)
        result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
        assert result.reservation is not None
        result.reservation.publish("work")
        entity_published.set()

    def persistence() -> None:
        assert queue.claim_batch(max_entities=1) is None
        wake_event.clear()
        wakeup_cleared.set()
        _wait(entity_published)
        assert wake_event.is_set()
        lease = queue.claim_batch(max_entities=1)
        assert lease is not None
        lease.acknowledge_durable(
            (PersistQueueBatchDisposition.SQLITE_COMMITTED,)
        )

    threads = start_checked_threads(
        (("communicator-wakeup", communicator), ("persistence-recheck", persistence))
    )
    join_checked_threads(threads)


# Sustained real-thread wraparound preserves every identity without sleeps or deadlock.
def test_sustained_spsc_wraparound_has_stable_entity_accounting() -> None:
    item_count = 3_000
    wake_event = Event()
    space_available = Event()
    queue = PersistQueue(capacity_entities=7, wake_event=wake_event)
    _make_available(queue)
    received: list[int] = []

    def communicator() -> None:
        for value in range(item_count):
            while True:
                result = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
                if result.status is AdmissionResult.RESERVED:
                    assert result.reservation is not None
                    result.reservation.publish(value)
                    break
                assert result.status is AdmissionResult.QUEUE_FULL
                space_available.clear()
                retry = queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
                if retry.status is AdmissionResult.RESERVED:
                    assert retry.reservation is not None
                    retry.reservation.publish(value)
                    break
                assert retry.status is AdmissionResult.QUEUE_FULL
                _wait(space_available)
        queue.close()

    def persistence() -> None:
        next_expected = 0
        while True:
            lease = queue.claim_batch(max_entities=3)
            if lease is None:
                if queue.snapshot().closed_and_drained:
                    break
                wake_event.clear()
                lease = queue.claim_batch(max_entities=3)
                if lease is None:
                    if queue.snapshot().closed_and_drained:
                        break
                    _wait(wake_event)
                    continue
            for entry in lease.entries:
                assert entry.entity == next_expected
                received.append(entry.entity)  # type: ignore[arg-type]
                next_expected += 1
            lease.acknowledge_durable(
                (PersistQueueBatchDisposition.SQLITE_COMMITTED,)
                * len(lease.entries)
            )
            space_available.set()
        assert next_expected == item_count

    threads = start_checked_threads(
        (("communicator-sustained", communicator), ("persistence-sustained", persistence))
    )
    join_checked_threads(threads, timeout_seconds=10.0)

    assert received == list(range(item_count))
    snapshot = queue.snapshot()
    assert snapshot.closed_and_drained
    assert snapshot.reserved_entities == 0
    assert snapshot.published_entities == 0
    assert snapshot.claimed_entities == 0
