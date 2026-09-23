"""Real queue results counted at the communicator producer boundary."""

import pytest
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.persist_queue import PersistQueue, PersistenceAdmissionSnapshot, PersistQueueInterfaceError
from cura_receiver.persist_queue_entities import PROFILE_ONLY_V1_SPEC, RECEIVER_HEALTH_REQUEST_V1_SPEC
from cura_receiver.producer_admission import ProducerAdmission
from tests.host.test_runtime_time import runtime, sample, tracking
from tests.support.fakes.chrony import FakeChronyControl


def test_two_clock_publications_in_one_poll_are_both_counted():
    rt, clock, kernel, queue = runtime()
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    queue.claim_batch(max_entities=100).acknowledge_durable(completed_entities=1)
    before = rt.queue.counts
    clock.advance_elapsed_us(60_000_000)
    sample(rt, kernel)
    chrony = FakeChronyControl()
    chrony.tracking_results.append(tracking(rt))
    result = rt.poll_chrony(chrony)
    assert result.observation.system_time_quality is E.SystemTimeQuality.NETWORK_SYNCED
    entries = queue.claim_batch(max_entities=100).entries
    assert [e.entity.system_time_quality for e in entries] == [E.SystemTimeQuality.UNTRUSTED, E.SystemTimeQuality.NETWORK_SYNCED]
    assert before[4] == (1, 0, 0)
    assert rt.queue.counts[4] == (3, 0, 0)
    assert rt.time_quality_transition_count == 3


def test_cancelled_reservation_counts_and_interface_error_does_not():
    queue = PersistQueue(capacity_entities=1)
    producer = ProducerAdmission(queue)
    unavailable = producer.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert unavailable.status is E.AdmissionResult.PERSISTENCE_UNAVAILABLE
    queue.publish_admission_state(PersistenceAdmissionSnapshot(1, E.PersistenceAdmissionState.AVAILABLE, 0))
    reserved = producer.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    frozen = producer.counts
    with pytest.raises(PersistQueueInterfaceError):
        producer.try_reserve_one(PROFILE_ONLY_V1_SPEC)
    assert producer.counts == frozen
    reserved.reservation.cancel()
    assert producer.counts[1] == (1, 1, 0)
    # A health producer sees its own admission in the snapshot it will publish.
    health = producer.try_reserve_one(RECEIVER_HEALTH_REQUEST_V1_SPEC)
    assert producer.counts[2] == (1, 0, 0)
    assert frozen[2] == (0, 0, 0)
    health.reservation.cancel()


def test_full_queue_and_saturation():
    rt, _, kernel, _ = runtime(capacity=1)
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    sample(rt, kernel)
    rt.sample_network(tracking(rt))
    assert rt.queue.counts[4] == (1, 0, 1)
    rt.queue._counts[4][2] = (1 << 64) - 1
    rt.publish_pending()
    assert rt.queue.counts[4][2] == (1 << 64) - 1
