"""Clock fences belong to each unreserved telemetry publication, including exits."""

from dataclasses import replace

import pytest

from cura_receiver.communicator_telemetry import CommunicatorTelemetry, EmissionSkipped
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.persist_queue import PersistenceAdmissionSnapshot
from cura_receiver.ports.ds3231 import Ds3231ReadResult, Ds3231ReadStatus
from cura_receiver.protocol_ingress import ProtocolIngressTerminalV1
from tests.host.test_communicator import composition
from tests.support.builders.persistence import _diagnostic, _radio_observation
from tests.support.builders.protocol_ingress import ingress_packet
from tests.support.fakes.ds3231 import FakeDs3231Control


def expiry(c):
    return min(c.time.schedule.trust_expires_at_monotonic_us, c.time.tracking_poll_deadline)


def drain_one(c):
    batch = c.queue.claim_batch(max_entities=1)
    assert batch is not None
    entity = batch.entries[0].entity
    batch.acknowledge_durable(completed_entities=1)
    return entity


def producer(c, telemetry, kind):
    if kind == 'health':
        return telemetry.health
    if kind == 'diagnostic':
        return lambda: telemetry.diagnostic(
            lambda episode, **identity: _diagnostic(sequence=identity['diagnostic_sequence']), object())
    c.communicator.retain_radio_observation(_radio_observation().snapshot)
    return telemetry.publish_observations


@pytest.mark.parametrize('kind', ['health', 'diagnostic', 'observation'])
@pytest.mark.parametrize('fault', ['queue_full', 'persistence_unavailable'])
def test_failed_clock_boundary_blocks_every_producer_without_retry(composition, kind, fault):
    c = composition(isolated_queue=True)
    telemetry = CommunicatorTelemetry(c.communicator, rtc=None)
    emit = producer(c, telemetry, kind)
    c.clock.advance_elapsed_us(expiry(c) - c.clock.now_monotonic_us())
    if fault == 'persistence_unavailable':
        c.queue.publish_admission_state(PersistenceAdmissionSnapshot(
            2, E.PersistenceAdmissionState.UNAVAILABLE_IO, c.clock.now_monotonic_us()))
    before = c.communicator.queue.counts
    assert emit() is None
    assert c.time.state.quality is E.SystemTimeQuality.UNTRUSTED
    assert c.time.pending_observation is not None
    after = c.communicator.queue.counts
    clock_index = E.PersistQueueEntityKind.CLOCK_OBSERVATION.value-1
    rejected = E.AdmissionResult.QUEUE_FULL if fault == 'queue_full' else E.AdmissionResult.PERSISTENCE_UNAVAILABLE
    assert after[clock_index][rejected.value] == before[clock_index][rejected.value]+1
    assert all(a == b for i, (a, b) in enumerate(zip(after, before)) if i != clock_index)
    assert emit() is None
    assert c.communicator.queue.counts == after
    assert telemetry.health_sequence == telemetry.diagnostic_sequence == 0
    assert c.communicator.observations_dropped == 0
    assert len(c.communicator.pending_radio_observations) == (kind == 'observation')
    # The normal time owner, rather than a producer retry loop, releases the
    # fence. The captured incident keeps its identity until actual publication.
    drain_one(c)
    if fault == 'persistence_unavailable':
        c.queue.publish_admission_state(PersistenceAdmissionSnapshot(
            3, E.PersistenceAdmissionState.AVAILABLE, c.clock.now_monotonic_us()))
    assert c.time.publish_pending().admission_result is E.AdmissionResult.RESERVED
    boundary = drain_one(c)
    assert boundary.system_time_quality is E.SystemTimeQuality.UNTRUSTED
    emit()
    entity = drain_one(c)
    if kind == 'health':
        assert entity.system_time_quality is E.SystemTimeQuality.UNTRUSTED
    elif kind == 'observation':
        assert entity.observation_sequence == 1
        assert not c.communicator.pending_radio_observations
    else:
        assert entity.diagnostic_sequence == 1


@pytest.mark.parametrize('kind', ['health', 'diagnostic', 'observation'])
def test_expired_clock_cannot_reserve_over_accepted_packet(composition, kind):
    c = composition(isolated_queue=True)
    drain_one(c)
    occurrence = c.communicator.ingress.begin(ingress_packet())
    telemetry = CommunicatorTelemetry(c.communicator, rtc=None)
    emit = producer(c, telemetry, kind)
    c.clock.advance_elapsed_us(expiry(c) - c.clock.now_monotonic_us())
    before = c.communicator.queue.counts
    assert emit() is None
    assert c.communicator.queue.counts == before
    assert c.time.state.quality is E.SystemTimeQuality.CHRONY_SYNCED
    assert c.time.pending_observation is None
    assert c.queue.snapshot().reserved_entities == 1
    # Completion keeps the already accepted data even though new telemetry
    # would first need a clock boundary and cannot own another reservation.
    completed = c.communicator.ingress.finalize(occurrence, ProtocolIngressTerminalV1(
        E.AckTxResult.SET_TX_FAILED, None, None, None, radio_state=E.RadioState.SHUTDOWN))
    assert completed.published_entity.candidate is not None
    assert drain_one(c) == completed.published_entity
    assert c.queue.snapshot().reserved_entities == 0


@pytest.mark.parametrize('probe', ['radio', 'rtc'])
def test_health_probe_expiry_publishes_boundary_before_original_sample(composition, monkeypatch, probe):
    c = composition(isolated_queue=True)
    drain_one(c)
    rtc = FakeDs3231Control()
    telemetry = CommunicatorTelemetry(c.communicator, rtc=rtc)
    deadline = expiry(c)
    c.clock.advance_elapsed_us(deadline - c.clock.now_monotonic_us() - 1)
    before = c.time.state
    transitions = c.time.time_quality_transition_count
    last_transition = c.time.last_time_quality_transition_monotonic_us
    if probe == 'radio':
        c.io.after_transfer = lambda command: c.clock.advance_elapsed_us(10)

    def read():
        start = c.clock.now_monotonic_us()
        if probe == 'rtc':
            c.clock.advance_elapsed_us(10)
        return Ds3231ReadResult(Ds3231ReadStatus.OK, start, c.clock.now_monotonic_us(), 1_800_000_000)

    rtc.read_results.append(read)
    published = []
    reserve = c.communicator.queue.try_reserve_one

    def collect(spec):
        # Acknowledge the boundary like a worker freeing capacity. Retain the
        # actual immutable entity so ordering and software sampling are checked.
        if spec.kind is E.PersistQueueEntityKind.RECEIVER_HEALTH_REQUEST:
            published.append(drain_one(c))
        return reserve(spec)

    monkeypatch.setattr(c.communicator.queue, 'try_reserve_one', collect)
    assert telemetry.health() is E.AdmissionResult.RESERVED
    published.append(drain_one(c))
    boundary, health = published
    assert boundary.sampled_at_monotonic_us == deadline
    assert boundary.system_time_quality is E.SystemTimeQuality.UNTRUSTED
    assert health.communicator_sampled_at_monotonic_us < deadline
    assert health.system_time_quality is before.quality
    assert health.time_quality_transition_count == transitions
    assert health.last_time_quality_transition_monotonic_us == last_transition
    assert c.time.state.quality is E.SystemTimeQuality.UNTRUSTED
    assert c.time.state.rtc_health is before.rtc_health
    assert len(rtc.calls) == 1 and c.queue.snapshot().reserved_entities == 0


@pytest.mark.parametrize('close', [False, True])
def test_health_lost_after_probe_is_counted_once_without_resampling(composition, close):
    c = composition(isolated_queue=True)  # The original clock row fills capacity.
    rtc = FakeDs3231Control()
    telemetry = CommunicatorTelemetry(c.communicator, rtc=rtc)
    deadline = expiry(c)
    c.clock.advance_elapsed_us(deadline - c.clock.now_monotonic_us() - 1)

    def read():
        start = c.clock.now_monotonic_us()
        c.clock.advance_elapsed_us(10)
        if close:
            c.queue.close()
        return Ds3231ReadResult(Ds3231ReadStatus.OK, start, c.clock.now_monotonic_us(), 1_800_000_000)

    rtc.read_results.append(read)
    expected = EmissionSkipped.CLOSED if close else None
    assert telemetry.health() is expected
    assert telemetry.health_sequence == c.communicator.observation_sequence == 1
    assert c.communicator.observations_dropped == 1
    assert len(rtc.calls) == 1 and c.queue.snapshot().reserved_entities == 0
    assert not c.communicator.pending_radio_observations
    counts = c.communicator.queue.counts
    assert telemetry.health() is expected
    assert c.communicator.queue.counts == counts
    assert c.communicator.observations_dropped == 1 and len(rtc.calls) == 1


def test_diagnostic_construction_crossing_expiry_is_fenced(composition):
    c = composition(isolated_queue=True)
    drain_one(c)
    telemetry = CommunicatorTelemetry(c.communicator, rtc=None)
    deadline = expiry(c)

    def factory(episode, **identity):
        c.clock.advance_elapsed_us(deadline - c.clock.now_monotonic_us())
        return replace(_diagnostic(sequence=identity['diagnostic_sequence']), sampled_at_monotonic_us=deadline)

    # With one slot the boundary wins admission. The diagnostic's own attempt
    # then reports ordinary pressure, leaving no reservation or recursive work.
    assert telemetry.diagnostic(factory, object()) is E.AdmissionResult.QUEUE_FULL
    boundary = drain_one(c)
    assert boundary.system_time_quality is E.SystemTimeQuality.UNTRUSTED
    assert boundary.sampled_at_monotonic_us == deadline
    assert c.queue.snapshot().reserved_entities == 0


def test_each_deferred_observation_rechecks_clock_before_its_reservation(composition, monkeypatch):
    c = composition(isolated_queue=True)
    drain_one(c)
    telemetry = CommunicatorTelemetry(c.communicator, rtc=None)
    first = c.communicator.retain_radio_observation(_radio_observation().snapshot)
    second = c.communicator.retain_radio_observation(_radio_observation().snapshot)
    deadline = expiry(c)
    reserve = c.communicator.queue.try_reserve_one

    def cross_expiry(spec):
        result = reserve(spec)
        if spec.kind is E.PersistQueueEntityKind.RADIO_OBSERVATION:
            c.clock.advance_elapsed_us(deadline - c.clock.now_monotonic_us())
        return result

    monkeypatch.setattr(c.communicator.queue, 'try_reserve_one', cross_expiry)
    assert telemetry.publish_observations() is None
    assert drain_one(c).observation_sequence == first
    assert [o.observation_sequence for o in c.communicator.pending_radio_observations] == [second]
    assert c.communicator.observations_dropped == 0
    assert c.time.pending_observation.sampled_at_monotonic_us == deadline
    assert c.queue.snapshot().reserved_entities == 0
