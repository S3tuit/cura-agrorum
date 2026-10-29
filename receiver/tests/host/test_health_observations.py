"""Health probes preserve evidence without altering operational policy."""
from types import SimpleNamespace

import pytest

from cura_receiver.application_settings import ApplicationSettings
from cura_receiver.communicator_telemetry import CommunicatorTelemetry
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.health_observations import read_rtc_for_health
from cura_receiver.ports.ds3231 import Ds3231ReadResult, Ds3231ReadStatus as R
from cura_receiver.ports.radio import Dio1Edge
from tests.host.test_communicator import composition
from tests.support.fakes.ds3231 import FakeDs3231Control


@pytest.mark.parametrize('status', list(R))
def test_probe_once_retains_status_errno_times_without_changing_runtime(composition, status):
    c = composition(isolated_queue=True)
    rtc = FakeDs3231Control()
    now = c.clock.now_monotonic_us()
    result = Ds3231ReadResult(status, now, now + 5,
        1_800_000_000 if status is R.OK else None, 5 if status is R.IO_ERROR else None)
    rtc.read_results.append(result)
    before = (c.time.state, c.time.sample, c.time.pending_observation, c.airtime.owner.state,
              c.time.rtc_health_transition_count, c.time.time_quality_transition_count)
    evidence = read_rtc_for_health(c.time, rtc)
    assert evidence == (E.RtcReadStatus[status.name], result.os_errno, now, now + 5, result.rtc_utc_s)
    assert len(rtc.calls) == 1
    assert before == (c.time.state, c.time.sample, c.time.pending_observation, c.airtime.owner.state,
                      c.time.rtc_health_transition_count, c.time.time_quality_transition_count)


def test_pending_durable_mutation_and_expired_budget_do_not_probe(composition):
    c = composition()
    rtc = FakeDs3231Control()
    owner = c.time.state_owner
    c.time.state_owner = SimpleNamespace(pending=object())
    assert read_rtc_for_health(c.time, rtc)[0] is E.RtcReadStatus.NOT_ATTEMPTED
    c.time.state_owner = owner
    assert read_rtc_for_health(c.time, rtc, deadline=0)[0] is E.RtcReadStatus.NOT_ATTEMPTED
    assert not rtc.calls


def test_health_samples_before_reservation_and_preserves_mismatch(composition):
    c = composition(isolated_queue=True)
    c.queue.claim_batch(max_entities=1).acknowledge_durable(completed_entities=1)
    c.io.status = 0x24  # Hardware silently left RX; software still expects RX.
    rtc = FakeDs3231Control()
    def read():
        assert c.queue.snapshot().reserved_entities == 0
        now = c.clock.now_monotonic_us()
        return Ds3231ReadResult(R.OK, now, now, 1_800_000_000)
    rtc.read_results.append(read)
    before = (c.radio.state, c.radio.counters, c.time.state, c.airtime.owner.state)
    assert CommunicatorTelemetry(c.communicator, rtc=rtc).health() is E.AdmissionResult.RESERVED
    batch = c.queue.claim_batch(max_entities=1)
    entity = batch.entries[0].entity
    assert entity.radio_observation.snapshot.software_state is E.RadioState.RX_SINGLE
    assert entity.radio_observation.snapshot.chip_mode is E.RadioChipMode.STBY_RC
    assert entity.consistency is E.HealthConsistency.INCONSISTENT
    assert entity.persist_queue_admission_counts[2] == (0, 0, 0)
    assert before == (c.radio.state, c.radio.counters, c.time.state, c.airtime.owner.state)
    batch.acknowledge_durable(completed_entities=1)


def test_full_queue_leaves_no_separate_health_observation(composition):
    c = composition(isolated_queue=True)
    assert CommunicatorTelemetry(c.communicator, rtc=None).health() is E.AdmissionResult.QUEUE_FULL
    assert c.communicator.observations_dropped == 1
    assert not c.communicator.pending_radio_observations
    assert c.queue.snapshot().reserved_entities == 0


def test_queued_zero_irq_event_remains_pending_and_health_is_indeterminate(composition):
    c = composition(isolated_queue=True)
    c.queue.claim_batch(max_entities=1).acknowledge_durable(completed_entities=1)
    edge = Dio1Edge(c.clock.now_monotonic_us() * 1000, 1)
    c.io.edges.append(edge)
    rtc = FakeDs3231Control()
    now = c.clock.now_monotonic_us()
    rtc.read_results.append(Ds3231ReadResult(R.OK, now, now, 1_800_000_000))
    CommunicatorTelemetry(c.communicator, rtc=rtc).health()
    batch = c.queue.claim_batch(max_entities=1)
    entity = batch.entries[0].entity
    assert entity.radio_observation.snapshot.gpio_event_pending is True
    assert entity.radio_observation.snapshot.irq_status == 0
    assert entity.consistency is E.HealthConsistency.INDETERMINATE
    assert list(c.io.edges) == [edge]
    batch.acknowledge_durable(completed_entities=1)


def test_pilot_interval_is_five_minutes():
    assert ApplicationSettings(persistence_startup_budget_us=12_000_000).health_interval_us == 300_000_000
