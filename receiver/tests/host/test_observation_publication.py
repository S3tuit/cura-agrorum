from cura_receiver.communicator_telemetry import CommunicatorTelemetry
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.protocol_ingress import ProtocolIngressTerminalV1
from tests.host.test_communicator import composition
from tests.support.builders.persistence import _observation, _radio_observation
from tests.support.builders.protocol_ingress import ingress_packet


def test_nested_capture_never_reserves_during_accepted_exchange(composition):
    c = composition()
    telemetry = CommunicatorTelemetry(c.communicator, rtc=None)
    occurrence = c.communicator.ingress.begin(ingress_packet())
    before = c.communicator.queue.counts
    c.communicator.retain_radio_observation(_radio_observation().snapshot)
    telemetry.publish_observations()
    assert c.communicator.queue.counts == before
    assert c.queue.snapshot().reserved_entities == 1
    assert len(c.communicator.pending_radio_observations) == 1
    c.communicator.ingress.finalize(occurrence, ProtocolIngressTerminalV1(
        E.AckTxResult.SET_TX_FAILED, None, None, None, radio_state=E.RadioState.SHUTDOWN))
    telemetry.publish_observations()
    assert not c.communicator.pending_radio_observations
    assert c.queue.snapshot().reserved_entities == 0
    assert c.communicator.queue.counts[E.PersistQueueEntityKind.RADIO_OBSERVATION.value-1] == (1, 0, 0)


def test_clock_gated_history_is_bounded_and_counts_evicted_observations(composition):
    c = composition()
    telemetry = CommunicatorTelemetry(c.communicator, rtc=None)
    c.time.pending_observation = _observation()
    before = c.communicator.queue.counts
    for _ in range(20):
        c.communicator.retain_radio_observation(_radio_observation().snapshot)
        telemetry.publish_observations()
    assert c.communicator.queue.counts == before
    assert c.communicator.observations_dropped == 4
    assert [x.observation_sequence for x in c.communicator.pending_radio_observations] == list(range(5, 21))
    c.time.pending_observation = None
    telemetry.publish_observations()
    assert not c.communicator.pending_radio_observations


def test_full_queue_discards_observation_without_recursive_diagnostics(composition):
    c = composition(isolated_queue=True)  # initial clock observation occupies the slot
    telemetry = CommunicatorTelemetry(c.communicator, rtc=None)
    c.communicator.retain_radio_observation(_radio_observation().snapshot)
    before = c.communicator.queue.counts[E.PersistQueueEntityKind.DIAGNOSTIC.value-1]
    telemetry.publish_observations()
    assert c.communicator.observations_dropped == 1
    assert not c.communicator.pending_radio_observations
    assert c.communicator.queue.counts[E.PersistQueueEntityKind.DIAGNOSTIC.value-1] == before


def test_queue_closure_discards_bounded_capture_without_admission(composition):
    c = composition(isolated_queue=True)
    telemetry = CommunicatorTelemetry(c.communicator, rtc=None)
    c.communicator.retain_radio_observation(_radio_observation().snapshot)
    c.queue.close()
    before = c.communicator.queue.counts
    telemetry.publish_observations()
    assert c.communicator.observations_dropped == 1
    assert not c.communicator.pending_radio_observations
    assert c.communicator.queue.counts == before
