"""Approved radio completion facts through real ingress, queue and SQLite."""

from dataclasses import replace

import pytest

from cura_receiver.generated.receiver_enums_generated import AckTxResult, RadioState
from cura_receiver.ordinary_persistence import OrdinaryBatchCommitOutcome
from cura_receiver.protocol_ingress import (
    ProtocolIngress,
    ProtocolIngressInterfaceError,
    ProtocolIngressTerminalV1,
)
from tests.support.builders.persistence import INSTANCE
from tests.support.builders.protocol_ingress import (
    REVIEWED_ACCEPTED_ACK,
    REVIEWED_CURRENT_FRAME,
    REVIEWED_NODE_ID,
    REVIEWED_NODE_KEY,
    ingress_packet,
)
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.fakes.radio_io import PhysicalPort, Wait
from cura_receiver.ports.radio import Dio1Edge
from cura_receiver.radio import Radio
from cura_receiver.sx1262 import Sx1262


def _begin(queue):
    ingress = ProtocolIngress(
        queue=queue,
        monotonic_clock=FakeOsClock(monotonic_us=20, realtime_us=0),
        auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY},
    )
    occurrence = ingress.begin(
        ingress_packet(receiver_instance_id=INSTANCE, frame=REVIEWED_CURRENT_FRAME)
    )
    return ingress, occurrence


# D-005/D-006 preserve accepted bytes and real optional timestamps through SQLite.
@pytest.mark.parametrize(
    ("result", "stored_code"),
    ((AckTxResult.SET_TX_FAILED, 3), (AckTxResult.TX_UNCONFIRMED, 7)),
)
@pytest.mark.parametrize("t4", (None, 21), ids=("before_set_tx", "attempted_set_tx"))
@pytest.mark.parametrize(
    ("state", "t6"),
    (
        (RadioState.RX_SINGLE, 23),
        (RadioState.RX_EVENT_PENDING, 23),
        (RadioState.SHUTDOWN, None),
        (RadioState.RECOVERY_EXHAUSTED, None),
        (RadioState.HARDWARE_MISSING, None),
    ),
)
def test_radio_failure_completion_reaches_real_sqlite(
    setup, result, stored_code, t4, state, t6
):
    _, connection, queue, _, create = setup
    persistence = create()
    persistence.enable_admission()
    ingress, occurrence = _begin(queue)
    candidate = occurrence.candidate
    assert candidate is not None
    assert queue.snapshot().reserved_entities == 1
    assert queue.claim_batch(max_entities=1) is None

    finalized = ingress.finalize(
        occurrence,
        ProtocolIngressTerminalV1(result, t4, None, t6, radio_state=state),
    )

    assert finalized.published_entity.candidate is candidate
    assert finalized.published_entity.profile.ack_frame == REVIEWED_ACCEPTED_ACK
    assert persistence.attempt(max_entities=1).outcome is (
        OrdinaryBatchCommitOutcome.COMMITTED
    )
    assert connection.execute(
        "SELECT ack_tx_result_id, t4_set_tx_attempted_monotonic_us, "
        "t5_tx_done_monotonic_us, t6_set_rx_issued_monotonic_us, "
        "received_frame, ack_frame FROM message_profiles"
    ).fetchall() == [
        (
            stored_code,
            t4,
            None,
            t6,
            REVIEWED_CURRENT_FRAME + bytes(255 - len(REVIEWED_CURRENT_FRAME)),
            REVIEWED_ACCEPTED_ACK,
        )
    ]
    assert connection.execute("SELECT count(*) FROM reading_messages").fetchone() == (1,)
    assert queue.snapshot().published_entities == 0


# Failed/unconfirmed results cannot smuggle a TxDone edge into a completed profile.
@pytest.mark.parametrize("result", (AckTxResult.SET_TX_FAILED, AckTxResult.TX_UNCONFIRMED))
@pytest.mark.parametrize("t4", (None, 21))
def test_radio_failure_rejects_txdone_without_consuming_reservation(setup, result, t4):
    _, _, queue, _, create = setup
    create().enable_admission()
    ingress, occurrence = _begin(queue)
    terminal = ProtocolIngressTerminalV1(
        result, t4, 22, 23, radio_state=RadioState.RX_SINGLE
    )
    before = queue.snapshot()

    with pytest.raises(ProtocolIngressInterfaceError, match="forbids T5"):
        ingress.finalize(occurrence, terminal)

    assert queue.snapshot() == before
    assert ingress.finalize(
        occurrence, replace(terminal, t5_tx_done_monotonic_us=None)
    ).published_entity is not None


# Adding optional pre-command T4 does not weaken timeout/exception attempt evidence.
@pytest.mark.parametrize("result", (AckTxResult.TX_TIMEOUT, AckTxResult.UNKNOWN_INTERRUPTED))
def test_confirmed_timeout_and_interruption_still_require_settx_attempt(setup, result):
    _, _, queue, _, create = setup
    create().enable_admission()
    ingress, occurrence = _begin(queue)
    terminal = ProtocolIngressTerminalV1(
        result, None, None, None, radio_state=RadioState.SHUTDOWN
    )
    before = queue.snapshot()

    with pytest.raises(ProtocolIngressInterfaceError, match="requires T4"):
        ingress.finalize(occurrence, terminal)

    assert queue.snapshot() == before
    assert ingress.finalize(
        occurrence, replace(terminal, t4_set_tx_attempted_monotonic_us=21)
    ).published_entity is not None


# Real authentication, profiling, queueing and SQLite consume the pre-mutation copy.
def test_radio_snapshot_through_real_ingress_and_sqlite(setup):
    _, connection, queue, _, create = setup
    persistence = create()
    persistence.enable_admission()
    clock = FakeOsClock(monotonic_us=1000)
    io = PhysicalPort(clock)
    radio = Radio(Sx1262(io, clock, Wait(clock)))
    radio.initialize()
    io.buffer[:] = REVIEWED_CURRENT_FRAME
    io.irq = 2
    io.status = 0x24
    io.edges.append(Dio1Edge(clock.now_monotonic_us() * 1000, 1))

    def mutate(command):
        if command[0] == 0x1E:
            io.buffer[:] = bytes(len(REVIEWED_CURRENT_FRAME))

    io.after_transfer = mutate
    snapshot = radio.receive(deadline_monotonic_us=10000).packet
    ingress = ProtocolIngress(queue=queue, monotonic_clock=clock,
                              auth_node_keys={REVIEWED_NODE_ID: REVIEWED_NODE_KEY})
    occurrence = ingress.begin(ingress_packet(
        receiver_instance_id=INSTANCE, frame=snapshot.frame,
        received_at_monotonic_us=snapshot.received_at_monotonic_us,
        t1_handler_started_monotonic_us=snapshot.t1_handler_started_monotonic_us,
        t2_packet_copied_monotonic_us=snapshot.t2_packet_copied_monotonic_us,
        rssi_dbm_x2=snapshot.rssi_dbm_x2, snr_db_x4=snapshot.snr_db_x4,
    ))
    assert occurrence.candidate is not None
    restored = radio.rearm()
    finalized = ingress.finalize(occurrence, ProtocolIngressTerminalV1(
        AckTxResult.SUPPRESSED_AIRTIME_BUDGET, None, None,
        restored.t6_set_rx_issued_monotonic_us, radio_state=restored.state,
    ))
    assert finalized.published_entity.profile.received_frame[:len(REVIEWED_CURRENT_FRAME)] == REVIEWED_CURRENT_FRAME
    assert persistence.attempt(max_entities=1).outcome is OrdinaryBatchCommitOutcome.COMMITTED
    assert connection.execute("SELECT received_frame FROM message_profiles").fetchone()[0] == REVIEWED_CURRENT_FRAME + bytes(255 - len(REVIEWED_CURRENT_FRAME))
