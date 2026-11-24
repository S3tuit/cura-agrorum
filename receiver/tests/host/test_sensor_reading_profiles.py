"""One ingress and fresh SQLite authority handle interleaved sensor families."""
import sqlite3
import struct

import pytest

from cura_receiver.generated import protocol_v2_lora_generated as P
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.producer_admission import ProducerAdmission
from cura_receiver.protocol_ingress import ProtocolIngress, ProtocolIngressTerminalV1
from cura_receiver.reading_persistence import load_message
from cura_receiver.sqlite_repository import SqliteRepository
from tests.support.builders.persistence import INSTANCE
from tests.support.builders.protocol_ingress import authenticated_frame, ingress_packet, REVIEWED_NODE_KEY


def body(selection, *, partial=False, moisture0=0, moisture1=65535):
    # Independent field ordering and bit assignments, including physically
    # impossible mask 15. No firmware electrical policy belongs in admission.
    flags = 0xe0
    values = [0, 0, 0, 0]
    moisture = [0, 0]
    for slot in (0, 1):
        if partial and slot == 0:
            continue
        if selection & (1 << slot):
            flags |= 2 << slot
            values[slot] = 1000 + slot
        if selection & (4 << slot):
            flags |= 1024 << slot
            moisture[slot] = (moisture0, moisture1)[slot]
        if selection & (5 << slot):
            flags |= 8 << slot
            values[2 + slot] = (-970, 32760)[slot]
    return struct.pack('<IHHHhhhIHBBHHBBHHHBB', 7, 500, *values,
                       2200, 100000, 5000, 1, 0, 0, 0, 0, 0, flags,
                       *moisture, selection, 0)


def receiver(setup):
    _, connection, queue, clock, create = setup
    persistence = create()
    persistence.enable_admission()
    ingress = ProtocolIngress(queue=ProducerAdmission(queue), monotonic_clock=clock,
        auth_node_keys={node: REVIEWED_NODE_KEY for node in (b'a'*8, b'b'*8, b'c'*8)})
    return ingress, persistence, connection


def deliver(ingress, persistence, raw, *, sequence, node=b'b'*8, message=1, domain=1,
            expected=E.ProcessingResult.ACCEPTED):
    frame = authenticated_frame(node_id=node, message_id=message, domain=domain, body=raw)
    occurrence = ingress.begin(ingress_packet(frame=frame, receiver_instance_id=INSTANCE,
                                             occurrence_sequence=sequence))
    assert occurrence.pre_tx_profile.processing_result is expected
    ingress.finalize(occurrence, ProtocolIngressTerminalV1(
        ack_tx_result=E.AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
        t4_set_tx_attempted_monotonic_us=None, t5_tx_done_monotonic_us=None,
        t6_set_rx_issued_monotonic_us=101, radio_state=E.RadioState.RX_SINGLE))
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1


def test_interleaved_families_keep_values_selection_and_duplicates_after_reopen(setup):
    ingress, persistence, connection = receiver(setup)
    analog, single, dual = body(3), body(4), body(12, partial=True)
    traffic = [(b'a'*8, 1, 1, analog), (b'b'*8, 1, 1, single),
               (b'c'*8, 1, 1, dual), (b'a'*8, 1, 1, analog),
               (b'b'*8, 2, 2, single), (b'c'*8, 2, 2, dual)]
    for sequence, (node, message, domain, raw) in enumerate(traffic, 1):
        deliver(ingress, persistence, raw, sequence=sequence, node=node,
                message=message, domain=domain)
    assert connection.execute('SELECT persistence_classification_id FROM packet_receptions ORDER BY occurrence_sequence').fetchall() == [(1,), (1,), (1,), (2,), (3,), (3,)]
    # A second connection reads committed storage and uses production semantic
    # validation of every immutable column, not a cached candidate object.
    with sqlite3.connect(setup[0]) as reopened:
        repository = SqliteRepository(reopened)
        for node, message, _, raw in traffic:
            _, saved = load_message(repository, node, message)
            assert saved[2] == 7
            assert (saved[2], *saved[4:]) == struct.unpack('<IHHHhhhIHBBHHBBHHHBB', raw)


@pytest.mark.parametrize('selection', [0, 1, 3, 4, 12, 15])
def test_receiver_accepts_structural_masks_without_electrical_restrictions(setup, selection):
    ingress, persistence, connection = receiver(setup)
    deliver(ingress, persistence, body(selection), sequence=1)
    assert connection.execute('SELECT sensor_selection, reserved FROM reading_messages').fetchone() == (selection, 0)


@pytest.mark.parametrize('changed', [body(12), body(4, moisture0=1), body(4, partial=True)])
def test_new_fields_and_validity_participate_in_immutable_sample_conflicts(setup, changed):
    ingress, persistence, connection = receiver(setup)
    deliver(ingress, persistence, body(4), sequence=1)
    deliver(ingress, persistence, changed, sequence=2, message=2, domain=2)
    deliver(ingress, persistence, changed, sequence=3)
    assert connection.execute('SELECT persistence_classification_id FROM packet_receptions ORDER BY occurrence_sequence').fetchall() == [(1,), (4,), (4,)]
    assert connection.execute('SELECT sensor_selection,soil_0_moisture_deci_pct FROM reading_messages WHERE is_canonical=1').fetchone() == (4, 0)


@pytest.mark.parametrize('offset,value', [(36, 16), (37, 1), (36, 0), (32, 1), (31, 0x14)])
def test_receiver_rejects_malformed_new_fields_without_reading_authority(setup, offset, value):
    ingress, persistence, connection = receiver(setup)
    raw = bytearray(body(4, partial=(offset == 32)))
    raw[offset] = value
    deliver(ingress, persistence, bytes(raw), sequence=1,
            expected=E.ProcessingResult.REJECTED_MALFORMED_BODY)
    assert connection.execute('SELECT count(*) FROM reading_messages').fetchone() == (0,)
