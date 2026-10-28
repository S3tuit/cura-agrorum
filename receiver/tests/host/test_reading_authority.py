"""Transport/sample independence and exact atomic reception effects, with real SQLite."""

from dataclasses import replace

import pytest

from cura_receiver.generated import receiver_entities_generated as rows
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.generated import protocol_v2_lora_generated as protocol
from cura_receiver.persist_queue_entities import READING_RECEPTION_V2_SPEC, RECEPTION_ONLY_V2_SPEC, ReceptionOnlyUnitV2
from cura_receiver.reading_persistence import PersistenceIdentityCollision, load_message, reading_row
from cura_receiver.sqlite_repository import SqliteRepository
from cura_receiver.sqlite_transactions import SqliteTransactions
from tests.support.builders.persistence import _measurement, _profile
from tests.support.builders.protocol_ingress import authenticated_frame


def publish(queue, unit):
    spec = READING_RECEPTION_V2_SPEC if hasattr(unit, "candidate") else RECEPTION_ONLY_V2_SPEC
    queue.try_reserve_one(spec).reservation.publish(unit)


def run_units(setup, units, transactions=None):
    _, connection, queue, clock, create = setup
    persistence = create(transactions)
    persistence.enable_admission()
    for unit in units:
        publish(queue, unit)
    result = persistence.attempt(max_entities=len(units))
    return persistence, result, connection, queue, clock


def test_transport_subtype_lookup_does_not_hide_missing_related_row(setup):
    _, connection, *_ = setup
    repository = SqliteRepository(connection)
    assert load_message(repository, b"n"*8, 1) == (None, None)
    connection.execute("BEGIN")
    repository.insert_received_message(rows.ReceivedMessageRowV1(b"n"*8, 1, 1))
    assert repository.find_received_message(b"n"*8, 1) == (b"n"*8, 1, 1)
    with pytest.raises(PersistenceIdentityCollision, match="authority"):
        load_message(repository, b"n"*8, 1)
    repository.insert_reading_message(reading_row(_measurement(message=1).candidate, canonical=True))
    assert load_message(repository, b"n"*8, 1)[1][2:4] == (200, 1)
    connection.execute("ROLLBACK")
    assert load_message(repository, b"n"*8, 1) == (None, None)


def test_conflicting_transport_retains_its_first_payload(setup):
    units = [
        _measurement(sequence=1, message=1, sample=1, soil=1000),
        _measurement(sequence=2, message=2, sample=1, soil=2000),
        _measurement(sequence=3, message=2, sample=2, soil=3000),
        _measurement(sequence=4, message=2, sample=1, soil=1000),
        _measurement(sequence=5, message=2, sample=1, soil=2000),
    ]
    _, result, connection, queue, _ = run_units(setup, units)
    assert result.acknowledged_entities == 5
    assert connection.execute("SELECT message_id,sample_id,soil_0_mv,is_canonical FROM reading_messages ORDER BY message_id").fetchall() == [(1,1,1000,1),(2,1,2000,0)]
    assert connection.execute("SELECT persistence_classification_id FROM packet_receptions ORDER BY occurrence_sequence").fetchall() == [(1,),(4,),(5,),(4,),(4,)]
    assert connection.execute("SELECT received_frame FROM packet_reception_exceptions ORDER BY occurrence_sequence").fetchall() == [(u.received_frame,) for u in units[1:]]
    assert queue.snapshot().published_entities == 0


@pytest.mark.parametrize("changes, expected", [
    ({}, [1, 2]),
    ({"message": 101, "domain": 2}, [1, 3]),
    ({"domain": 2}, [1, 5]),
    ({"sample": 201}, [1, 5]),
    ({"soil": 1001}, [1, 4]),
])
def test_identity_and_payload_comparisons(setup, changes, expected):
    _, result, connection, _, _ = run_units(setup, [_measurement(), _measurement(sequence=2, **changes)])
    assert result.acknowledged_entities == 2
    assert [r[0] for r in connection.execute("SELECT persistence_classification_id FROM packet_receptions ORDER BY occurrence_sequence")] == expected


def test_classification_does_not_require_historical_reception(setup):
    persistence, _, connection, queue, _ = run_units(setup, [_measurement()])
    connection.execute("DROP TRIGGER packet_receptions_no_delete")
    connection.execute("DELETE FROM packet_receptions")
    publish(queue, _measurement(sequence=2))
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert connection.execute("SELECT persistence_classification_id FROM packet_receptions").fetchall() == [(2,)]


def test_older_unseen_backlog_is_not_rejected_by_high_water(setup):
    _, result, connection, _, _ = run_units(setup, [
        _measurement(sequence=1, message=200, sample=200),
        _measurement(sequence=2, message=5, sample=5, domain=2),
        _measurement(sequence=3, message=5, sample=5, domain=2),
    ])
    assert result.acknowledged_entities == 3
    assert connection.execute("SELECT persistence_classification_id FROM packet_receptions ORDER BY occurrence_sequence").fetchall() == [(1,),(1,),(2,)]


@pytest.mark.parametrize("processing,domain,body,expected_reason", [
    (E.ProcessingResult.WRONG_DIRECTION, 3, b"\0", E.PacketExceptionReason.WRONG_DIRECTION),
    (E.ProcessingResult.REJECTED_MALFORMED_LENGTH, 1, b"\0", E.PacketExceptionReason.REJECTED_MALFORMED_LENGTH),
    (E.ProcessingResult.RETRY_LATER_QUEUE_FULL, 1, b"\0", None),
])
def test_nonadmitted_reception_does_not_claim_transport(setup, processing, domain, body, expected_reason):
    raw = authenticated_frame(node_id=b"n"*8, message_id=100, domain=domain, body=body)
    rejection = ReceptionOnlyUnitV2(replace(_profile(), domain=domain, processing_result=processing,
        ack_tx_result=E.AckTxResult.NOT_APPLICABLE if processing is E.ProcessingResult.WRONG_DIRECTION else E.AckTxResult.SUPPRESSED_AIRTIME_BUDGET), raw)
    _, result, connection, _, _ = run_units(setup, [rejection, _measurement(sequence=2)])
    assert result.acknowledged_entities == 2
    assert connection.execute("SELECT domain FROM received_messages").fetchall() == [(1,)]
    assert connection.execute("SELECT persistence_classification_id FROM packet_receptions ORDER BY occurrence_sequence").fetchall() == [(0,),(1,)]
    assert connection.execute("SELECT reason_id,received_frame FROM packet_reception_exceptions").fetchall() == ([] if expected_reason is None else [(expected_reason.value,raw)])


class UnknownCommit(SqliteTransactions):
    def __init__(self, applied):
        self.applied = applied
        self.first = True

    def commit(self, connection):
        if self.first:
            self.first = False
            if self.applied:
                super().commit(connection)
            raise OSError("lost COMMIT outcome")
        super().commit(connection)


@pytest.mark.parametrize("applied", [False, True])
def test_unknown_commit_preserves_original_classification_and_all_effects(setup, applied):
    units = [_measurement(), _measurement(sequence=2, message=101, soil=2000)]
    persistence, result, connection, queue, clock = run_units(setup, units, UnknownCommit(applied))
    assert result.outcome.name == "OUTCOME_UNKNOWN"
    assert queue.snapshot().claimed_entities == 2
    clock.advance_elapsed_us(10_000_000)
    retry = persistence.attempt(max_entities=2)
    assert retry.acknowledged_entities == 2
    assert connection.execute("SELECT persistence_classification_id FROM packet_receptions ORDER BY occurrence_sequence").fetchall() == [(1,),(4,)]
    assert connection.execute("SELECT count(*) FROM received_messages").fetchone() == (2,)
    assert connection.execute("SELECT received_frame FROM packet_reception_exceptions").fetchall() == [(units[1].received_frame,)]


def test_replayed_queue_unit_preserves_first_seen_classification(setup):
    unit = _measurement()
    persistence, _, connection, queue, _ = run_units(setup, [unit])
    publish(queue, unit)
    assert persistence.attempt(max_entities=1).acknowledged_entities == 1
    assert connection.execute("SELECT persistence_classification_id FROM packet_receptions").fetchall() == [(1,)]


def test_missing_exception_after_unknown_commit_is_not_repaired(setup):
    units = [_measurement(), _measurement(sequence=2, message=101, soil=2000)]
    persistence, _, connection, queue, clock = run_units(setup, units, UnknownCommit(True))
    connection.execute("DROP TRIGGER packet_reception_exceptions_no_delete")
    connection.execute("DELETE FROM packet_reception_exceptions")
    clock.advance_elapsed_us(10_000_000)
    result = persistence.attempt(max_entities=2)
    assert result.acknowledged_entities == 0
    assert queue.snapshot().published_entities == 2
    assert connection.execute("SELECT count(*) FROM packet_reception_exceptions").fetchone() == (0,)


@pytest.mark.parametrize("field", [
    "run_ms", "soil_0_mv", "soil_1_mv", "soil_temp_0_centi_c", "soil_temp_1_centi_c",
    "enclosure_centi_c", "enclosure_pressure_pa", "enclosure_humidity_centi_pct",
    "reset_reason", "previous_current_tx_attempts", "previous_awake_ms",
    "previous_current_delivery_ms", "previous_cycle_tx_attempts",
    "previous_cycle_accepted_readings", "flags",
])
def test_every_decoded_payload_field_participates_in_equality(setup, field):
    first = _measurement(soil=0 if field == "flags" else 1000)
    decoded = protocol.decode_reading(first.candidate.reading_body)
    if field == "reset_reason":
        decoded = replace(decoded, reset_reason=9, flags=decoded.flags & ~1)
        body = protocol.encode_reading(decoded)
        first = replace(first, candidate=replace(first.candidate, reading_body=body),
                        received_frame=authenticated_frame(node_id=b"n"*8, message_id=100, body=body))
    value = getattr(decoded, field)
    changed = replace(decoded, **{field: value ^ 2 if field == "flags" else value + 1})
    body = protocol.encode_reading(changed)
    second = replace(_measurement(sequence=2),
                     candidate=replace(first.candidate, reading_body=body),
                     received_frame=authenticated_frame(node_id=b"n"*8, message_id=100, body=body))
    _, result, connection, _, _ = run_units(setup, [first, second])
    assert result.acknowledged_entities == 2
    assert connection.execute("SELECT persistence_classification_id FROM packet_receptions ORDER BY occurrence_sequence").fetchall() == [(1,), (4,)]


@pytest.mark.parametrize("processing,control,domain,body", [
    (E.ProcessingResult.REJECTED_UNSUPPORTED_CONTROL, 0x30, 1, b"\0"),
    (E.ProcessingResult.REJECTED_UNSUPPORTED_DOMAIN, 0x20, 127, b"\0"),
    (E.ProcessingResult.REJECTED_MALFORMED_BODY, 0x20, 1, bytes(32)),
])
def test_other_authenticated_rejections_preserve_exact_raw_evidence(setup, processing, control, domain, body):
    raw = authenticated_frame(node_id=b"n"*8, message_id=100, control=control, domain=domain, body=body)
    unit = ReceptionOnlyUnitV2(replace(_profile(), domain=domain, processing_result=processing,
        ack_tx_result=E.AckTxResult.SUPPRESSED_AIRTIME_BUDGET), raw)
    _, result, connection, _, _ = run_units(setup, [unit])
    assert result.acknowledged_entities == 1
    assert connection.execute("SELECT received_frame_length,received_frame FROM packet_reception_exceptions").fetchall() == [(len(raw),raw)]
    assert connection.execute("SELECT count(*) FROM received_messages").fetchone() == (0,)


def test_failure_during_exception_insert_rolls_back_whole_batch(setup):
    _, connection, queue, _, create = setup
    persistence = create()
    persistence.enable_admission()
    connection.execute("CREATE TEMP TRIGGER reject_exception BEFORE INSERT ON packet_reception_exceptions BEGIN SELECT RAISE(ABORT, 'test'); END")
    for unit in [_measurement(), _measurement(sequence=2, message=101, soil=2000)]:
        publish(queue, unit)
    assert persistence.attempt(max_entities=2).acknowledged_entities == 0
    for table in ("received_messages","reading_messages","packet_receptions","packet_reception_exceptions"):
        assert connection.execute(f"SELECT count(*) FROM {table}").fetchone() == (0,)
    assert queue.snapshot().published_entities == 2


@pytest.mark.parametrize("table", ["reading_messages", "received_messages", "packet_receptions"])
def test_partial_current_effect_after_unknown_commit_is_not_repaired(setup, table):
    persistence, _, connection, queue, clock = run_units(setup, [_measurement()], UnknownCommit(True))
    connection.execute("PRAGMA foreign_keys = OFF")
    connection.execute(f"DROP TRIGGER {table}_no_delete")
    connection.execute(f"DELETE FROM {table}")
    connection.execute("PRAGMA foreign_keys = ON")
    clock.advance_elapsed_us(10_000_000)
    result = persistence.attempt(max_entities=1)
    assert result.acknowledged_entities == 0
    assert queue.snapshot().published_entities == 1
    assert connection.execute(f"SELECT count(*) FROM {table}").fetchone() == (0,)


def test_queued_receptions_choose_ack_without_history_then_classify_in_order(setup):
    from cura_receiver.producer_admission import ProducerAdmission
    from cura_receiver.protocol_ingress import ProtocolIngress, ProtocolIngressTerminalV1
    from tests.support.builders.persistence import INSTANCE
    from tests.support.builders.protocol_ingress import REVIEWED_NODE_KEY, ingress_packet
    from tests.support.fakes.os_clock import FakeOsClock

    _, connection, queue, _, create = setup
    persistence = create()
    persistence.enable_admission()
    ingress = ProtocolIngress(queue=ProducerAdmission(queue),
        monotonic_clock=FakeOsClock(monotonic_us=20),
        auth_node_keys={b"n"*8: REVIEWED_NODE_KEY})
    units = [
        _measurement(sequence=1, message=1, sample=1),
        _measurement(sequence=2, message=1, sample=2),
        _measurement(sequence=3, message=1, sample=1),
    ]
    statements = []
    connection.set_trace_callback(statements.append)
    for sequence, unit in enumerate(units, 1):
        occurrence = ingress.begin(ingress_packet(receiver_instance_id=INSTANCE,
            occurrence_sequence=sequence, frame=unit.received_frame))
        assert occurrence.pre_tx_profile.ack_selected is E.AckSelection.ACCEPTED
        ingress.finalize(occurrence, ProtocolIngressTerminalV1(
            E.AckTxResult.SUPPRESSED_AIRTIME_BUDGET, None, None, 23,
            radio_state=E.RadioState.RX_SINGLE))
    assert statements == []
    assert queue.snapshot().published_entities == 3
    assert persistence.attempt(max_entities=3).acknowledged_entities == 3
    assert connection.execute("SELECT persistence_classification_id FROM packet_receptions ORDER BY occurrence_sequence").fetchall() == [(1,), (5,), (2,)]
    assert connection.execute("SELECT message_id,sample_id FROM reading_messages").fetchall() == [(1, 1)]


def test_bound_backlog_retried_after_newer_current(setup):
    _, result, connection, _, _ = run_units(setup, [
        _measurement(sequence=1, message=5, sample=5, domain=2),
        _measurement(sequence=2, message=200, sample=200),
        _measurement(sequence=3, message=5, sample=5, domain=2),
    ])
    assert result.acknowledged_entities == 3
    assert connection.execute("SELECT persistence_classification_id FROM packet_receptions ORDER BY occurrence_sequence").fetchall() == [(1,), (1,), (2,)]
