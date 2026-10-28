"""First transport payloads and immutable canonical samples, classified in FIFO order."""

from __future__ import annotations

from dataclasses import dataclass, fields

from .generated import protocol_v2_lora_generated as protocol
from .generated import receiver_entities_generated as rows
from .generated.receiver_enums_generated import PersistenceClassification as Classification, ProcessingResult
from .packet_reception_persistence import reception_exception, validate_reception
from .persist_queue_entities import AuthenticatedReadingCandidateV1, ReadingReceptionUnitV2
from .sqlite_repository import SqliteRepository, SqliteRow


class PersistenceIdentityCollision(Exception):
    """A durable identity or atomic relation contradicts intended immutable work."""


@dataclass(frozen=True, slots=True)
class PreparedReading:
    profile: rows.PacketReceptionRowV1
    insertion: rows.ReadingMessageRowV2 | None
    transport_insertion: rows.ReceivedMessageRowV1 | None
    prior_message: SqliteRow | None
    prior_transport: SqliteRow | None
    canonical_sample: SqliteRow | None
    exception: rows.PacketReceptionExceptionRowV1 | None
    replayed: bool = False


def exact_sql_row(actual, expected):
    """SQLite booleans are INTEGER; other storage classes must match exactly."""
    if actual is None or len(actual) != len(expected):
        return False
    return all(type(stored) is type(int(value) if type(value) is bool else value)
               and stored == value for stored, value in zip(actual, expected, strict=True))


def reading_row(candidate, *, canonical):
    decoded = protocol.decode_reading(candidate.reading_body)
    if type(candidate.sample_id) is not int or decoded.sample_id != candidate.sample_id:
        raise ValueError("candidate sample ID differs from decoded reading")
    return rows.ReadingMessageRowV2(
        node_id=candidate.node_id, message_id=candidate.message_id,
        is_canonical=canonical,
        **{field.name: getattr(decoded, field.name) for field in fields(decoded)},
    )


def validate_candidate(unit):
    if type(unit) is not ReadingReceptionUnitV2 or type(unit.candidate) is not AuthenticatedReadingCandidateV1:
        raise TypeError("invalid reading/reception composition")
    candidate, profile = unit.candidate, unit.profile
    validate_reception(profile, unit.received_frame)
    if (profile.processing_result is not ProcessingResult.ACCEPTED
        or type(candidate.domain) is not int or candidate.domain not in (1, 2)
        or type(candidate.message_id) is not int
        or type(candidate.node_id) is not bytes
        or type(candidate.reading_body) is not bytes
        or len(unit.received_frame) != protocol.READING_FRAME_SIZE
        or unit.received_frame[0] != protocol.CONTROL
        or (candidate.node_id, candidate.message_id, candidate.domain, candidate.sample_id)
           != (profile.node_id, profile.message_id, profile.domain, profile.decoded_sample_id)):
        raise ValueError("invalid authenticated reading candidate/reception")
    reading_row(candidate, canonical=True)


def decoded_payload(stored):
    """Complete payload columns, excluding transport ID and canonical selection."""
    return (stored[2], *stored[4:])


def load_message(repository: SqliteRepository, node_id: bytes, message_id: int):
    """Separate lookups preserve a transport whose required subtype is missing."""
    transport = repository.find_received_message(node_id, message_id)
    reading = repository.find_reading_message(node_id, message_id)
    if transport is None and reading is None:
        return None, None
    try:
        if transport is None or reading is None:
            raise ValueError("transport/subtype is missing")
        if (len(transport) != 3 or transport[:2] != (node_id, message_id)
            or type(transport[0]) is not bytes or len(transport[0]) != 8
            or type(transport[1]) is not int or not 0 <= transport[1] <= 0xFFFFFFFF
            or type(transport[2]) is not int or transport[2] not in (1, 2)
            or len(reading) != len(rows.READING_MESSAGE_ROW_V2_COLUMNS)
            or reading[:2] != transport[:2] or type(reading[0]) is not bytes
            or any(type(value) is not int for value in reading[1:])
            or reading[3] not in (0, 1)):
            raise ValueError("invalid transport/subtype")
        payload = protocol.Reading(**dict(zip(
            (field.name for field in fields(protocol.Reading)), decoded_payload(reading), strict=True)))
        # Reuse the protocol's structural/range validator, never a historical blob.
        protocol.encode_reading(payload)
    except (ValueError, TypeError, AttributeError, IndexError) as error:
        raise PersistenceIdentityCollision("invalid immutable transport/reading authority") from error
    return transport, reading


def canonical_sample(repository, node_id, sample_id):
    canonical = repository.find_canonical_sample(node_id, sample_id)
    if canonical is None:
        return None
    _, checked = load_message(repository, canonical[0], canonical[1])
    if checked != canonical or canonical[2:4] != (sample_id, 1):
        raise PersistenceIdentityCollision("invalid canonical sample authority")
    return canonical


def _existing_classification(transport, previous, canonical, candidate, prospective):
    if canonical is None or (previous[3] == 1 and previous != canonical):
        raise PersistenceIdentityCollision("reading lacks its canonical sample")
    if transport[2] != candidate.domain or previous[2] != candidate.sample_id:
        return Classification.MESSAGE_ID_CONFLICT
    if decoded_payload(previous) != decoded_payload(prospective):
        return Classification.DUPLICATE_CONFLICT
    if decoded_payload(previous) != decoded_payload(canonical):
        return Classification.DUPLICATE_CONFLICT
    return Classification.RETRANSMISSION


def prepare_reading(repository: SqliteRepository, unit: ReadingReceptionUnitV2) -> PreparedReading:
    validate_candidate(unit)
    candidate = unit.candidate
    prospective = rows.reading_message_row_v2_parameters(reading_row(candidate, canonical=True))
    transport, previous = load_message(repository, candidate.node_id, candidate.message_id)
    canonical = canonical_sample(repository, candidate.node_id,
                                 candidate.sample_id if previous is None else previous[2])
    insertion = new_transport = None
    if previous is not None:
        classification = _existing_classification(transport, previous, canonical, candidate, prospective)
    else:
        classification = (Classification.FIRST_SEEN if canonical is None
                          else Classification.DUPLICATE_SAME_CONTENT
                          if decoded_payload(canonical) == decoded_payload(prospective)
                          else Classification.DUPLICATE_CONFLICT)
        insertion = reading_row(candidate, canonical=canonical is None)
        new_transport = rows.ReceivedMessageRowV1(candidate.node_id, candidate.message_id, candidate.domain)
    profile = rows.PacketReceptionRowV1(unit.profile, classification)
    return PreparedReading(profile, insertion, new_transport, previous, transport, canonical,
                           reception_exception(unit.profile, unit.received_frame, classification))


def prepare_replayed_reading(repository: SqliteRepository, unit: ReadingReceptionUnitV2,
                             stored_profile: SqliteRow) -> PreparedReading:
    """A durable reception requires all its effects; replay never repairs partial work."""
    validate_candidate(unit)
    try:
        classification = Classification(stored_profile[-1])
    except (ValueError, TypeError, IndexError) as error:
        raise PersistenceIdentityCollision("unknown stored classification") from error
    profile = rows.PacketReceptionRowV1(unit.profile, classification)
    if not exact_sql_row(stored_profile, rows.packet_reception_row_v1_parameters(profile)):
        raise PersistenceIdentityCollision("reception identity collision")
    candidate = unit.candidate
    transport, previous = load_message(repository, candidate.node_id, candidate.message_id)
    if previous is None:
        raise PersistenceIdentityCollision("reception lacks transport effects")
    canonical = canonical_sample(repository, candidate.node_id, previous[2])
    prospective = rows.reading_message_row_v2_parameters(reading_row(candidate, canonical=True))
    relation = _existing_classification(transport, previous, canonical, candidate, prospective)
    if classification is Classification.FIRST_SEEN:
        valid = relation is Classification.RETRANSMISSION and previous == canonical
    elif classification is Classification.DUPLICATE_SAME_CONTENT:
        valid = relation is Classification.RETRANSMISSION and previous[3] == 0
    else:
        valid = classification is relation
    if not valid:
        raise PersistenceIdentityCollision("stored classification contradicts authority")
    return PreparedReading(profile, None, None, previous, transport, canonical,
                           reception_exception(unit.profile, unit.received_frame, classification), True)


def write_prepared_reading(repository: SqliteRepository, work: PreparedReading,
                           unit: ReadingReceptionUnitV2) -> None:
    """Compare frozen dependencies and all current effects before any write.

    Receipt existence distinguishes a completed atomic operation from an uncommitted
    retry. Its absence cannot legitimize a partially present new transport/subtype.
    """
    profile = work.profile.profile
    owner = (profile.receiver_instance_id, profile.occurrence_sequence)
    saved = repository.find_packet_reception(*owner)
    saved_exception = repository.find_packet_reception_exception(*owner)
    expected_exception = (None if work.exception is None
                          else rows.packet_reception_exception_row_v1_parameters(work.exception))
    if saved is not None:
        if not exact_sql_row(saved, rows.packet_reception_row_v1_parameters(work.profile)):
            raise PersistenceIdentityCollision("reception identity collision")
        if (saved_exception is not None if expected_exception is None
                else not exact_sql_row(saved_exception, expected_exception)):
            raise PersistenceIdentityCollision("missing or contradictory reception exception")
    elif work.replayed or saved_exception is not None:
        raise PersistenceIdentityCollision("partial reception/exception effects")

    candidate = unit.candidate
    transport = repository.find_received_message(candidate.node_id, candidate.message_id)
    reading = repository.find_reading_message(candidate.node_id, candidate.message_id)
    if work.insertion is not None:
        if saved is None:
            if transport is not None or reading is not None:
                raise PersistenceIdentityCollision("partial new transport effects")
        elif (not exact_sql_row(transport, rows.received_message_row_v1_parameters(work.transport_insertion))
              or not exact_sql_row(reading, rows.reading_message_row_v2_parameters(work.insertion))):
            raise PersistenceIdentityCollision("missing or contradictory transport effects")
    elif not exact_sql_row(transport, work.prior_transport) or not exact_sql_row(reading, work.prior_message):
        raise PersistenceIdentityCollision("prior transport authority changed")
    sample_id = candidate.sample_id if work.insertion is not None else work.prior_message[2]
    canonical = repository.find_canonical_sample(candidate.node_id, sample_id)
    expected_canonical = (rows.reading_message_row_v2_parameters(work.insertion)
                          if saved is not None and work.insertion is not None and work.insertion.is_canonical
                          else work.canonical_sample)
    if expected_canonical is None:
        if canonical is not None:
            raise PersistenceIdentityCollision("canonical sample appeared before retry")
    elif not exact_sql_row(canonical, expected_canonical):
        raise PersistenceIdentityCollision("canonical authority changed")

    if saved is None:
        if work.transport_insertion is not None:
            repository.insert_received_message(work.transport_insertion)
            repository.insert_reading_message(work.insertion)
        repository.insert_packet_reception(work.profile)
        if work.exception is not None:
            repository.insert_packet_reception_exception(work.exception)
