"""Immutable reading classification and its SQLite evidence, in FIFO order."""

from __future__ import annotations

from dataclasses import dataclass, fields

from .generated import protocol_v2_lora_generated as protocol
from .generated import receiver_entities_generated as rows
from .generated.receiver_enums_generated import (
    PersistenceClassification as Classification,
)
from .generated.receiver_enums_generated import ProcessingResult
from .persist_queue_entities import (
    AuthenticatedReadingCandidateV1,
    MeasurementProfileUnitV1,
)
from .sqlite_repository import SqliteRepository, SqliteRow


class PersistenceIdentityCollision(Exception):
    """A durable identity or atomic relation contradicts intended immutable work."""


@dataclass(frozen=True, slots=True)
class PreparedReading:
    profile: rows.MessageProfileRowV1
    insertion: rows.ReadingMessageRowV1 | None
    prior_message: SqliteRow | None
    prior_first_profile: SqliteRow | None
    canonical_sample: SqliteRow | None
    canonical_first_profile: SqliteRow | None = None


def reading_row(
    candidate: AuthenticatedReadingCandidateV1,
    *,
    canonical: bool,
    owner: tuple[bytes, int],
) -> rows.ReadingMessageRowV1:
    decoded = protocol.decode_reading(candidate.reading_body)
    if type(candidate.sample_id) is not int or decoded.sample_id != candidate.sample_id:
        raise ValueError("candidate sample ID does not match its reading body")
    return rows.ReadingMessageRowV1(
        node_id=candidate.node_id,
        message_id=candidate.message_id,
        reading_body=candidate.reading_body,
        is_canonical_for_sample=canonical,
        first_receiver_instance_id=owner[0],
        first_occurrence_sequence=owner[1],
        **{field.name: getattr(decoded, field.name) for field in fields(decoded)},
    )


def validate_candidate(unit: MeasurementProfileUnitV1) -> None:
    candidate, profile = unit.candidate, unit.profile
    if (
        type(candidate) is not AuthenticatedReadingCandidateV1
        or type(profile) is not rows.MessageProfilingV1
    ):
        raise TypeError("invalid measurement/profile composition")
    if (
        type(candidate.node_id) is not bytes
        or len(candidate.node_id) != 8
        or type(candidate.message_id) is not int
        or not 0 <= candidate.message_id <= 0xFFFFFFFF
        or type(candidate.domain) is not int
        or candidate.domain not in (1, 2)
        or type(candidate.reading_body) is not bytes
        or len(candidate.reading_body) != 32
        or profile.header_authenticated is not True
        or profile.processing_result is not ProcessingResult.ACCEPTED
        or profile.received_frame_length != 54
        or type(profile.received_frame) is not bytes
        or len(profile.received_frame) != 255
        or profile.received_frame[54:] != bytes(201)
    ):
        raise ValueError("invalid authenticated reading candidate/profile")
    header = protocol.decode_clear_header(profile.received_frame[:14])
    if (
        header.control != protocol.CONTROL
        or (header.node_id, header.message_id, header.domain)
        != (candidate.node_id, candidate.message_id, candidate.domain)
        or (
            profile.claimed_node_id,
            profile.claimed_message_id,
            profile.claimed_domain,
            profile.claimed_control,
            profile.decoded_sample_id,
        )
        != (
            candidate.node_id,
            candidate.message_id,
            candidate.domain,
            protocol.CONTROL,
            candidate.sample_id,
        )
    ):
        raise ValueError("candidate and authenticated profiling identities disagree")


def prepare_reading(
    repository: SqliteRepository, unit: MeasurementProfileUnitV1
) -> PreparedReading:
    validate_candidate(unit)
    candidate, profile = unit.candidate, unit.profile
    owner = (profile.receiver_instance_id, profile.occurrence_sequence)
    prospective = reading_row(candidate, canonical=True, owner=owner)
    previous = repository.find_reading_message(candidate.node_id, candidate.message_id)
    first_profile = None
    canonical = None
    insertion = None
    if previous is not None:
        if previous[-2:] == owner:
            raise PersistenceIdentityCollision(
                "reading claims an occurrence whose profile is absent"
            )
        if not _earlier(repository, previous[-2:], owner):
            raise PersistenceIdentityCollision(
                "existing message is not owned by an earlier occurrence"
            )
        first_profile = _validate_stored_reading(repository, previous)
        if previous[4] == 0:
            canonical = repository.find_canonical_sample(previous[0], previous[2])
        if first_profile is None:
            raise PersistenceIdentityCollision("reading first profile is missing")
        if previous[2] != candidate.sample_id:
            classification = Classification.MESSAGE_ID_CONFLICT
        elif (first_profile[3], first_profile[4]) == (
            profile.received_frame_length,
            profile.received_frame,
        ):
            classification = Classification.RETRANSMISSION
        else:
            classification = Classification.DUPLICATE_CONFLICT
    else:
        canonical = repository.find_canonical_sample(
            candidate.node_id, candidate.sample_id
        )
        if canonical is None:
            classification = Classification.FIRST_SEEN
            insertion = prospective
        else:
            if not _earlier(repository, canonical[-2:], owner):
                raise PersistenceIdentityCollision(
                    "canonical sample owner is not earlier"
                )
            _validate_stored_reading(repository, canonical)
            classification = (
                Classification.DUPLICATE_SAME_CONTENT
                if canonical[3] == candidate.reading_body
                else Classification.DUPLICATE_CONFLICT
            )
            insertion = reading_row(candidate, canonical=False, owner=owner)
    canonical_profile = (
        None if canonical is None else _validate_stored_reading(repository, canonical)
    )
    return PreparedReading(
        rows.MessageProfileRowV1(profile, classification),
        insertion,
        previous,
        first_profile,
        canonical,
        canonical_profile,
    )


def exact_sql_row(actual: SqliteRow | None, expected: SqliteRow) -> bool:
    """SQLite booleans are INTEGER; every other storage class must match too."""
    if actual is None or len(actual) != len(expected):
        return False
    for stored, intended in zip(actual, expected, strict=True):
        if type(intended) is bool:
            intended = int(intended)
        if type(stored) is not type(intended) or stored != intended:
            return False
    return True


def _earlier(repository: SqliteRepository, earlier: tuple, current: tuple) -> bool:
    if earlier[0] == current[0]:
        return earlier[1] < current[1]
    previous_instance = repository.find_receiver_instance(earlier[0])
    current_instance = repository.find_receiver_instance(current[0])
    return (
        previous_instance is not None
        and current_instance is not None
        and previous_instance[0] < current_instance[0]
    )


def _validate_stored_reading(
    repository: SqliteRepository, stored: SqliteRow
) -> SqliteRow:
    """Check decoded columns and the owning profile, without trusting SQL shape."""
    try:
        if (
            len(stored) != len(rows.READING_MESSAGE_ROW_V1_COLUMNS)
            or type(stored[4]) is not int
            or stored[4] not in (0, 1)
        ):
            raise ValueError("invalid canonical Boolean")
        owner = repository.find_message_profile(stored[-2], stored[-1])
        if (
            owner is None
            or owner[3] != 54
            or type(owner[4]) is not bytes
            or len(owner[4]) != 255
            or owner[4][54:] != bytes(201)
        ):
            raise ValueError("invalid first profile frame")
        header = protocol.decode_clear_header(owner[4][:14])
        if (header.control, header.domain) not in ((0x20, 1), (0x20, 2)):
            raise ValueError("invalid authenticated first header")
        if (
            owner[5:11] != (0x20, header.domain, stored[0], stored[1], 1, stored[2])
            or (header.node_id, header.message_id) != stored[:2]
            or owner[15] != ProcessingResult.ACCEPTED.value
        ):
            raise ValueError("first profile does not own the stored reading")
        candidate = AuthenticatedReadingCandidateV1(
            stored[0], stored[1], header.domain, stored[2], stored[3]
        )
        expected = reading_row(candidate, canonical=bool(stored[4]), owner=stored[-2:])
        if not exact_sql_row(stored, rows.reading_message_row_v1_parameters(expected)):
            raise ValueError("stored decoded reading columns disagree")
        allowed = (1,) if stored[4] else (3, 4)
        if owner[-1] not in allowed:
            raise ValueError("first classification contradicts canonicality")
        if stored[4] == 0:
            canonical = repository.find_canonical_sample(stored[0], stored[2])
            if canonical is None or not _earlier(
                repository, canonical[-2:], stored[-2:]
            ):
                raise ValueError("noncanonical reading has no earlier canonical sample")
            _validate_stored_reading(repository, canonical)
            if (canonical[3] == stored[3]) != (
                owner[-1] == Classification.DUPLICATE_SAME_CONTENT.value
            ):
                raise ValueError(
                    "first duplicate classification contradicts its canonical body"
                )
        return owner
    except (TypeError, ValueError, AttributeError, IndexError) as error:
        raise PersistenceIdentityCollision(
            "invalid immutable reading evidence"
        ) from error


def prepare_replayed_reading(
    repository: SqliteRepository,
    unit: MeasurementProfileUnitV1,
    stored_profile: SqliteRow,
) -> PreparedReading:
    """Use the stored classification, including insertion of absent intended rows."""
    validate_candidate(unit)
    candidate, profile = unit.candidate, unit.profile
    try:
        classification = Classification(stored_profile[-1])
    except (ValueError, TypeError) as error:
        raise PersistenceIdentityCollision("unknown stored classification") from error
    profile_row = rows.MessageProfileRowV1(profile, classification)
    if not exact_sql_row(
        stored_profile, rows.message_profile_row_v1_parameters(profile_row)
    ):
        raise PersistenceIdentityCollision("occurrence profile collision")
    owner = (profile.receiver_instance_id, profile.occurrence_sequence)
    current = repository.find_reading_message(candidate.node_id, candidate.message_id)
    canonical = repository.find_canonical_sample(candidate.node_id, candidate.sample_id)
    previous = None
    previous_profile = None
    canonical_dependency = None
    insertion = None
    if classification is Classification.FIRST_SEEN:
        insertion = reading_row(candidate, canonical=True, owner=owner)
        if canonical is not None and not exact_sql_row(
            canonical, rows.reading_message_row_v1_parameters(insertion)
        ):
            raise PersistenceIdentityCollision("first-seen canonical collision")
    elif classification is Classification.DUPLICATE_SAME_CONTENT or (
        classification is Classification.DUPLICATE_CONFLICT
        and (current is None or current[-2:] == owner)
    ):
        if canonical is None or not _earlier(repository, canonical[-2:], owner):
            raise PersistenceIdentityCollision(
                "duplicate lacks an earlier canonical sample"
            )
        _validate_stored_reading(repository, canonical)
        same_body = canonical[3] == candidate.reading_body
        if same_body != (classification is Classification.DUPLICATE_SAME_CONTENT):
            raise PersistenceIdentityCollision("duplicate body relation disagrees")
        canonical_dependency = canonical
        insertion = reading_row(candidate, canonical=False, owner=owner)
    elif classification in (
        Classification.RETRANSMISSION,
        Classification.DUPLICATE_CONFLICT,
        Classification.MESSAGE_ID_CONFLICT,
    ):
        if current is None or not _earlier(repository, current[-2:], owner):
            raise PersistenceIdentityCollision(
                "classification lacks an earlier message"
            )
        previous = current
        previous_profile = _validate_stored_reading(repository, current)
        if current[4] == 0:
            canonical_dependency = repository.find_canonical_sample(
                current[0], current[2]
            )
        same_sample = current[2] == candidate.sample_id
        same_frame = (previous_profile[3], previous_profile[4]) == (
            profile.received_frame_length,
            profile.received_frame,
        )
        if classification is Classification.RETRANSMISSION:
            valid = same_sample and same_frame and current[3] == candidate.reading_body
        elif classification is Classification.MESSAGE_ID_CONFLICT:
            valid = not same_sample
        else:
            valid = same_sample and not same_frame
        if not valid:
            raise PersistenceIdentityCollision(
                "transport relation disagrees with classification"
            )
    else:
        raise PersistenceIdentityCollision(
            "accepted reading has no persistence classification"
        )
    if (
        insertion is not None
        and current is not None
        and not exact_sql_row(
            current, rows.reading_message_row_v1_parameters(insertion)
        )
    ):
        raise PersistenceIdentityCollision("reading effect collision")
    canonical_profile = (
        None
        if canonical_dependency is None
        else _validate_stored_reading(repository, canonical_dependency)
    )
    return PreparedReading(
        profile_row,
        insertion,
        previous,
        previous_profile,
        canonical_dependency,
        canonical_profile,
    )


def validate_prepared_reading(
    repository: SqliteRepository, work: PreparedReading, unit: MeasurementProfileUnitV1
) -> None:
    profile_parameters = tuple(
        int(value) if type(value) is bool else value
        for value in rows.message_profile_row_v1_parameters(work.profile)
    )
    checked = prepare_replayed_reading(repository, unit, profile_parameters)
    # Initial and replay preparation have the same immutable dependency values.
    if checked != work:
        raise PersistenceIdentityCollision(
            "frozen reading work or its dependencies changed"
        )
