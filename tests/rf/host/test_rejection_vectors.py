"""RF-023 matrix checked against production ingress, not the vector builder itself."""
from dataclasses import asdict

from cryptography.hazmat.primitives.ciphers.aead import AESCCM
from cryptography.exceptions import InvalidTag
import pytest

from rejection_vectors import CASES, matrix, verify_profile
from cura_receiver.protocol_ingress import ProtocolIngress
from cura_receiver.producer_admission import ProducerAdmission
from cura_receiver.persist_queue import PersistQueue, PersistenceAdmissionSnapshot
from cura_receiver.generated.receiver_enums_generated import PersistenceAdmissionState
from tests.support.builders.protocol_ingress import ingress_packet
from tests.support.fakes.os_clock import FakeOsClock

ACTIVE = (b'active01', bytes(range(16)))
UNKNOWN = (b'unknown1', bytes(range(16, 32)))
REVOKED = (b'revoked1', bytes(range(32, 48)))


def vectors():
    return matrix(active=ACTIVE, unknown=UNKNOWN, revoked=REVOKED, first_message=100, first_sample=200)


def decide(vector):
    queue = PersistQueue(capacity_entities=4)
    queue.publish_admission_state(PersistenceAdmissionSnapshot(
        generation=1, state=PersistenceAdmissionState.AVAILABLE, changed_at_monotonic_us=1))
    keys = dict([ACTIVE] + ([REVOKED] if vector.phase == 'before_revocation' else []))
    ingress = ProtocolIngress(queue=ProducerAdmission(queue),
                              monotonic_clock=FakeOsClock(monotonic_us=20, realtime_us=0), auth_node_keys=keys)
    occurrence = ingress.begin(ingress_packet(frame=vector.frame))
    p = asdict(occurrence.pre_tx_profile)
    for name in ('processing_result', 'ack_selected'):
        p[name + '_id'] = p.pop(name).value
    return occurrence, p


@pytest.mark.parametrize('name', CASES)
def test_packet_matrix_matches_real_ordered_ingress(name):
    vector = next(v for v in vectors() if v.name == name)
    occurrence, profile = decide(vector)
    verify_profile(vector, profile)
    if vector.reading_body is None:
        assert occurrence.candidate is None
    else:
        assert occurrence.candidate.reading_body == vector.reading_body
        assert occurrence.candidate.sample_id == vector.sample_id
    if vector.name == 'implausible_reading':
        # The receiver preserves representable values without physical clamping.
        assert int.from_bytes(occurrence.candidate.reading_body[6:8], 'little') == 65000


def test_every_changed_authenticated_frame_uses_a_distinct_nonce():
    used = set()
    keys = dict([ACTIVE, UNKNOWN, REVOKED])
    for vector in vectors():
        for frame in (vector.frame, vector.ack):
            if frame is None or len(frame) < 22:
                continue
            nonce = frame[2:14] + frame[1:2]
            identity = (keys[frame[2:10]], nonce)
            assert identity not in used
            used.add(identity)
            if vector.name == 'bad_tag' and frame is vector.frame:
                with pytest.raises(InvalidTag): AESCCM(identity[0], tag_length=8).decrypt(nonce, frame[14:], frame[:14])
            else:
                AESCCM(identity[0], tag_length=8).decrypt(nonce, frame[14:], frame[:14])


@pytest.mark.parametrize('field,value', [
    ('header_authenticated', 0), ('decoded_sample_id', None),
    ('ack_frame', None), ('claimed_message_id', 999),
    ('processing_result_id', 1), ('received_frame', bytes(255)),
])
def test_conflicting_profile_does_not_pass(field, value):
    vector = vectors()[0]
    _, profile = decide(vector)
    profile[field] = value
    with pytest.raises(ValueError, match=field): verify_profile(vector, profile)


def test_silent_case_cannot_claim_an_ack_or_decoded_sample():
    vector = next(v for v in vectors() if v.name == 'bad_tag')
    _, profile = decide(vector)
    profile['ack_frame'] = vectors()[0].ack
    with pytest.raises(ValueError, match='ack_frame'): verify_profile(vector, profile)


@pytest.mark.parametrize('field,value', [('first_message', -1), ('first_message', 0xffffffff),
                                        ('first_sample', 0xffffffff), ('first_message', True)])
def test_counter_reservation_rejected_before_construction(field, value):
    args = dict(active=ACTIVE, unknown=UNKNOWN, revoked=REVOKED, first_message=1, first_sample=1)
    args[field] = value
    with pytest.raises(ValueError, match='counter range'): matrix(**args)


def test_identity_aliasing_rejected_and_revocation_phase_is_last():
    with pytest.raises(ValueError, match='distinct'):
        matrix(active=ACTIVE, unknown=ACTIVE, revoked=REVOKED, first_message=1, first_sample=1)
    value = vectors()
    assert all(v.phase == 'before_revocation' for v in value[:-1])
    assert value[-1].phase == 'after_revocation'
    assert value[-2].name == 'revocation_baseline' and value[-2].ack is not None
    assert value[-1].ack is None
