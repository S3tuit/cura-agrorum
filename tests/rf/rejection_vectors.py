"""RF-023 packet matrix and expected observations; no device or credential I/O.

Call only for a newly provisioned dedicated test identity set and reserved counter
range. This function is not a nonce allocator and must not reuse a production
node's key/counters. Device generation/transport remains a separate integration.
"""
from dataclasses import dataclass
import struct

from cryptography.hazmat.primitives.ciphers.aead import AESCCM
from cura_receiver.generated.receiver_enums_generated import ProcessingResult, AckSelection

CASES = ('implausible_reading', 'unsupported_control', 'unknown_domain',
         'malformed_length', 'malformed_flags', 'wrong_direction',
         'unsupported_control_wrong_direction', 'bad_tag', 'unknown_node',
         'short_header', 'revocation_baseline', 'revoked_node')
C6_CASES = ('RF-023.implausible', 'RF-023.control', 'RF-023.domain',
            'RF-023.body-length', 'RF-023.flags', 'RF-023.direction',
            'RF-023.control-direction', 'RF-023.bad-tag', 'RF-023.unknown',
            'RF-023.short-header', 'RF-023.before-revoke', 'RF-023.revoked')


@dataclass(frozen=True)
class Vector:
    name: str
    phase: str
    frame: bytes
    processing_result: str
    authenticated: bool
    sample_id: int | None
    reading_body: bytes | None
    ack: bytes | None


def encrypted(key, node, message, domain, body, control=0x20):
    header = struct.pack('<BB8sI', control, domain, node, message)
    nonce = struct.pack('<8sIB', node, message, domain)
    return header + AESCCM(key, tag_length=8).encrypt(nonce, body, header)


def matrix(*, active, unknown, revoked, first_message, first_sample):
    """Each identity is (8-byte node ID, 16-byte key); returns no key material."""
    identities = (active, unknown, revoked)
    if any(len(node) != 8 or len(key) != 16 for node, key in identities) or len({n for n, _ in identities}) != 3:
        raise ValueError('three distinct disposable node identities/keys required')
    for value in (first_message, first_sample):
        if type(value) is not int or not 0 <= value <= 0xffffffff - len(CASES) + 1:
            raise ValueError('reserved counter range would overflow')
    output = []
    for index, name in enumerate(CASES):
        node, key = unknown if name == 'unknown_node' else revoked if name.startswith('revoc') or name == 'revoked_node' else active
        message, sample = first_message + index, first_sample + index
        # Literal reviewed 32-byte layout. Values intentionally exceed physical
        # plausibility while remaining representable and correctly flagged.
        body = struct.pack('<IHHHhhhIHBBHHBBH', sample, 1000, 65000, 64000,
                           20000, -20000, 25000, 4000000000, 60000, 1, 0, 0, 0, 0, 0, 0xfe)
        control, domain, status = 0x20, 1, 0
        result, authenticated = 'ACCEPTED', True
        if name in ('unsupported_control', 'unsupported_control_wrong_direction'):
            control, status, result = 0x30, 2, 'REJECTED_UNSUPPORTED_CONTROL'
        if name == 'unknown_domain':
            domain, status, result = 0x7f, 2, 'REJECTED_UNSUPPORTED_DOMAIN'
        if name == 'malformed_length':
            body, status, result = body[:-1], 3, 'REJECTED_MALFORMED_LENGTH'
        if name == 'malformed_flags':
            body = body[:30] + struct.pack('<H', 0xfc)  # Nonzero soil0 with its validity bit cleared.
            status, result = 3, 'REJECTED_MALFORMED_BODY'
        if name in ('wrong_direction', 'unsupported_control_wrong_direction'):
            domain, body = 3, b'\0'
            if name == 'wrong_direction':
                status, result = None, 'WRONG_DIRECTION'
        if name in ('unknown_node', 'revoked_node'):
            status, result, authenticated = None, 'UNKNOWN_NODE', False
        if name == 'bad_tag':
            status, result, authenticated = None, 'AUTHENTICATION_FAILED', False
        frame = encrypted(key, node, message, domain, body, control)
        if name == 'bad_tag':
            frame = frame[:-1] + bytes([frame[-1] ^ 1])
        if name == 'short_header':
            # Raw short header, never a second authenticated frame for this nonce.
            frame, status, result, authenticated = frame[:13], None, 'REJECTED_MALFORMED_LENGTH', False
        ack = None if status is None else encrypted(key, node, message, status + 3, bytes([status]))
        output.append(Vector(name, 'after_revocation' if name == 'revoked_node' else 'before_revocation',
                             frame, result, authenticated, sample if result == 'ACCEPTED' else None,
                             body if result == 'ACCEPTED' else None, ack))
    return tuple(output)


def verify_profile(vector, profile):
    """Check a saved protocol profile; ACK selection is not physical TX/silence proof."""
    frame = vector.frame
    expected_claims = dict(claimed_control=frame[0] if len(frame) >= 1 else None,
                           claimed_domain=frame[1] if len(frame) >= 2 else None,
                           claimed_node_id=frame[2:10] if len(frame) >= 10 else None,
                           claimed_message_id=int.from_bytes(frame[10:14], 'little') if len(frame) >= 14 else None)
    ack_selected = AckSelection.NONE.value if vector.ack is None else vector.ack[1]
    expected = dict(received_frame_length=len(frame), received_frame=frame + bytes(255 - len(frame)),
                    processing_result_id=ProcessingResult[vector.processing_result].value,
                    ack_selected_id=ack_selected, header_authenticated=int(vector.authenticated),
                    decoded_sample_id=vector.sample_id, ack_frame=vector.ack, **expected_claims)
    for name, value in expected.items():
        if name not in profile or profile[name] != value:
            raise ValueError(f'{vector.name}: mismatched {name}')
    return dict(case=vector.name, scope='profile/ACK selection only; physical RF evidence still required')
