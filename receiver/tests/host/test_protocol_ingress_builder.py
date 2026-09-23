from __future__ import annotations

from cura_receiver.generated import protocol_v2_lora_generated as protocol
from cura_receiver.protocol_v2_lora_crypto import open_frame
from tests.support.builders.protocol_ingress import (
    REVIEWED_BACKLOG_FRAME,
    REVIEWED_CURRENT_FRAME,
    REVIEWED_MESSAGE_ID,
    REVIEWED_NODE_ID,
    REVIEWED_NODE_KEY,
    REVIEWED_READING_BODY,
    REVIEWED_SAMPLE_ID,
    authenticated_frame,
    ingress_packet,
)


# Locks the literal current vector to the governing protocol's checked-in golden frame.
def test_reviewed_current_frame_is_the_protocol_golden_vector() -> None:
    assert REVIEWED_CURRENT_FRAME.hex() == (
        "2001010203040506070844332211"
        "adae77b2af564eea61861744eef32731637d1d12ac8b186fe36388c2eded90d5"
        "41bb930f548308b9"
    )
    authenticated = open_frame(REVIEWED_NODE_KEY, REVIEWED_CURRENT_FRAME)
    assert authenticated.header == protocol.ClearHeader(
        control=protocol.CONTROL,
        domain=protocol.Domain.CURRENT_READING_UPLINK,
        node_id=REVIEWED_NODE_ID,
        message_id=REVIEWED_MESSAGE_ID,
    )
    assert authenticated.plaintext_body == REVIEWED_READING_BODY
    assert protocol.decode_reading(authenticated.plaintext_body).sample_id == (
        REVIEWED_SAMPLE_ID
    )


# Locks a reviewed backlog counterpart with unchanged transport/application fields.
def test_reviewed_backlog_frame_has_only_the_protocol_defined_domain_change() -> None:
    assert REVIEWED_BACKLOG_FRAME.hex() == (
        "2002010203040506070844332211"
        "0badc99335db74910f378aeac41892874b4259d599b55914dd43ecc75026b743"
        "e36bb78761644e77"
    )
    authenticated = open_frame(REVIEWED_NODE_KEY, REVIEWED_BACKLOG_FRAME)
    assert authenticated.header == protocol.ClearHeader(
        control=protocol.CONTROL,
        domain=protocol.Domain.BACKLOG_READING_UPLINK,
        node_id=REVIEWED_NODE_ID,
        message_id=REVIEWED_MESSAGE_ID,
    )
    assert authenticated.plaintext_body == REVIEWED_READING_BODY
    assert authenticated_frame(
        domain=protocol.Domain.BACKLOG_READING_UPLINK
    ) == REVIEWED_BACKLOG_FRAME


# Applies literal, deterministic packet defaults and only explicit keyword overrides.
def test_packet_builder_has_reviewed_defaults_and_explicit_overrides() -> None:
    current = ingress_packet()
    backlog = ingress_packet(
        frame=REVIEWED_BACKLOG_FRAME,
        occurrence_sequence=2,
    )

    assert current.frame is REVIEWED_CURRENT_FRAME
    assert current.occurrence_sequence == 1
    assert backlog.frame is REVIEWED_BACKLOG_FRAME
    assert backlog.occurrence_sequence == 2
