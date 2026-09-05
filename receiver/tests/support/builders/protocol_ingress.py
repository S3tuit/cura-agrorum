"""Reviewed literal inputs and keyword-only builders for protocol ingress."""

from __future__ import annotations

from cura_receiver.generated import protocol_v2_lora_generated as protocol
from cura_receiver.protocol_ingress import ProtocolIngressPacketV1
from cura_receiver.protocol_v2_lora_crypto import seal_frame


REVIEWED_NODE_ID = bytes.fromhex("0102030405060708")
REVIEWED_NODE_KEY = bytes.fromhex("c0f9a1a0f386692e01028082be92330e")
REVIEWED_MESSAGE_ID = 0x11223344
REVIEWED_SAMPLE_ID = 0x55667788
REVIEWED_READING_BODY = bytes.fromhex(
    "887766553412e803d00785ffc801ebfca0860100381508033075c4090702ff03"
)
REVIEWED_CURRENT_FRAME = bytes.fromhex(
    "2001010203040506070844332211"
    "adae77b2af564eea61861744eef32731637d1d12ac8b186fe36388c2eded90d5"
    "41bb930f548308b9"
)
REVIEWED_BACKLOG_FRAME = bytes.fromhex(
    "2002010203040506070844332211"
    "0badc99335db74910f378aeac41892874b4259d599b55914dd43ecc75026b743"
    "e36bb78761644e77"
)
REVIEWED_ACCEPTED_ACK = bytes.fromhex(
    "2003010203040506070844332211e0af3cbe68558a346b"
)
REVIEWED_RETRY_LATER_ACK = bytes.fromhex(
    "2004010203040506070844332211fb771dc230d339d60f"
)
REVIEWED_REJECTED_UNSUPPORTED_ACK = bytes.fromhex(
    "2005010203040506070844332211389ddb4676129e89e3"
)
REVIEWED_REJECTED_MALFORMED_ACK = bytes.fromhex(
    "2006010203040506070844332211fc834d4cf1792538f2"
)


def authenticated_frame(
    *,
    node_key: bytes = REVIEWED_NODE_KEY,
    control: int = protocol.CONTROL,
    domain: int = protocol.Domain.CURRENT_READING_UPLINK,
    node_id: bytes = REVIEWED_NODE_ID,
    message_id: int = REVIEWED_MESSAGE_ID,
    body: bytes = REVIEWED_READING_BODY,
) -> bytes:
    """Construct one authenticated frame from explicit reviewed defaults."""

    return seal_frame(
        node_key,
        protocol.ClearHeader(
            control=control,
            domain=domain,
            node_id=node_id,
            message_id=message_id,
        ),
        body,
    )


def ingress_packet(
    *,
    frame: bytes = REVIEWED_CURRENT_FRAME,
    receiver_instance_id: bytes = b"r" * 16,
    occurrence_sequence: int = 1,
    received_at_monotonic_us: int = 10,
    rssi_dbm_x2: int | None = -140,
    snr_db_x4: int | None = 12,
    irq_status: int | None = 2,
    device_errors: int | None = 0,
    busy_wait_total_us: int = 0,
    busy_wait_max_us: int = 0,
    busy_wait_count: int = 0,
    busy_timeout_count: int = 0,
    last_busy_timeout_opcode: int | None = None,
    t1_handler_started_monotonic_us: int = 11,
    t2_packet_copied_monotonic_us: int = 12,
) -> ProtocolIngressPacketV1:
    """Build the stable copied-packet boundary without consulting mutable state."""

    return ProtocolIngressPacketV1(
        receiver_instance_id=receiver_instance_id,
        occurrence_sequence=occurrence_sequence,
        received_at_monotonic_us=received_at_monotonic_us,
        frame=frame,
        rssi_dbm_x2=rssi_dbm_x2,
        snr_db_x4=snr_db_x4,
        irq_status=irq_status,
        device_errors=device_errors,
        busy_wait_total_us=busy_wait_total_us,
        busy_wait_max_us=busy_wait_max_us,
        busy_wait_count=busy_wait_count,
        busy_timeout_count=busy_timeout_count,
        last_busy_timeout_opcode=last_busy_timeout_opcode,
        t1_handler_started_monotonic_us=t1_handler_started_monotonic_us,
        t2_packet_copied_monotonic_us=t2_packet_copied_monotonic_us,
    )


__all__ = [
    "REVIEWED_NODE_ID",
    "REVIEWED_NODE_KEY",
    "REVIEWED_MESSAGE_ID",
    "REVIEWED_SAMPLE_ID",
    "REVIEWED_READING_BODY",
    "REVIEWED_CURRENT_FRAME",
    "REVIEWED_BACKLOG_FRAME",
    "REVIEWED_ACCEPTED_ACK",
    "REVIEWED_RETRY_LATER_ACK",
    "REVIEWED_REJECTED_UNSUPPORTED_ACK",
    "REVIEWED_REJECTED_MALFORMED_ACK",
    "authenticated_frame",
    "ingress_packet",
]
