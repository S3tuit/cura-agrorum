from __future__ import annotations

import copy
from dataclasses import FrozenInstanceError, fields, replace
from inspect import signature

import pytest

from cura_receiver.generated.receiver_enums_generated import AckTxResult
from cura_receiver.persist_queue import PersistQueue
from cura_receiver.protocol_ingress import (
    ProtocolIngress,
    ProtocolIngressConfigurationError,
    ProtocolIngressInterfaceError,
    ProtocolIngressPacketV1,
    ProtocolIngressTerminalV1,
)
from tests.support.builders.protocol_ingress import (
    REVIEWED_NODE_ID as NODE_ID,
    REVIEWED_NODE_KEY as NODE_KEY,
)
from tests.support.fakes.os_clock import FakeOsClock


def _packet(**changes: object) -> ProtocolIngressPacketV1:
    values: dict[str, object] = {
        "receiver_instance_id": b"r" * 16,
        "occurrence_sequence": 1,
        "received_at_monotonic_us": 10,
        "frame": bytes(54),
        "rssi_dbm_x2": -140,
        "snr_db_x4": 12,
        "irq_status": 2,
        "device_errors": 0,
        "busy_wait_total_us": 8,
        "busy_wait_max_us": 5,
        "busy_wait_count": 3,
        "busy_timeout_count": 1,
        "last_busy_timeout_opcode": 0x1D,
        "t1_handler_started_monotonic_us": 11,
        "t2_packet_copied_monotonic_us": 12,
    }
    values.update(changes)
    return ProtocolIngressPacketV1(**values)  # type: ignore[arg-type]


# Freezes the complete copied-frame boundary and preserves the exact bytes supplied.
def test_packet_snapshot_is_deeply_immutable_and_pi_owned() -> None:
    frame = bytes(range(54))
    packet = _packet(frame=frame)

    assert packet.frame is frame
    assert not hasattr(packet, "__dict__")
    assert [field.name for field in fields(packet)] == [
        "receiver_instance_id",
        "occurrence_sequence",
        "received_at_monotonic_us",
        "frame",
        "rssi_dbm_x2",
        "snr_db_x4",
        "irq_status",
        "device_errors",
        "busy_wait_total_us",
        "busy_wait_max_us",
        "busy_wait_count",
        "busy_timeout_count",
        "last_busy_timeout_opcode",
        "t1_handler_started_monotonic_us",
        "t2_packet_copied_monotonic_us",
    ]
    with pytest.raises(FrozenInstanceError):
        packet.frame = b"changed"  # type: ignore[misc]


# Rejects values that cannot form the stable pre-ingress profiling boundary.
@pytest.mark.parametrize(
    "changes",
    (
        {"receiver_instance_id": b"short"},
        {"occurrence_sequence": True},
        {"occurrence_sequence": -1},
        {"frame": bytearray(54)},
        {"frame": bytes(256)},
        {"rssi_dbm_x2": None},
        {"snr_db_x4": None},
        {"irq_status": 1 << 16},
        {"busy_wait_max_us": 9},
        {"busy_timeout_count": 4},
        {"busy_timeout_count": 0},
        {"last_busy_timeout_opcode": None},
        {"t1_handler_started_monotonic_us": 9},
        {"t2_packet_copied_monotonic_us": 10},
    ),
)
def test_packet_snapshot_rejects_invalid_stable_facts(
    changes: dict[str, object],
) -> None:
    with pytest.raises(ProtocolIngressInterfaceError):
        _packet(**changes)


# Keeps terminal radio facts typed and immutable without importing a radio adapter.
def test_terminal_facts_are_immutable_and_contain_no_radio_port() -> None:
    terminal = ProtocolIngressTerminalV1(
        ack_tx_result=AckTxResult.TX_DONE,
        t4_set_tx_attempted_monotonic_us=20,
        t5_tx_done_monotonic_us=21,
        t6_set_rx_issued_monotonic_us=22,
    )

    assert not hasattr(terminal, "__dict__")
    assert tuple(signature(ProtocolIngress).parameters) == (
        "queue",
        "monotonic_clock",
        "auth_node_keys",
    )
    with pytest.raises(FrozenInstanceError):
        terminal.ack_tx_result = AckTxResult.TX_TIMEOUT  # type: ignore[misc]
    with pytest.raises(ProtocolIngressInterfaceError):
        replace(terminal, ack_tx_result=1)  # type: ignore[arg-type]


# Copies startup authentication material and retains no caller-owned node map state.
def test_ingress_snapshots_the_minimal_authentication_map() -> None:
    node_keys = {NODE_ID: NODE_KEY}
    ingress = ProtocolIngress(
        queue=PersistQueue(capacity_entities=1),
        monotonic_clock=FakeOsClock(monotonic_us=0, realtime_us=0),
        auth_node_keys=node_keys,
    )

    node_keys.clear()

    assert dict(ingress._auth_node_keys) == {NODE_ID: NODE_KEY}
    with pytest.raises(TypeError):
        ingress._auth_node_keys[NODE_ID] = bytes(16)  # type: ignore[index]
    with pytest.raises(TypeError):
        copy.copy(ingress._auth_node_keys)


# Rejects malformed key material before the ingress path can enter receive handling.
@pytest.mark.parametrize(
    "node_keys",
    (
        {b"short": NODE_KEY},
        {NODE_ID: b"short"},
        {NODE_ID: bytearray(16)},
    ),
)
def test_ingress_rejects_invalid_startup_authentication_material(
    node_keys: dict[bytes, object],
) -> None:
    with pytest.raises(ProtocolIngressConfigurationError):
        ProtocolIngress(
            queue=PersistQueue(capacity_entities=1),
            monotonic_clock=FakeOsClock(monotonic_us=0, realtime_us=0),
            auth_node_keys=node_keys,  # type: ignore[arg-type]
        )
