"""Exercise the actual session owner with retained bytes and a fake USB endpoint."""
import json
from types import SimpleNamespace

import pytest

import production_node as production


@pytest.fixture
def session_rig(tmp_path, monkeypatch):
    def info(path="/dev/ttyACM0", serial="bound-board"):
        return SimpleNamespace(device=path, vid=0x303a, pid=0x1001,
                               serial_number=serial, location="3-3:1.0")
    ports = [info()]
    monkeypatch.setattr(production.list_ports, "comports", lambda: ports)
    commands, receipts, lifecycle = [], [], []

    class UART:
        def __init__(self, port, root, *, on_data, usb_binding):
            assert port == ports[0].device
            assert production.usb_identity(port) == usb_binding
            self.port = SimpleNamespace(identity=production.usb_identity(port), write=receipts.append)
            self.failure = None
            self.on_data = on_data
            self.closed = False
            self.close_error = False
            self.raw = (root / "node-uart.bin").open("xb")

        def emit(self, data):
            self.raw.write(data)
            self.raw.flush()
            try:
                self.on_data(data)
            except BaseException as error:
                self.failure = error
                raise

        def start(self):
            lifecycle.append("start")

        def close(self):
            self.closed = True
            self.raw.close()
            lifecycle.append("close")
            if self.close_error:
                raise RuntimeError("capture close failed")

    def invoke(command, **kwargs):
        assert session.node.closed, "ROM traffic must not compete with capture"
        commands.append(command)
        if "read-flash" in command:
            from pathlib import Path
            Path(command[-1]).write_bytes(b"x" * production.STORAGE_SIZE)
        return SimpleNamespace(returncode=0, stdout="BASE MAC: 00:11:22:33:44:55\n", stderr="")

    monkeypatch.setattr(production, "NodeUART", UART)
    monkeypatch.setattr(production.subprocess, "run", invoke)
    monkeypatch.setattr(production, "decode_image", lambda *_: dict(logs={}))
    seal = dict(node_id="01" * 8, sleep_seconds=10, sleep_observation=True,
                files={"cura_agrorum_firmware.bin": "app", "partition_table/partition-table.bin": "table"})
    session = production.ProductionSession(dict(node_uart="/dev/ttyACM0", node_dut="001122334455"),
        tmp_path, tmp_path / "reader", seal, "a" * 32, 2, stop_timeout=0)
    session.prepare()
    session.start()
    return session, ports, info, commands, receipts, lifecycle


def first_wake(session):
    session.node.emit(b"rst:0x1 (POWERON)\nRF_NODE_SLEEP duration_us=10000000\nRF_FINALIZE 11111111\n")


def last_wake(session):
    first_wake(session)
    session.node.emit(b"rst:0x5 (SLEEP_WAKEUP)\nRF_NODE_SLEEP duration_us=10000000\nRF_FINALIZE 22222222\n")


def test_final_wake_parks_before_rom_handoff_and_resolves_port_for_every_capture_command(session_rig):
    session, ports, info, commands, receipts, lifecycle = session_rig
    last_wake(session)
    assert receipts == [b"RF_FINALIZE_ACK 11111111 SLEEP\n", b"RF_FINALIZE_ACK 22222222 STOP\n"]
    assert not session.finalized
    with pytest.raises(RuntimeError, match="final capture"):
        session.capture_after()
    session.node.emit(b"RF_PARKED 22222222\n")
    assert session.finalized
    ports[:] = [info("/dev/ttyACM1")]
    session.stop()
    assert session.stopped and lifecycle == ["start", "close"]
    assert commands[0][commands[0].index("--port") + 1] == "/dev/ttyACM1"
    ports[:] = [info("/dev/ttyACM2")]
    session.capture_after()
    assert all(command[command.index("--port") + 1] == "/dev/ttyACM2" for command in commands[1:])
    assert all(command[command.index("--after") + 1] == "no-reset" for command in commands)
    assert json.loads((session.root / "node-stop.json").read_text())["wake"] == 2
    session.stop()
    assert len(commands) == 3 and lifecycle == ["start", "close"]


@pytest.mark.parametrize("fault", ["absent", "substitution", "ambiguous", "close", "no_park", "wrong_park", "extra_wake"])
def test_failed_stop_cannot_capture_or_replay_rom_commands(session_rig, fault):
    session, ports, info, commands, receipts, _ = session_rig
    last_wake(session)
    if fault == "wrong_park":
        with pytest.raises(ValueError, match="unbound"):
            session.node.emit(b"RF_PARKED deadbeef\n")
    elif fault == "extra_wake":
        session.node.emit(b"RF_PARKED 22222222\n")
        session.node.emit(b"rst:0x5 (SLEEP_WAKEUP)\n")
    elif fault != "no_park":
        session.node.emit(b"RF_PARKED 22222222\n")
    if fault == "absent": ports.clear()
    if fault == "substitution": ports[:] = [info(serial="another-board")]
    if fault == "ambiguous": ports.append(info("/dev/ttyACM1"))
    if fault == "close": session.node.close_error = True
    with pytest.raises((ValueError, RuntimeError, TimeoutError)):
        session.stop()
    with pytest.raises((ValueError, RuntimeError, TimeoutError)):
        session.stop()
    assert not commands and not session.stopped
    assert (session.root / "node-restoration-required.json").is_file()
    with pytest.raises(RuntimeError):
        session.capture_after()


def test_stop_request_freezes_uncertain_sleep_receipt_and_stops_next_admitted_boundary(session_rig):
    session, _, _, _, receipts, _ = session_rig
    first_wake(session)
    session.abort_requested = True
    session.receipt.clock = lambda: 1e12
    session.node.emit(b"RF_FINALIZE 11111111\n")
    assert receipts[-1] == b"RF_FINALIZE_ACK 11111111 SLEEP\n"
    session.node.emit(b"rst:0x5 (SLEEP_WAKEUP)\nRF_NODE_SLEEP duration_us=10000000\nRF_FINALIZE 22222222\nRF_PARKED 22222222\n")
    session.stop()
    assert receipts[-1] == b"RF_FINALIZE_ACK 22222222 STOP\n"


@pytest.mark.parametrize("raw", [b"RF_FINALIZE 11111111\n",
    b"rst:0x1 (POWERON)\nRF_NODE_SLEEP duration_us=10000000\nRF_NODE_SLEEP duration_us=10000000\nRF_FINALIZE 11111111\n"])
def test_unbound_finalization_does_not_send_a_receipt(session_rig, raw):
    session, _, _, _, receipts, _ = session_rig
    with pytest.raises(ValueError):
        session.node.emit(raw)
    assert receipts == []
    with pytest.raises(RuntimeError):
        session.stop()
