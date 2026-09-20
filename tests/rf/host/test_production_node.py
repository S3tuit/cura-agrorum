import json
from types import SimpleNamespace

import pytest

from production_node import identify, verify_installed, capture_storage, STORAGE_SIZE


def test_readonly_capture_binds_complete_image_and_never_restarts(tmp_path, monkeypatch):
    commands = []
    fixture = dict(c6_uart="/dev/ttyUSB0", c6_dut="cc8da2fc0224")

    def invoke(command, **kwargs):
        commands.append(command)
        if "read-flash" in command:
            from pathlib import Path
            Path(command[-1]).write_bytes(b"x" * STORAGE_SIZE)
        return SimpleNamespace(returncode=0, stdout="BASE MAC: cc:8d:a2:fc:02:24\n", stderr="")

    monkeypatch.setattr("production_node.subprocess.run", invoke)
    monkeypatch.setattr("production_node.decode_image", lambda *_: {"logs": {}})
    seal = dict(node_id="01" * 8, files={"cura_agrorum_firmware.bin": "app", "partition_table/partition-table.bin": "table"})
    capture_storage(fixture, tmp_path, "after", tmp_path / "reader", seal, "a" * 32)
    assert all(command[command.index("--after") + 1] == "no-reset" for command in commands)
    assert commands[-1][-4:-1] == ["read-flash", "0x110000", "0x2e0000"]
    assert json.loads((tmp_path / "after-binding.json").read_text())["application_sha256"] == "app"
    with pytest.raises(ValueError, match="already exists"):
        capture_storage(fixture, tmp_path, "after", tmp_path / "reader", seal, "a" * 32)


def test_wrong_physical_identity_stops_before_read(tmp_path, monkeypatch):
    monkeypatch.setattr("production_node.esptool", lambda *_: SimpleNamespace(stdout="MAC: 00:00:00:00:00:01\n"))
    with pytest.raises(ValueError, match="wrong C6"):
        identify(dict(c6_dut="cc8da2fc0224"), tmp_path, "mac")


def test_changed_source_rejects_before_uart(tmp_path, monkeypatch):
    def changed(_):
        raise ValueError("source changed")
    monkeypatch.setattr("production_node.verify_build", changed)
    monkeypatch.setattr("production_node.identify", lambda *_: pytest.fail("device accessed"))
    with pytest.raises(ValueError, match="source changed"):
        verify_installed({}, tmp_path, tmp_path)


def test_uart_holds_reset_until_explicit_start(tmp_path, monkeypatch):
    import time
    import serial
    from esptool.targets.esp32c6 import ESP32C6ROM
    from production_node import NodeUART

    class Port:
        def __init__(self, **kwargs):
            self.dtr = self.rts = True
            self.opened = False
            self.releases = 0

        def open(self):
            self.opened = True
            # Opening must not release EN on the reviewed reset circuit.
            assert self.dtr is False and self.rts is True

        def read(self, size):
            time.sleep(.001)
            return b""

        def isOpen(self):
            return self.opened

        def setRTS(self, value):
            if self.rts and not value and not self.dtr:
                self.releases += 1
            self.rts = value

        def setDTR(self, value):
            self.dtr = value

        def close(self):
            self.opened = False

    monkeypatch.setattr(serial, "Serial", Port)
    monkeypatch.setattr(ESP32C6ROM, "uses_hardware_flow_control", lambda self: False)
    node = NodeUART("reviewed-uart", tmp_path)
    try:
        assert node.port.releases == 0 and node.port.rts is True
        node.start()  # Exercise the real esptool reset sequence on the fake pins.
        assert node.port.releases == 1
        assert node.port.rts is False and node.port.dtr is False
    finally:
        node.close()
