import json
from types import SimpleNamespace

import pytest

from production_node import identify, verify_installed, capture_storage, STORAGE_SIZE

@pytest.fixture(autouse=True)
def native_usb_admission(monkeypatch):
    monkeypatch.setattr("production_node.usb_identity", lambda port: None)



def test_readonly_capture_binds_complete_image_and_never_restarts(tmp_path, monkeypatch):
    commands = []
    fixture = dict(node_uart="/dev/ttyACM0", node_dut="cc8da2fc0224")

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
    with pytest.raises(ValueError, match="wrong S3"):
        identify(dict(node_dut="cc8da2fc0224"), tmp_path, "mac")


def test_changed_source_rejects_before_uart(tmp_path, monkeypatch):
    def changed(_):
        raise ValueError("source changed")
    monkeypatch.setattr("production_node.verify_build", changed)
    monkeypatch.setattr("production_node.identify", lambda *_: pytest.fail("device accessed"))
    with pytest.raises(ValueError, match="source changed"):
        verify_installed({}, tmp_path, tmp_path)


def test_usb_rom_admission_precedes_capture_and_explicit_native_reset(tmp_path, monkeypatch):
    import sys
    import serial
    from esptool.targets.esp32s3 import ESP32S3ROM
    from production_node import NodeUART
    sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[3] / 'firmware/tests/support/runner_tests'))
    from test_usb_console import Port, info
    probes = []
    monkeypatch.setattr(serial, 'Serial', Port)
    monkeypatch.setattr('usb_console.list_ports.comports', lambda: [info()])
    monkeypatch.setattr(ESP32S3ROM, 'connect', lambda self, **kw: probes.append(kw))
    monkeypatch.setattr(ESP32S3ROM, 'write_reg', lambda self, *args: probes.append(args))
    monkeypatch.setattr(ESP32S3ROM, 'uses_hardware_flow_control', lambda self: False)
    monkeypatch.setattr('esptool.reset.time.sleep', lambda _: None)
    node = NodeUART('/dev/ttyACM0', tmp_path)
    try:
        assert probes == [dict(mode='no-reset', attempts=1),
                          (ESP32S3ROM.RTC_CNTL_OPTION1_REG, 0,
                           ESP32S3ROM.RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK)]
        assert node.port.lines == []
        node.start()  # The real esptool native USB reset sequence on fake lines.
        assert [line for line in node.port.lines if line[0] == 'RTS'] == [('RTS', True), ('RTS', False)]
        assert all(value is False for name, value in node.port.lines if name == 'DTR')
    finally:
        node.close()
