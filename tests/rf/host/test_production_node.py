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


@pytest.mark.parametrize("transition", ("start", "reset_failure", "close"))
def test_usb_start_preserves_rom_ownership_and_never_resets_on_open(tmp_path, monkeypatch, transition):
    import sys
    import time
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

    def reset(rom):
        assert not node.thread.is_alive(), 'UART reader could consume ROM replies'
        probes.append('watchdog')
        rom._port.data = b'first application output\n'
        if transition == 'reset_failure':
            raise OSError('uncertain reset')

    monkeypatch.setattr(ESP32S3ROM, 'watchdog_reset', reset)
    node = NodeUART('/dev/ttyACM0', tmp_path)
    try:
        assert node.port.open_resets == 0
        assert probes == [dict(mode='no-reset', attempts=1),
                          (ESP32S3ROM.RTC_CNTL_OPTION1_REG, 0,
                           ESP32S3ROM.RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK)]
        assert not node.thread.is_alive()
        if transition == 'close':
            node.close()
            with pytest.raises(RuntimeError, match='cannot start'):
                node.start()
            assert 'watchdog' not in probes
            return
        if transition == 'reset_failure':
            with pytest.raises(OSError, match='uncertain reset'):
                node.start()
        else:
            node.start()
        deadline = time.monotonic() + 2
        while not (tmp_path / 'node-uart.bin').read_bytes() and time.monotonic() < deadline:
            time.sleep(.01)
        assert (tmp_path / 'node-uart.bin').read_bytes() == b'first application output\n'
        with pytest.raises(RuntimeError, match='cannot start'):
            node.start()
        assert probes.count('watchdog') == 1
        assert node.port.lines == []  # No USB line reset at either boundary.
    finally:
        node.close()
