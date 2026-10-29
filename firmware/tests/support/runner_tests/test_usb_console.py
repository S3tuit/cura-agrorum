"""Exercise one serial object across USB loss, re-enumeration and new boots."""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from usb_console import USBSerial, attach_usb_console


class Port:
    def __init__(self, port='/dev/ttyACM0', **settings):
        self.port = port
        self.is_open = port is not None
        self.baudrate = settings.get('baudrate', 115200)
        self.timeout = settings.get('timeout', .1)
        self.write_timeout = settings.get('write_timeout')
        self.data = b''
        self.writes = []
        self.lines = []
        self.broken = False

    def get_settings(self):
        return dict(baudrate=self.baudrate, timeout=self.timeout,
                    write_timeout=self.write_timeout)

    def open(self):
        # CDC-ACM opens with both lines asserted. Deasserting them separately
        # passes through DTR=false/RTS=true and resets the native USB MCU.
        self.open_resets = int(not self.dtr)
        self.is_open = True

    def close(self):
        self.is_open = False

    def read_all(self):
        if self.broken:
            raise OSError('device detached')
        result, self.data = self.data, b''
        return result

    def read(self, size):
        return self.read_all()[:size]

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def isOpen(self):
        return self.is_open

    def setRTS(self, value):
        self.lines.append(('RTS', value))
        self.rts = value

    def setDTR(self, value):
        self.lines.append(('DTR', value))
        self.dtr = value


def info(device='/dev/ttyACM0', serial='s3-selected', location='1-2'):
    return SimpleNamespace(device=device, vid=0x303a, pid=0x1001,
                           serial_number=serial, location=location)


@pytest.fixture
def transport():
    raw = Port()
    ports = [info()]
    opened = []
    now = [0.]
    def opener(**kwargs):
        port = Port(**kwargs)
        opened.append(port)
        return port
    wrapped = USBSerial(raw, ports=lambda: ports, opener=opener,
                        clock=lambda: now[0], reconnect_timeout=10, handshake=True)
    return wrapped, raw, ports, opened, now


def test_reconnect_preserves_writer_settings_and_never_resets(transport):
    usb, raw, ports, opened, now = transport
    usb.baudrate = 460800  # Esptool assignments must reach the real handle.
    assert raw.baudrate == 460800
    raw.broken = True
    assert usb.read_all() == b''
    ports.clear()
    now[0] = 9
    assert usb.read_all() == b''
    ports.append(info('/dev/ttyACM9'))
    assert usb.read_all() == b''
    assert usb.port == '/dev/ttyACM9' and opened[0].baudrate == 460800
    assert opened[0].write_timeout == 0
    assert opened[0].lines == [] and opened[0].open_resets == 0
    usb.write(b'phase 1\n')
    assert opened[0].writes == [b'phase 1\n'] and raw.writes == []


@pytest.mark.parametrize('change', ['wrong_device', 'ambiguous', 'deadline'])
def test_reconnect_rejects_substitution_ambiguity_and_expired_absence(transport, change):
    usb, raw, ports, opened, now = transport
    raw.broken = True
    usb.read_all()
    ports.clear()
    if change == 'wrong_device': ports.append(info(serial='another-s3'))
    if change == 'ambiguous': ports.extend([info('/dev/ttyACM1'), info('/dev/ttyACM2')])
    if change == 'deadline': now[0] = 10
    with pytest.raises(RuntimeError): usb.read_all()
    with pytest.raises(RuntimeError, match='USB capture failed'): usb.check_health()
    assert not opened and not raw.writes


def test_partial_and_lost_commands_are_fatal_and_never_replayed(transport):
    usb, raw, ports, opened, now = transport
    raw.write = lambda data: len(data)-1
    with pytest.raises(RuntimeError, match='partial'): usb.write(b'start\n')
    with pytest.raises(RuntimeError): usb.write(b'start\n')
    assert not opened


def test_complete_command_does_not_wait_for_usb_after_device_sleeps(monkeypatch):
    import serial.serialposix as posix

    class PortWithoutIoctl(posix.Serial):
        def _reconfigure_port(self, force_update=False):
            pass

        def close(self):
            self.is_open = False

    raw = PortWithoutIoctl(port=None, timeout=.05)
    raw.port = '/dev/ttyACM0'
    raw.is_open = True
    raw.fd, raw.pipe_abort_write_r = 123, 456
    writes = []

    def accepted(fd, data):
        assert fd == 123
        writes.append(data)
        return len(data)  # The device can now consume this and power off USB.

    def disconnected_wait(readers, writers, errors, timeout):
        assert timeout is None
        raise RuntimeError('unbounded writable wait after complete write')

    monkeypatch.setattr(posix, 'os', SimpleNamespace(write=accepted))
    monkeypatch.setattr(posix, 'select', SimpleNamespace(select=disconnected_wait, error=OSError))
    # The actual installed POSIX writer exhibits the original stall boundary.
    with pytest.raises(RuntimeError, match='unbounded writable wait'):
        raw.write(b'control\n')
    usb = USBSerial(raw, ports=lambda: [info()])
    assert usb.write(b'CARRIER_HOLD_ACK transition-on\n') == 31
    assert writes == [b'control\n', b'CARRIER_HOLD_ACK transition-on\n']
    assert usb._settings['write_timeout'] == 0


def test_unavailable_command_is_not_queued_for_later(transport):
    usb, raw, ports, opened, now = transport
    raw.broken = True
    usb.read_all()
    ports.clear()
    with pytest.raises(RuntimeError, match='not sent'): usb.write(b'start\n')
    ports.append(info('/dev/ttyACM9'))
    with pytest.raises(RuntimeError): usb.read_all()
    assert not opened


def test_handshake_handles_split_lines_and_new_boot_nonce_without_duplicate_ack(transport):
    usb, raw, ports, opened, now = transport
    for data in [b'CURAG_USB_RE', b'ADY 12345678\r\n', b'CURAG_USB_READY 12345678\n',
                 b'CARRIER_BOOT identity\n', b'CURAG_USB_READY 87654321\n']:
        raw.data = data
        assert usb.read_all() == data
    assert raw.writes == [b'CURAG_USB_ACK 12345678\n', b'CURAG_USB_ACK 87654321\n']


def test_sleep_receipt_is_distinct_from_boot_and_handles_split_greeting(transport):
    usb, raw, _, _, now = transport
    for data in [b'CURAG_USB_READY 12345678\n', b'CURAG_USB_SLEEP_RE',
                 b'ADY 12345678\r\n', b'CURAG_USB_SLEEP_READY 12345678\n']:
        raw.data = data
        assert usb.read_all() == data
    assert raw.writes == [b'CURAG_USB_ACK 12345678\n', b'CURAG_USB_SLEEP_ACK 12345678\n']
    now[0] = .5
    raw.data = b'CURAG_USB_SLEEP_READY 12345678\n'
    usb.read_all()
    assert raw.writes[-1] == b'CURAG_USB_SLEEP_ACK 12345678\n'
    assert len(raw.writes) == 3


def test_lost_boot_ack_retries_only_on_a_fresh_paced_greeting(transport):
    usb, raw, ports, opened, now = transport
    greeting = b'CURAG_USB_READY 12345678\n'
    ack = b'CURAG_USB_ACK 12345678\n'
    raw.data = greeting
    usb.read_all()  # Model a successful host write lost during enumeration.
    assert raw.writes == [ack]
    now[0] = .25
    raw.data = greeting * 2
    usb.read_all()
    assert raw.writes == [ack]
    now[0] = .5
    usb.read_all()  # Elapsed time alone cannot trigger a write.
    assert raw.writes == [ack]
    raw.data = greeting * 2
    usb.read_all()
    assert raw.writes == [ack, ack]
    now[0] = 1
    raw.data = b'Press ENTER to see the list of tests.\n'
    usb.read_all()
    assert raw.writes == [ack, ack]


def test_deliberate_reset_releases_dtr_after_asserted_reconnect(monkeypatch):
    from usb_console import hard_reset_native_usb
    raw = Port()
    raw.dtr = raw.rts = True
    monkeypatch.setattr('esptool.reset.time.sleep', lambda seconds: None)
    hard_reset_native_usb(raw)
    assert raw.lines[0] == ('DTR', False)
    assert ('RTS', True) in raw.lines
    assert raw.dtr is raw.rts is False


def test_attach_replaces_reader_writer_esptool_and_cached_reset(monkeypatch):
    raw = Port()
    resets = []
    esp = SimpleNamespace(_port=raw, hard_reset=lambda **kw: resets.append(kw))
    channel = SimpleNamespace(proc=raw, esp=esp, stop_redirect_thread=lambda: None,
                              start_redirect_thread=lambda: None)
    dut = SimpleNamespace(serial=channel, _hard_reset_func=lambda: pytest.fail('stale reset'))
    monkeypatch.setattr('usb_console.list_ports.comports', lambda: [info()])
    monkeypatch.setattr('usb_console.hard_reset_native_usb', lambda port: resets.append(port))
    usb = attach_usb_console(dut)
    assert channel.proc is esp._port is usb
    channel.hard_reset()
    dut._hard_reset_func()
    assert resets == [usb, usb]
    assert attach_usb_console(dut) is usb


def test_explicit_unlimited_exploration_preserves_identity_guard(transport):
    usb, raw, ports, opened, now = transport
    usb.reconnect_timeout = None
    raw.broken = True
    usb.read_all()
    ports.clear()
    now[0] = 3600
    assert usb.read_all() == b''
    ports.append(info(serial='another-s3'))
    with pytest.raises(RuntimeError, match='another device'): usb.read_all()
    assert not opened


def test_native_usb_admission_precedes_bare_board_flash_fixture(monkeypatch):
    import importlib.util
    path = Path(__file__).resolve().parents[2]/'target/pytest_node_persistence.py'
    spec = importlib.util.spec_from_file_location('bare_usb_admission', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    adapter = info()
    adapter.vid, adapter.pid = 0x10c4, 0xea60
    monkeypatch.setattr('usb_console.list_ports.comports', lambda: [adapter])
    request = SimpleNamespace(config=SimpleNamespace(getoption=lambda name: adapter.device),
        getfixturevalue=lambda name: pytest.fail('flash fixture opened before USB admission'))
    with pytest.raises(ValueError, match='native USB'):
        module.test_node_persistence_on_device(request)


def test_factory_usb_cdc_explains_bootstrap_without_relaxing_native_guard(monkeypatch):
    from usb_console import usb_identity
    factory = info()
    factory.vid, factory.pid = 0x2886, 0x0059
    monkeypatch.setattr('usb_console.list_ports.comports', lambda: [factory])
    with pytest.raises(ValueError, match=r'2886:0059.*303a:1001.*BOOT \+ RESET'):
        usb_identity(factory.device)
