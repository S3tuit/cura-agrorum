"""One USB identity and serial handle across native S3 reset/deep sleep."""
from pathlib import Path
import re
import threading
import time

import serial
from serial.tools import list_ports


def hard_reset_native_usb(port):
    # The ROM subclass's hard_reset also exchanges register commands. That
    # would compete with the capture reader and is inappropriate in an app.
    from esptool.reset import HardReset
    # Reconnection leaves both lines asserted; release DTR before pulsing RTS.
    port.setDTR(False)
    HardReset(port, uses_usb=True).reset()


def fingerprint(info):
    return info.vid, info.pid, info.serial_number, info.location


def usb_identity(port, *, ports=None):
    """Read-only admission before a runner opens or flashes its DUT."""
    if not isinstance(port, str) or not port:
        raise ValueError("select the XIAO native USB port explicitly")
    found = [p for p in (ports or list_ports.comports)()
             if Path(p.device).resolve() == Path(port).resolve()]
    if len(found) != 1 or not found[0].serial_number:
        raise ValueError("select one ESP32S3 native USB Serial/JTAG device with a USB serial number")
    if (found[0].vid, found[0].pid) != (0x303A, 0x1001):
        raise ValueError(
            f"selected USB interface {found[0].vid or 0:04x}:{found[0].pid or 0:04x} "
            "is not ESP32S3 native USB Serial/JTAG (303a:1001). For a first flash, "
            "hold BOOT while reconnecting USB (or use BOOT + RESET) to enter ROM "
            "download mode, then select its native USB port")
    return fingerprint(found[0])


class USBSerial:
    _serial_settings = frozenset({"baudrate", "bytesize", "parity", "stopbits", "timeout",
                                  "write_timeout", "xonxoff", "rtscts", "dsrdtr", "inter_byte_timeout"})

    def __setattr__(self, name, value):
        if name in self._serial_settings and "_raw" in self.__dict__:
            with self._lock:
                setattr(self._raw, name, value)
                self._settings = self._raw.get_settings()
        else:
            object.__setattr__(self, name, value)

    def __init__(self, raw, *, handshake=False, reconnect_timeout=650,
                 ports=None, opener=None, clock=time.monotonic):
        self._ports = ports or list_ports.comports
        self._opener = opener or serial.Serial
        self._clock = clock
        self._lock = threading.RLock()
        self._raw = raw
        self._selected = raw.port
        self._settings = raw.get_settings()
        self.identity = usb_identity(raw.port, ports=self._ports)
        self.reconnect_timeout = reconnect_timeout
        self.handshake = handshake
        self.failure = None
        self._absent_since = None
        self._closed = False
        self._tail = b""
        self._acknowledged = None
        self._acknowledged_at = 0.0
        self.on_data = None
        # POSIX pyserial otherwise waits for writability after os.write has
        # accepted even a complete command. A command that starts deep sleep
        # can remove USB during that unbounded wait, also blocking the reader
        # on our lock. Submit once without waiting; a short write is fatal.
        self.write_timeout = 0

    @property
    def port(self):
        return self._raw.port

    @property
    def closed(self):
        return self._closed

    def __getattr__(self, name):
        return getattr(self._raw, name)

    def _lost(self):
        self._raw.close()
        if self._absent_since is None:
            self._absent_since = self._clock()
        self._tail = b""

    def _connected(self):
        self.check_health()
        if self._closed:
            return False
        if self._raw.is_open:
            return True
        if self._absent_since is None:
            self._absent_since = self._clock()
        if (self.reconnect_timeout is not None and
                self._clock() - self._absent_since >= self.reconnect_timeout):
            raise RuntimeError("USB reconnect deadline expired")
        available = list(self._ports())
        matching = [p for p in available if fingerprint(p) == self.identity]
        if len(matching) > 1:
            raise RuntimeError("ambiguous USB device identity")
        if not matching:
            if any(p.device == self._selected for p in available):
                raise RuntimeError("selected USB path now belongs to another device")
            return False
        # Linux CDC-ACM asserts both lines on open. Keep that state: changing
        # DTR and RTS individually to false briefly asserts reset after wake.
        candidate = self._opener(port=None, exclusive=True, **self._settings)
        candidate.dtr = True
        candidate.rts = True
        candidate.port = matching[0].device
        try:
            candidate.open()
        except (OSError, serial.SerialException):
            candidate.close()
            return False
        self._raw = candidate
        self._absent_since = None
        return True

    def _read(self, size=None):
        with self._lock:
            try:
                if not self._connected():
                    return b""
                data = self._raw.read_all() if size is None else self._raw.read(size)
            except (OSError, serial.SerialException):
                self._lost()
                return b""
            except RuntimeError as error:
                self.failure = error
                raise
            if self.handshake:
                self._tail = (self._tail + data)[-4096:]
                for purpose, nonce in re.findall(rb"(?:^|\n)CURAG_USB_(SLEEP_)?READY ([0-9a-f]{8})\r?\n", self._tail):
                    now = self._clock()
                    # The first successful host write after USB enumeration
                    # can be lost. Only a fresh greeting permits retrying this
                    # idempotent ACK; action commands are never replayed.
                    receipt = purpose, nonce
                    if receipt != self._acknowledged or now - self._acknowledged_at >= 0.5:
                        self.write(b"CURAG_USB_" + purpose + b"ACK " + nonce + b"\n")
                        self._acknowledged = receipt
                        self._acknowledged_at = now
                # Retain only a possibly incomplete final line, not old nonces.
                self._tail = self._tail.rsplit(b"\n", 1)[-1]
            if data and self.on_data is not None:
                try:
                    self.on_data(data)
                except BaseException as error:
                    self.failure = error
                    raise
            return data

    def read_all(self):
        return self._read()

    def read(self, size=1):
        return self._read(size)

    def write(self, data):
        with self._lock:
            try:
                if not self._connected():
                    raise RuntimeError("USB unavailable for command; command was not sent")
                count = self._raw.write(data)
                if count != len(data):
                    raise RuntimeError("partial USB command; do not resend")
                return count
            except (OSError, serial.SerialException) as error:
                self.failure = RuntimeError("USB command outcome uncertain; do not resend")
                raise self.failure from error
            except RuntimeError as error:
                self.failure = error
                raise

    def check_health(self):
        if self.failure is not None:
            raise RuntimeError("USB capture failed: " + str(self.failure))

    def close(self):
        with self._lock:
            self._closed = True
            self._raw.close()


def attach_usb_console(dut, *, reconnect_timeout=650):
    """Replace the plugin's split reader/writer handles after its guarded flash."""
    channel = dut.serial
    if hasattr(channel, "_cura_usb"):
        return channel._cura_usb
    wrapped = USBSerial(channel.proc, handshake=True, reconnect_timeout=reconnect_timeout)
    channel.stop_redirect_thread()
    channel.proc = wrapped
    channel.esp._port = wrapped
    channel._cura_usb = wrapped
    # IDFDut caches this callback when constructed, before the guarded attach.
    def usb_reset():
        hard_reset_native_usb(wrapped)
    channel.hard_reset = usb_reset
    dut._hard_reset_func = usb_reset
    channel.start_redirect_thread()
    return wrapped
