"""Temporary zero-IRQ investigation, kept in one reproducible file.

configure() installs bounded command observations and starts the scope worker.
investigate() is the receiver's nonblocking RAM/GPIO hook. Only the worker
uses Ethernet or files. Single arming belongs to the operator.
No receiver imports are needed: this file can be retained independently.

Reproduction hooks (temporary, removable as a group):
  * After the process guard, configure(backend, instance_id, capture_directory).
  * In Radio.receive's zero-IRQ UNEXPECTED_IRQ branch, immediately after
    constructing its episode and before standby: investigate("zero_irq",
    edge=edge, started=started, event=event, episode=episode).
  * In Radio._result, after collecting completed episodes:
    investigate("radio_result", result=result).
  * After allocating this failed receive's occurrence_sequence:
    investigate("occurrence", packet=packet, sequence=occurrence_sequence).
  * After the RADIO diagnostic's existing allocation/admission attempt:
    investigate("diagnostic", episode=episode, diagnostic=diagnostic,
                admission=result.status).
  * On process teardown, close().
The builder exposes _trigger.at_us; completed episodes expose
sampled_at_monotonic_us. Both name the same existing episode start.
"""

import copy
import datetime
import hashlib
import json
import os
from pathlib import Path
import socket
import struct
import threading
import time

_session = None
MARKER_GPIO = 25
SCOPE_ADDRESS = ("10.11.13.220", 5025)
WIRING = {"C1": "GPIO25 marker, physical pin 22", "C2": "GPIO23 DIO1, pin 16",
          "C3": "GPIO9 MISO, pin 21", "C4": "GPIO24 BUSY, pin 18"}


def _json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


class Scope:
    """SDS800X HD SCPI transport; never starts or forces an acquisition."""

    def __init__(self, address=SCOPE_ADDRESS):
        self.socket = socket.create_connection(address, timeout=5)
        self.socket.settimeout(20)
        self.buffer = bytearray()
        self.deadline = None

    def close(self):
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()

    def send(self, command):
        self.socket.sendall((command + "\n").encode("ascii"))

    def exact(self, count):
        while len(self.buffer) < count:
            if self.deadline is not None and time.monotonic() >= self.deadline:
                raise TimeoutError("scope export exceeded 120 seconds")
            block = self.socket.recv(min(1048576, count - len(self.buffer)))
            if not block:
                raise RuntimeError("scope disconnected")
            self.buffer.extend(block)
        value = bytes(self.buffer[:count])
        del self.buffer[:count]
        return value

    def query(self, command):
        self.send(command)
        value = bytearray()
        while len(value) < 4096:
            byte = self.exact(1)
            if byte == b"\n":
                if value.strip():
                    return value.decode("ascii").strip()
                value.clear()
            else:
                value.extend(byte)
        raise ValueError("oversized SCPI text response")

    def binary(self, command):
        self.send(command)
        prefix = bytearray()
        while len(prefix) < 512:
            byte = self.exact(1)
            if byte == b"#":
                digits = int(self.exact(1))
                if not 1 <= digits <= 9:
                    raise ValueError("unsupported SCPI block")
                count = int(self.exact(digits))
                if not 0 < count <= 20000000:
                    raise ValueError("unexpected SCPI block size")
                return self.exact(count)
            prefix.extend(byte)
            if prefix[-8:] == b"\x89PNG\r\n\x1a\n":
                data = bytearray(prefix[-8:])
                while len(data) <= 20000000:
                    header = self.exact(8)
                    count = struct.unpack(">I", header[:4])[0]
                    if count > 20000000:
                        raise ValueError("oversized PNG chunk")
                    data.extend(header)
                    data.extend(self.exact(count + 4))
                    if header[4:] == b"IEND":
                        return bytes(data)
                raise ValueError("oversized PNG")
        raise ValueError("unrecognized binary reply")

    def settings(self):
        commands = ["*IDN?", ":TRIG:STAT?", ":TRIG:MODE?", ":TRIG:TYPE?",
                    ":TRIG:EDGE:SOUR?", ":TRIG:EDGE:SLOP?", ":TRIG:EDGE:LEV?",
                    ":TRIG:EDGE:COUP?", ":TIM:SCAL?", ":TIM:DEL?",
                    ":ACQ:MODE?", ":ACQ:MDEP?", ":ACQ:POIN?", ":ACQ:SRAT?",
                    ":ACQ:TYPE?"]
        commands += [f":CHAN{i}:{field}?" for i in range(1, 5)
                     for field in ("SWIT", "PROB", "SCAL", "OFFS", "COUP", "INVERT", "BWL")]
        return {command: self.query(command) for command in commands}

    def ready(self):
        # Validate the operator's configuration each time a new Single is seen.
        if self.query(":TRIG:STAT?") != "Ready":
            return False
        s = self.settings()
        expected = {":TRIG:MODE?": "SINGle", ":TRIG:TYPE?": "EDGE",
                    ":TRIG:EDGE:SOUR?": "C1", ":TRIG:EDGE:SLOP?": "RISing",
                    ":ACQ:MODE?": "YT", ":ACQ:TYPE?": "NORMal", ":ACQ:MDEP?": "10M"}
        if any(s[key] != value for key, value in expected.items()):
            raise ValueError("scope must use the documented Single configuration")
        for key, value in ((":TRIG:EDGE:LEV?", 1.65), (":TIM:SCAL?", .01),
                           (":TIM:DEL?", -.04), (":ACQ:SRAT?", 100000000)):
            if abs(float(s[key]) - value) > max(abs(value) * 1e-6, 1e-9):
                raise ValueError(f"unexpected scope setting {key}: {s[key]}")
        for i in range(1, 5):
            if s[f":CHAN{i}:SWIT?"] != "ON" or float(s[f":CHAN{i}:PROB?"]) != 10:
                raise ValueError("all four x10 probes must be enabled")
        return self.query(":TRIG:STAT?") == "Ready"

    def export(self, directory):
        self.deadline = time.monotonic() + 120
        deadline = time.monotonic() + 5
        while self.query(":TRIG:STAT?") != "Stop":
            if time.monotonic() >= deadline:
                raise TimeoutError("scope did not stop after the GPIO25 edge")
            time.sleep(.05)
        settings = self.settings()
        _json(directory / "scope-settings.json", settings)
        (directory / "capture.png").write_bytes(self.binary(":PRINt? PNG"))
        count = int(float(settings[":ACQ:POIN?"]))
        if count != 10000000:
            raise ValueError(f"expected 10M samples per channel, received {count}")
        for command in (":WAV:WIDT WORD", ":WAV:BYT LSB", ":WAV:INT 1"):
            self.send(command)
        if self.query(":WAV:WIDT?") != "WORD" or self.query(":WAV:BYT?") != "LSB":
            raise ValueError("scope did not accept WORD/LSB export")
        maximum = min(5000000, int(float(self.query(":WAV:MAXP?"))))
        if maximum <= 0:
            raise ValueError("invalid waveform transfer limit")
        for channel in range(1, 5):
            self.send(f":WAV:SOUR C{channel}")
            self.send(":WAV:STAR 0")
            self.send(f":WAV:POIN {min(count, maximum)}")
            descriptor = self.binary(":WAV:PRE?")
            if len(descriptor) < 346 or not descriptor.startswith(b"WAVEDESC"):
                raise ValueError("invalid waveform descriptor")
            (directory / f"C{channel}.wavedesc.bin").write_bytes(descriptor)
            fields = {"vdiv_before_probe": (156, "f"), "offset_before_probe": (160, "f"),
                      "code_per_div": (164, "f"), "sample_interval_s": (176, "f"),
                      "horizontal_delay_s": (180, "d"), "probe_attenuation": (328, "f")}
            _json(directory / f"C{channel}.parameters.json", {
                key: struct.unpack_from("<" + fmt, descriptor, offset)[0]
                for key, (offset, fmt) in fields.items()})
            with (directory / f"C{channel}.raw.i16le").open("wb") as stream:
                for start in range(0, count, maximum):
                    if self.query(":TRIG:STAT?") != "Stop":
                        raise RuntimeError("scope was rearmed during export; capture is incomplete")
                    points = min(maximum, count - start)
                    self.send(f":WAV:STAR {start}")
                    self.send(f":WAV:POIN {points}")
                    data = self.binary(":WAV:DATA?")
                    stream.write(data)
                    if len(data) != points * 2:
                        raise ValueError("incomplete waveform transfer")
                stream.flush()
                os.fsync(stream.fileno())
        if self.query(":TRIG:STAT?") != "Stop":
            raise RuntimeError("scope was rearmed before export completed")
        self.deadline = None


class _RecordingIo:
    def __init__(self, io, session):
        self.io, self.session = io, session

    def __getattr__(self, name):
        return getattr(self.io, name)

    def transfer(self, data, *, deadline_monotonic_us):
        if data[0] != 0x12:
            return self.io.transfer(data, deadline_monotonic_us=deadline_monotonic_us)
        start = self.session.clock.now_monotonic_us()
        result = self.io.transfer(data, deadline_monotonic_us=deadline_monotonic_us)
        self.session.last_irq_read = dict(tx_hex=data.hex(), rx_hex=result.hex(),
            transfer_started_monotonic_us=start,
            transfer_finished_monotonic_us=self.session.clock.now_monotonic_us())
        return result


class Investigation:
    """One capture in flight, at most 64 retained RAM detail records."""

    def __init__(self, backend, instance_id, directory, *, marker=None, scope=None):
        self.backend, self.clock = backend, backend.clock
        self.instance_id = instance_id.hex()
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.closing = False
        self.armed = False
        self.active = None
        self.records = {}
        self.dirty = set()
        self.completed = set()
        self.lost_details = 0
        self.last_irq_read = None
        self.last_clear = None
        self.error = None
        self.last_status = None
        self.scope = scope
        self.marker = marker
        self.original_io = backend.io
        self.original_clear = backend.clear_irq
        self.thread = None
        try:
            if self.marker is None:
                import gpiod
                self.marker = gpiod.request_lines(backend.configuration.gpio_chip,
                    consumer="cura-unexpected-irq", config={MARKER_GPIO: gpiod.LineSettings(
                        direction=gpiod.line.Direction.OUTPUT, bias=gpiod.line.Bias.AS_IS,
                        output_value=gpiod.line.Value.INACTIVE)})
            self._level(False)
            if self.scope is None:
                self.scope = Scope()
            identity = self.scope.query("*IDN?")
            if "SDS804X HD" not in identity:
                raise ValueError(f"unexpected scope identity: {identity}")
            backend.io = _RecordingIo(backend.io, self)
            backend.clear_irq = self.clear_irq
            _json(self.root / "session.json", dict(receiver_instance_id=self.instance_id,
                utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                linux_boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                scope_idn=identity, wiring=WIRING,
                module_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
            self.thread = threading.Thread(target=self.run, name="scope-capture", daemon=True)
            self.thread.start()
        except BaseException:
            self.close()
            raise

    def _level(self, high):
        # Test markers implement set(high); libgpiod requests use set_value().
        if hasattr(self.marker, "set"):
            self.marker.set(high)
        else:
            import gpiod
            self.marker.set_value(MARKER_GPIO, gpiod.line.Value.ACTIVE if high else gpiod.line.Value.INACTIVE)

    def clear_irq(self, mask, deadline):
        start = self.clock.now_monotonic_us()
        outcome = "CONFIRMED_APPLIED"
        try:
            return self.original_clear(mask, deadline)
        except Exception as error:
            outcome = getattr(getattr(getattr(error, "failure", None), "outcome", None), "name", "UNKNOWN")
            raise
        finally:
            self.last_clear = dict(mask=mask, started_monotonic_us=start,
                finished_monotonic_us=self.clock.now_monotonic_us(), outcome=outcome)

    def observe(self, action, **facts):
        with self.lock:
            if self.closing:
                return
            if action == "zero_irq":
                if len(self.records) >= 64:
                    self.lost_details += 1
                    return
                edge, event, episode = facts["edge"], facts["event"], facts["episode"]
                key = episode._trigger.at_us
                pins = {"sample_started_monotonic_us": self.clock.now_monotonic_us()}
                for name in ("dio1", "busy"):
                    try:
                        pins[name] = getattr(self.original_io, name)()
                    except Exception as error:
                        pins[name + "_error"] = type(error).__name__
                pins["sample_finished_monotonic_us"] = self.clock.now_monotonic_us()
                record = dict(receiver_instance_id=self.instance_id, diagnostic_sequence=None,
                    occurrence_sequence=None, episode_started_monotonic_us=key,
                    dio1_sequence=edge.sequence, edge_timestamp_monotonic_ns=edge.timestamp_ns,
                    handler_started_monotonic_us=facts["started"], original_irq_read=self.last_irq_read,
                    irq_status=event.irq_status, chip_status=event.chip_status,
                    device_errors=event.device_errors, pins=pins, last_irq_clear=self.last_clear,
                    last_set_rx_issued_monotonic_us=self.backend.last_set_rx_issued_us,
                    marker_write_started_monotonic_us=None, marker_write_finished_monotonic_us=None,
                    capture_status="SKIPPED_SCOPE_NOT_READY" if not self.armed else "SKIPPED_CAPTURE_BUSY")
                if self.armed and self.active is None and self.error is None:
                    self.armed = False
                    record["marker_write_started_monotonic_us"] = self.clock.now_monotonic_us()
                    try:
                        self._level(True)
                        record["capture_status"] = "MARKED"
                        self.active = key
                    except Exception as error:
                        record["capture_status"] = "MARKER_FAILED"
                        record["capture_error"] = repr(error)
                        self.error = "marker write failed; restart investigation"
                        try:
                            self._level(False)
                        except Exception as cleanup_error:
                            record["marker_cleanup_error"] = repr(cleanup_error)
                    record["marker_write_finished_monotonic_us"] = self.clock.now_monotonic_us()
                self.records[key] = record
            elif action == "radio_result":
                for episode in facts["result"].episodes:
                    key = episode.sampled_at_monotonic_us
                    if key in self.records:
                        self.records[key]["recovery_terminal_state"] = episode.context.terminal_state.name
                        self.records[key]["recovery_finished_monotonic_us"] = key + episode.context.episode_duration_us
                        self.completed.add(key)
                        self.dirty.add(key)
            elif action == "occurrence":
                for key, record in self.records.items():
                    if record["edge_timestamp_monotonic_ns"] == facts["packet"].edge_timestamp_ns:
                        record["occurrence_sequence"] = facts["sequence"]
                        if key in self.completed:
                            self.dirty.add(key)
            elif action == "diagnostic":
                key = facts["episode"].sampled_at_monotonic_us
                if key in self.records:
                    self.records[key]["diagnostic_sequence"] = facts["diagnostic"].diagnostic_sequence
                    self.records[key]["diagnostic_admission"] = facts["admission"].name
                    self.dirty.add(key)
            self.wake.set()

    def directory(self, record):
        return self.root / "captures" / (self.instance_id + "-dio1-" + str(record["dio1_sequence"])
                                        + "-" + str(record["edge_timestamp_monotonic_ns"]))

    def save_record(self, key):
        with self.lock:
            record = copy.deepcopy(self.records[key])
        directory = self.directory(record)
        directory.mkdir(parents=True, exist_ok=True)
        _json(directory / "incident.json", record)
        manifest = []
        for path in sorted(directory.iterdir()):
            if path.is_file() and path.name != "SHA256SUMS":
                with path.open("rb") as stream:
                    digest = hashlib.file_digest(stream, "sha256").hexdigest()
                manifest.append(f"{digest}  {path.name}\n")
        (directory / "SHA256SUMS").write_text("".join(manifest))

    def status(self):
        with self.lock:
            value = dict(armed=self.armed, capture_active=self.active is not None,
                         error=self.error, dropped_detail_records=self.lost_details,
                         receiver_instance_id=self.instance_id, closed=self.closing)
        if value != self.last_status:
            _json(self.root / "status.json", value)
            self.last_status = value

    def run(self):
        try:
            while not self.closing:
                with self.lock:
                    jobs = sorted(self.dirty)
                    self.dirty.clear()
                for key in jobs:
                    with self.lock:
                        capture = self.active == key and self.records[key]["capture_status"] == "MARKED"
                    if capture:
                        directory = self.directory(self.records[key])
                        directory.mkdir(parents=True, exist_ok=True)
                        self.save_record(key)
                        try:
                            if shutil_free_bytes(self.root) < (1 << 30) + 100000000:
                                raise OSError("capture would consume receiver's 1 GiB free-space reserve")
                            self.scope.export(directory)
                            with self.lock:
                                self.records[key]["capture_status"] = "SAVED"
                        except Exception as error:
                            with self.lock:
                                self.records[key]["capture_status"] = "FAILED"
                                self.records[key]["capture_error"] = repr(error)
                                self.error = "capture failed; retain evidence and restart investigation"
                        finally:
                            with self.lock:
                                try:
                                    self._level(False)
                                    self.records[key]["marker_lowered_monotonic_us"] = self.clock.now_monotonic_us()
                                except Exception as error:
                                    self.records[key]["capture_status"] = "FAILED"
                                    self.records[key]["marker_cleanup_error"] = repr(error)
                                    self.error = "could not lower GPIO25; stop investigation"
                                self.active = None
                    self.save_record(key)
                if self.error is None:
                    try:
                        ready = self.scope.query(":TRIG:STAT?") == "Ready"
                        if ready and not self.armed:
                            ready = self.scope.ready()
                        with self.lock:
                            self.armed = (ready and self.active is None
                                          and self.error is None and not self.closing)
                    except Exception as error:
                        with self.lock:
                            self.armed = False
                            self.error = repr(error)
                self.status()
                # Keep identities until their diagnostic attempt has been observed.
                with self.lock:
                    for key in list(self.records):
                        if (key != self.active and key not in self.dirty and key in self.completed
                                and self.records[key]["diagnostic_sequence"] is not None):
                            del self.records[key]
                            self.completed.discard(key)
                self.wake.wait(.5)
                self.wake.clear()
        except Exception as error:
            with self.lock:
                self.error = repr(error)
                self.armed = False
            try:
                self.status()
            except OSError:
                pass
        finally:
            with self.lock:
                if self.marker is not None:
                    try:
                        self._level(False)
                    except Exception:
                        pass

    def close(self):
        with self.lock:
            self.closing = True
            self.armed = False
        self.wake.set()
        if self.thread is not None:
            self.thread.join(2)
        if self.scope is not None:
            self.scope.close()
        if self.thread is not None:
            self.thread.join(3)
        with self.lock:
            if self.marker is not None:
                try:
                    self._level(False)
                finally:
                    if hasattr(self.marker, "reconfigure_lines"):
                        import gpiod
                        self.marker.reconfigure_lines({MARKER_GPIO: gpiod.LineSettings(
                            direction=gpiod.line.Direction.INPUT, bias=gpiod.line.Bias.AS_IS)})
                    self.marker.release()
                    self.marker = None
        self.backend.io = self.original_io
        self.backend.clear_irq = self.original_clear
        for key in list(self.records):
            if self.records[key]["capture_status"] == "MARKED":
                self.records[key]["capture_status"] = "INCOMPLETE_SHUTDOWN"
            self.save_record(key)
        with self.lock:
            self.active = None
        self.status()


def shutil_free_bytes(path):
    info = os.statvfs(path)
    return info.f_bavail * info.f_frsize


def configure(backend, instance_id, directory):
    global _session
    if directory:
        if _session is not None:
            raise RuntimeError("investigation already configured")
        _session = Investigation(backend, instance_id, directory)


def investigate(action, **facts):
    """Temporary production hook; failures cannot replace radio recovery."""
    session = _session
    if session is not None:
        try:
            session.observe(action, **facts)
        except Exception as error:
            session.error = repr(error)
            session.armed = False


def close():
    global _session
    if _session is not None:
        _session.close()
        _session = None
