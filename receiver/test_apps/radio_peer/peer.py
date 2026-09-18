"""Finite raw RF peer. Production components, no service or durable allowance.

Run from an isolated, hash-verified tree as the configured service UID.
stdout contains only bounded JSON records. RF-006 uses the existing backend
directly, with its layer named in every run; all other cases use Radio.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, is_dataclass
from enum import Enum
import fcntl
import hashlib
import importlib
import json
import os
from pathlib import Path
import pwd
import re
import select
import signal
import sys
import threading

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "receiver"))
from cura_receiver.platform.linux_clocks import LinuxOsClock
from cura_receiver.platform.linux_radio import LinuxRadioIo
from cura_receiver.ports.radio import RadioConfiguration, RadioTxAuthorization
from cura_receiver.radio import Radio, State
from cura_receiver.sx1262 import Sx1262, IRQ_RX_DONE, IRQ_TX_DONE

A = bytes.fromhex("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e2f303132333435")
U2 = bytes.fromhex("808182838485868788898a8b8c8d8e8f909192939495969798999a9b9c9d9e9fa0a1a2a3a4a5a6a7a8a9aaabacadaeafb0b1b2b3b4b5")
B = bytes.fromhex("a0a1a2a3a4a5a6a7a8a9aaabacadaeafb0b1b2b3b4b5b6")
D2 = bytes.fromhex("c0c1c2c3c4c5c6c7c8c9cacbcccdcecfd0d1d2d3d4d5d6")
X = bytes.fromhex("e0e1e2e3e4e5e6e7e8e9eaebecedeeeff0f1f2f3f4f5f6")
# Expected RX count, maximum TX count. Kept independent of laptop declarations.
CASES = {
    "RF-001.exchange": (1, 1), "RF-003.silence": (1, 0),
    "RF-006.invalid": (1, 3), "RF-008.silence": (2, 0),
    "RF-008.exchange": (2, 2), "RF-009.untouched": (1, 0),
    "RF-009.initialized": (1, 0), "RF-010.wake": (2, 2),
    "RF-012.disconnected": (1, 0), "RF-013.absent": (0, 0),
}


def encoded(value):
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, Enum):
        return value.name
    if isinstance(value, bytes):
        return value.hex()
    raise TypeError(type(value).__name__)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def verify_sources(path):
    manifest = json.loads(path.read_text())
    require(manifest.get("schema") == 1 and bool(manifest.get("files")), "invalid source manifest")
    for name, expected in manifest["files"].items():
        file = REPO / name
        require(not Path(name).is_absolute() and ".." not in Path(name).parts and
                file.is_file() and not file.is_symlink(), "unsafe manifest path")
        require(hashlib.sha256(file.read_bytes()).hexdigest() == expected, f"source mismatch: {name}")
    require(str(Path(__file__).relative_to(REPO)) in manifest["files"], "peer missing from source manifest")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dependencies():
    """Deployment capability check before any GPIO acquisition or SPI object."""
    result = {}
    for name, minimum, maximum in (("gpiod", (2, 2), 3), ("spidev", (3, 8), 4)):
        module = importlib.import_module(name)
        version = module.__version__
        parsed = tuple(int(part) for part in version.split(".")[:2])
        require(minimum <= parsed and parsed[0] < maximum, f"unsupported {name} {version}")
        result[name] = version
    require(callable(importlib.import_module("spidev").SpiDev.open_path), "spidev.open_path unavailable")
    return result


class TraceIo(LinuxRadioIo):
    """Buffer actual SPI/edge facts, including uncertain attempts; never fake I/O."""

    def __init__(self, clock, maximum_tx):
        super().__init__(clock)
        self.clock, self.maximum_tx = clock, maximum_tx
        self.attempts = 0
        self.transmit_deadline = None
        self.trace = []

    def capture(self, operation, call, **fields):
        before = self.clock.now_monotonic_us()
        event = dict(operation=operation, before=before, **fields)
        try:
            result = call()
            event["result"] = result
            return result
        except BaseException as exc:
            event["error"] = type(exc).__name__ + ": " + str(exc)
            raise
        finally:
            event["after"] = self.clock.now_monotonic_us()
            require(len(self.trace) < 8192, "peer trace overflow")
            self.trace.append(event)

    def transfer(self, data, *, deadline_monotonic_us):
        if data[0] == 0x83:
            require(self.transmit_deadline is not None and self.clock.now_monotonic_us() <= self.transmit_deadline,
                    "peer is not armed or its emission deadline has expired")
            require(self.attempts < self.maximum_tx, "episode TX ceiling exceeded before SetTx")
            self.attempts += 1
        return self.capture("spi", lambda: super(TraceIo, self).transfer(
            data, deadline_monotonic_us=deadline_monotonic_us), tx=data.hex(), deadline=deadline_monotonic_us)

    def wait_edge(self, *, deadline_monotonic_us):
        return self.capture("edge", lambda: super(TraceIo, self).wait_edge(
            deadline_monotonic_us=deadline_monotonic_us), deadline=deadline_monotonic_us)

    def set_reset(self, *, asserted):
        return self.capture("reset", lambda: super(TraceIo, self).set_reset(asserted=asserted), asserted=asserted)

    def close(self):
        return self.capture("close", lambda: super(TraceIo, self).close())


def healthy(result, *states):
    require(result.state in states and not result.episodes, f"radio result: {result!r}")
    return result


def scheduled(clock, target):
    clock.wait_until_monotonic_us(target)
    require(clock.now_monotonic_us() <= target + 100_000, "missed local target; no catch-up TX")


def response(radio, packet, frame, sequence):
    clock = radio.clock
    scheduled(clock, packet.received_at_monotonic_us + 250_000)
    healthy(radio.prepare_ack(frame, occurrence_sequence=sequence), State.RX_EVENT_PENDING)
    healthy(radio.start_ack(RadioTxAuthorization(clock.now_monotonic_us() + 500_000,
                                               occurrence_sequence=sequence)), State.TX_ACTIVE)
    result = healthy(radio.finish_ack(), State.RX_SINGLE, State.RX_EVENT_PENDING)
    require(result.tx is not None and result.tx.ack_tx_result.name == "TX_DONE", "unconfirmed peer TX")
    return result


def lower_receive(backend, deadline):
    edge = backend.wait_edge(deadline_monotonic_us=min(deadline, backend.deadline(500_000)))
    if edge is None:
        return None
    require(backend.last_set_rx_issued_us <= edge.monotonic_us <= backend.clock.now_monotonic_us(), "invalid RX edge")
    end = backend.deadline(500_000)
    event = backend.observe_event(end)
    backend.validate_event(event, transmit=False)
    require(event.irq_status == IRQ_RX_DONE, "unexpected lower-layer RX IRQ")
    backend.standby(end)
    frame, rssi, snr, copied = backend.read_packet(end)
    backend.clear_irq(event.irq_status, end)
    return dict(frame=frame, rssi_dbm_x2=rssi, snr_db_x4=snr, edge_timestamp_ns=edge.timestamp_ns,
                t2_packet_copied_monotonic_us=copied, irq_status=event.irq_status, device_errors=event.device_errors)


def lower_burst(backend, packet, stop):
    """Finite sequence, existing fixed-profile primitives and watchdog unchanged."""
    records = []
    origin = packet["edge_timestamp_ns"] // 1000
    for frame, offset in zip((b"\x00", b"\xde\xad\xbe\xef", X), (250_000, 750_000, 1_250_000)):
        scheduled(backend.clock, origin + offset)
        require(not stop.is_set(), "peer stopped before burst TX")
        deadline = backend.deadline(500_000)
        backend.write_buffer(frame, deadline)
        backend.install_profile(transmit=True, payload_length=len(frame), deadline=deadline)
        backend.account_stale_irqs(deadline)
        backend.start_tx(deadline)
        issued = backend.last_set_tx_issued_us
        edge = backend.wait_edge(deadline_monotonic_us=issued + 250_000)
        require(edge is not None and issued <= edge.monotonic_us <= issued + 250_000 and
                edge.monotonic_us <= backend.clock.now_monotonic_us(), "unconfirmed burst TX edge")
        end = backend.deadline(500_000)
        event = backend.observe_event(end)
        backend.validate_event(event, transmit=True)
        require(event.irq_status == IRQ_TX_DONE, "burst TX did not complete")
        backend.standby(end)
        backend.clear_irq(event.irq_status, end)
        records.append(dict(frame=frame, target=origin + offset, set_tx=issued,
                            tx_done=edge.monotonic_us, event=event, certainty=backend.set_tx_outcome))
    deadline = backend.deadline(500_000)
    backend.install_profile(transmit=False, payload_length=255, deadline=deadline)
    backend.arm_receive(deadline)
    return records


def execute(case, backend, radio, stop, started):
    expected_count, maximum_tx = CASES[case]
    expected = (A, U2)[:expected_count]
    packets, transmissions = [], []
    deadline = started + 45_000_000
    quiet_end = started + 5_000_000 if not expected_count else deadline
    while backend.clock.now_monotonic_us() < min(deadline, quiet_end):
        require(not stop.is_set(), "control lost or episode lease expired")
        if radio:
            result = healthy(radio.receive(deadline_monotonic_us=min(deadline, quiet_end)),
                             State.RX_SINGLE, State.RX_EVENT_PENDING)
            packet = result.receive_event
            if packet is None or not packet.usable_for_ingress:
                continue
            frame = packet.frame
        else:
            packet = lower_receive(backend, min(deadline, quiet_end))
            if packet is None:
                continue
            frame = packet["frame"]
        require(len(packets) < expected_count, "extra uplink")
        require(frame == expected[len(packets)], "unexpected uplink bytes")
        packets.append(packet)
        if case == "RF-006.invalid":
            transmissions.extend(lower_burst(backend, packet, stop))
        elif maximum_tx:
            transmissions.append(response(radio, packet, B if len(packets) == 1 else D2, len(packets)))
        else:
            healthy(radio.rearm(), State.RX_SINGLE, State.RX_EVENT_PENDING)
        if len(packets) == expected_count:
            quiet_end = backend.clock.now_monotonic_us() + 4_000_000
    require(len(packets) == expected_count, "missing expected uplink")
    require(len(transmissions) == maximum_tx, "incomplete downlink sequence")
    return dict(packets=packets, transmissions=transmissions, observation_end=backend.clock.now_monotonic_us())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, required=True)
    parser.add_argument("--run", required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--check", action="store_true", help="verify sources, identity and dependencies without device access")
    args = parser.parse_args()
    require(re.fullmatch("[0-9a-f]{32}", args.run), "invalid run identity")
    seal = verify_sources(args.manifest)
    fixture = json.loads(args.fixture.read_text())
    require(os.geteuid() != 0 and pwd.getpwuid(os.geteuid()).pw_name == fixture["pi_user"], "wrong service UID")
    require(fixture["pi_fixture"] == "radio_nominal" and fixture["receiver_service_stopped"] is True and
            fixture["exclusive_radios"] is True and fixture["rtc_shunts_open"] is True, "unconfirmed Pi fixture")
    require(Path("/proc/device-tree/model").read_bytes().startswith(b"Raspberry Pi"), "peer requires a Pi")
    require(Path("/proc/device-tree/serial-number").read_bytes().rstrip(b"\0").decode() == fixture["pi_board_id"],
            "Pi identity differs from fixture")
    installed = dependencies()
    if args.check:
        print(json.dumps(dict(kind="preflight", source=seal, dependencies=installed, uid=os.geteuid(),
                              python=sys.executable, pi_board_id=fixture["pi_board_id"])))
        return 0
    # GPIO requests also enforce physical exclusivity against non-cooperating owners.
    lock_path = f"/tmp/cura-rf-radio-{os.geteuid()}.lock"
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        clock = LinuxOsClock()
        io = TraceIo(clock, CASES[args.case][1])
        backend = Sx1262(io, clock, clock, RadioConfiguration())
        radio = None if args.case == "RF-006.invalid" else Radio(backend)
        stop = threading.Event()
        identity = dict(run=args.run, case=args.case, pid=os.getpid(),
                        boot=Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                        source=seal, layer="Sx1262/LinuxRadioIo" if radio is None else "Radio/Sx1262/LinuxRadioIo")

        def emit(kind, **fields):
            print(json.dumps(dict(kind=kind, **identity, **fields), default=encoded), flush=True)

        def cancel(*_):
            stop.set()
            if radio:
                radio.request_shutdown()

        def watch_control():
            # GO is the only arming command; later EOF/STOP/malformed input cancels.
            sys.stdin.readline()
            cancel()

        for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
            signal.signal(sig, cancel)
        timer = None
        outcome, cleanup = {}, {}
        failure = None
        try:
            if radio:
                initialized = healthy(radio.initialize(), State.RX_SINGLE)
            else:
                end = backend.deadline(2_000_000)
                backend.open(end); backend.initialize(end); backend.arm_receive(end)
                initialized = dict(profile=backend.profile, set_rx=backend.last_set_rx_issued_us)
            emit("ready", initialized=initialized, at=clock.now_monotonic_us())
            require(select.select([sys.stdin], [], [], 15)[0], "GO readiness timeout")
            require(sys.stdin.readline().strip() == f"GO {args.run} {args.case}", "invalid GO identity")
            started = clock.now_monotonic_us()
            io.transmit_deadline = started + 44_500_000  # Includes watchdog/cleanup margin in 45-second envelope.
            timer = threading.Timer(45, cancel); timer.daemon = True; timer.start()
            threading.Thread(target=watch_control, daemon=True).start()
            emit("armed", at=started, latest_end=started + 45_000_000)
            outcome = execute(args.case, backend, radio, stop, started)
        except BaseException as exc:
            failure = type(exc).__name__ + ": " + str(exc)
        finally:
            if timer:
                timer.cancel()
            try:
                if radio:
                    cleanup = asdict(radio.shutdown())
                    require(cleanup["safe_shutdown"] is True and not cleanup["episodes"], "unsafe peer cleanup")
                else:
                    backend.safe_standby(backend.deadline(500_000))
                    cleanup = dict(safe_shutdown=True)
            except BaseException as exc:
                cleanup = dict(safe_shutdown=False, error=type(exc).__name__ + ": " + str(exc))
                failure = failure or "peer cleanup failed"
            finally:
                if radio is None:
                    try:
                        backend.close()
                    except BaseException as exc:
                        cleanup = dict(safe_shutdown=False, error=str(exc))
                        failure = failure or "peer handle release failed"
            emit("complete", failure=failure, outcome=outcome, cleanup=cleanup,
                 attempts=io.attempts, trace=io.trace, at=clock.now_monotonic_us())
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
