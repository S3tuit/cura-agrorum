"""SX1262 commands and complete PHY profiles on the normalized physical port.

Command/register encodings: Semtech DS.SX1261-2.W.APP Rev 2.2, chapters 8,
12, 13 and 15. PHY values: protocol/protocol-v2-lora/README.md. This layer
confirms effects and reports evidence; it neither retries TX nor owns recovery.
"""

from dataclasses import dataclass, replace
import threading

from .elapsed_duration import (
    checked_monotonic_deadline, checked_monotonic_elapsed,
    maximum_lifetime_monotonic_us, minimum_wait_monotonic_us,
)
from .ports.radio import (
    BusyMetrics, Error, Outcome, RadioBackendError, RadioConfiguration,
    RadioFailure, RadioIo, RadioWait, Stage, integer,
)

IRQ_TX_DONE = 0x0001
IRQ_RX_DONE = 0x0002
IRQ_HEADER_ERROR = 0x0020
IRQ_CRC_ERROR = 0x0040
IRQ_TIMEOUT = 0x0200
IRQ_MASK = 0x0263


@dataclass(frozen=True, slots=True)
class RadioEventObservation:
    irq_status: int
    chip_status: int
    device_errors: int

    def __post_init__(self):
        integer(self.irq_status, 0, 65535)
        integer(self.chip_status, 0, 255)
        integer(self.device_errors, 0, 65535)


class Sx1262:
    def __init__(self, io: RadioIo, clock, waiter: RadioWait, configuration=None):
        self.io = io
        self.clock = clock
        self.waiter = waiter
        self.configuration = configuration or RadioConfiguration()
        self.profile = None
        self.metrics = BusyMetrics()
        self.last_set_rx_issued_us = None
        self.last_set_tx_issued_us = None
        self.set_tx_outcome = Outcome.NOT_APPLICABLE
        self.last_chip_status = None
        self.initial_reset_status = None
        self.last_irq = None
        self.last_device_errors = None
        self._owner = None
        self._open = False
        self.checkpoint = None
        self.command_uncertain = False
        self._pending_edge = None

    def _checkpoint(self):
        if self.checkpoint is not None:
            self.checkpoint()

    def _claim(self):
        current = threading.get_ident()
        if self._owner is None:
            self._owner = current
        elif current != self._owner:
            raise RuntimeError("SX1262 belongs to another thread")

    def deadline(self, duration_us, enclosing=None):
        bound = checked_monotonic_deadline(
            self.clock.now_monotonic_us(), maximum_lifetime_monotonic_us(duration_us)
        )
        return bound if enclosing is None else min(bound, integer(enclosing))

    def reset_metrics(self):
        self._claim()
        self.metrics = BusyMetrics()

    def _error(self, code, stage, *, opcode=0, outcome=Outcome.NOT_APPLICABLE, **facts):
        raise RadioBackendError(RadioFailure(code, stage, outcome, opcode, **facts))

    def _guard(self, deadline, stage, opcode=0, *, after=False):
        now = self.clock.now_monotonic_us()
        integer(deadline)
        if now > deadline or (now == deadline and not after):
            self._error(
                Error.DEADLINE, stage, opcode=opcode,
                outcome=Outcome.UNCERTAIN if after else Outcome.DEFINITELY_NOT_APPLIED,
                hardware_touched=after,
            )

    def open(self, deadline):
        self._claim()
        self.io.open(self.configuration, deadline_monotonic_us=deadline)
        self._open = True

    def wait_busy(self, deadline, *, opcode=0, after=False):
        self._claim()
        start = self.clock.now_monotonic_us()
        bound = self.deadline(100_000, deadline)
        timeout = False
        try:
            while True:
                self._checkpoint()
                try:
                    busy = self.io.busy()
                except RadioBackendError as error:
                    raise RadioBackendError(replace(
                        error.failure, opcode=opcode, stage=Stage.WAIT_BUSY,
                        outcome=Outcome.UNCERTAIN if after else Outcome.DEFINITELY_NOT_APPLIED,
                    )) from error
                now = self.clock.now_monotonic_us()
                if not busy and now <= bound:
                    return
                if now >= bound:
                    timeout = True
                    self._error(
                        Error.BUSY_TIMEOUT, Stage.WAIT_BUSY, opcode=opcode,
                        outcome=Outcome.UNCERTAIN if after else Outcome.DEFINITELY_NOT_APPLIED,
                        hardware_touched=after,
                    )
                self.waiter.wait_until_monotonic_us(min(bound, checked_monotonic_deadline(now, 50)))
        finally:
            elapsed = checked_monotonic_elapsed(start, self.clock.now_monotonic_us())
            previous = self.metrics
            self.metrics = BusyMetrics(
                previous.total_us + elapsed, max(previous.maximum_us, elapsed),
                previous.count + 1, previous.timeout_count + int(timeout),
                opcode if timeout else previous.last_timeout_opcode,
            )

    def reset(self, deadline):
        self._claim()
        self.profile = None
        self.initial_reset_status = None
        self._pending_edge = None
        self._guard(deadline, Stage.RESET)
        release_at = checked_monotonic_deadline(
            self.clock.now_monotonic_us(), minimum_wait_monotonic_us(1000)
        )
        if release_at >= deadline:
            self._error(Error.DEADLINE, Stage.RESET, outcome=Outcome.DEFINITELY_NOT_APPLIED)
        self.io.set_reset(asserted=True)
        try:
            # GPIO/scheduler latency before assertion is not part of the pulse.
            release_at = checked_monotonic_deadline(
                self.clock.now_monotonic_us(), minimum_wait_monotonic_us(1000)
            )
            self.waiter.wait_until_monotonic_us(min(release_at, deadline))
        finally:
            self.io.set_reset(asserted=False)
        if self.clock.now_monotonic_us() >= deadline:
            self._error(
                Error.DEADLINE, Stage.RESET,
                outcome=Outcome.UNCERTAIN, hardware_touched=True,
            )
        self.wait_busy(deadline, after=True)
        status = self._status(deadline, allow_unresponsive=True)
        self.initial_reset_status = status
        if status in (0, 255):
            status = self._status(deadline, allow_unresponsive=True)
            if status in (0, 255):
                self._error(
                    Error.IO, Stage.RESET, hardware_touched=True,
                    hardware_missing=True, chip_status=status,
                )
        if (status >> 4) & 7 != 2:
            self._error(Error.COMMAND_STATUS, Stage.RESET, chip_status=status, hardware_touched=True)
        # The initial status predates host commands. Establish a fresh command
        # result before inspecting the documented TCXO power-on error.
        self.standby(deadline)
        errors = self.read_device_errors(deadline)
        if errors & ~0x0020:
            self._error(Error.DEVICE_ERROR, Stage.RESET, opcode=0x17,
                        device_errors=errors, hardware_touched=True)

    def _raw(self, data, deadline, stage):
        self._claim()
        self._checkpoint()
        self._guard(deadline, stage, data[0])
        self.wait_busy(deadline, opcode=data[0])
        self._guard(deadline, stage, data[0])
        issued = self.clock.now_monotonic_us()
        if data[0] == 0x83:
            self.last_set_tx_issued_us = issued
            self.set_tx_outcome = Outcome.UNCERTAIN
            deadline = min(deadline, checked_monotonic_deadline(issued, maximum_lifetime_monotonic_us(250_000)))
        elif data[0] == 0x82:
            self.last_set_rx_issued_us = issued
        try:
            if data[0] != 0xC0:
                self.command_uncertain = True
            result = self.io.transfer(data, deadline_monotonic_us=deadline)
        except RadioBackendError as error:
            if error.failure.outcome is Outcome.DEFINITELY_NOT_APPLIED:
                self.command_uncertain = False
            raise RadioBackendError(replace(error.failure, stage=stage, opcode=data[0])) from error
        self._guard(deadline, stage, data[0], after=True)
        if type(result) is not bytes or len(result) != len(data):
            self._error(Error.MALFORMED_RESPONSE, stage, opcode=data[0],
                        outcome=Outcome.UNCERTAIN, hardware_touched=True)
        self.wait_busy(deadline, opcode=data[0], after=True)
        return result

    def _status(self, deadline, *, allow_unresponsive=False):
        """Read and validate structure; command ownership belongs to the caller."""
        status = self._raw(b"\xc0\x00", deadline, Stage.READ_COMMAND)[1]
        self.last_chip_status = status
        if allow_unresponsive and status in (0, 255):
            return status
        mode, command = (status >> 4) & 7, (status >> 1) & 7
        if status & 0x81 or mode not in (2, 3, 4, 5, 6) or command == 7:
            self._error(Error.MALFORMED_RESPONSE, Stage.READ_COMMAND, opcode=0xC0,
                        chip_status=status, hardware_touched=True)
        return status

    def _confirm_status(self, deadline):
        status = self._status(deadline)
        if (status >> 1) & 7 in (3, 4, 5):
            self._error(Error.COMMAND_STATUS, Stage.READ_COMMAND, opcode=0xC0,
                        chip_status=status, hardware_touched=True)
        return status

    def wait_edge(self, *, deadline_monotonic_us):
        self._claim()
        if self._pending_edge is not None:
            edge, self._pending_edge = self._pending_edge, None
            return edge
        return self.io.wait_edge(deadline_monotonic_us=deadline_monotonic_us)

    def observe_event(self, deadline, *, chip_status=None):
        """Capture bounded read-only evidence before attributing command bits."""
        irq = int.from_bytes(self._raw(b"\x12\x00\x00\x00", deadline, Stage.READ_IRQ)[2:], "big")
        self.last_irq = irq
        errors = int.from_bytes(self._raw(b"\x17\x00\x00\x00", deadline, Stage.READ_COMMAND)[2:], "big")
        self.last_device_errors = errors
        status = self._status(deadline) if chip_status is None else chip_status
        return RadioEventObservation(irq, status, errors)

    def validate_event(self, event, *, transmit):
        """The owner checks edge chronology; this checks matching device facts."""
        self._claim()
        irq, status, errors = event.irq_status, event.chip_status, event.device_errors
        mode, command = (status >> 4) & 7, (status >> 1) & 7
        facts = dict(chip_status=status, irq_status=irq, device_errors=errors, hardware_touched=True)
        if (command in (4, 5)
                or command == 3 and not (irq == IRQ_TIMEOUT and mode == 2)
                or command == 6 and not (transmit and irq == IRQ_TX_DONE)):
            self._error(Error.COMMAND_STATUS, Stage.READ_COMMAND, opcode=0xC0, **facts)
        if errors:
            self._error(Error.DEVICE_ERROR, Stage.READ_COMMAND, opcode=0x17, **facts)
        terminal = irq in (IRQ_TX_DONE, IRQ_TIMEOUT) if transmit else (
            irq == IRQ_TIMEOUT or bool(irq) and not irq & ~(IRQ_RX_DONE | IRQ_HEADER_ERROR | IRQ_CRC_ERROR)
        )
        if terminal and mode != 2:
            self._error(Error.COMMAND_STATUS, Stage.READ_COMMAND, opcode=0xC0, **facts)
        return terminal

    def _start_operation(self, data, deadline, *, transmit):
        """Confirm active mode, or retain a fresh edge proving immediate completion."""
        self._raw(data, deadline, Stage.WRITE_COMMAND)
        issued = self.last_set_tx_issued_us if transmit else self.last_set_rx_issued_us
        if transmit:
            deadline = min(deadline, checked_monotonic_deadline(issued, maximum_lifetime_monotonic_us(250_000)))
        try:
            status = self._status(deadline)
            mode, command = (status >> 4) & 7, (status >> 1) & 7
            if command in (4, 5):
                self._error(Error.COMMAND_STATUS, Stage.READ_COMMAND, opcode=0xC0,
                            chip_status=status, hardware_touched=True)
            if mode == (6 if transmit else 5) and command != 3:
                self.command_uncertain = False
                return
            event = self.observe_event(deadline, chip_status=status)
            if not self.validate_event(event, transmit=transmit):
                self._error(Error.COMMAND_STATUS, Stage.WRITE_COMMAND, opcode=data[0],
                            chip_status=status, irq_status=event.irq_status,
                            device_errors=event.device_errors, hardware_touched=True)
            edge = self.wait_edge(deadline_monotonic_us=deadline)
            self._checkpoint()
            if edge is None or edge.timestamp_ns > deadline * 1000:
                self._error(Error.DEADLINE, Stage.WAIT_IRQ, opcode=data[0], hardware_touched=True)
            if edge.timestamp_ns < issued * 1000:
                self._error(Error.UNEXPECTED_IRQ, Stage.WAIT_IRQ, opcode=data[0],
                            irq_status=event.irq_status, hardware_touched=True)
            if edge.timestamp_ns > self.clock.now_monotonic_us() * 1000 + 999:
                self._error(Error.MALFORMED_RESPONSE, Stage.CAPTURE_TIME,
                            opcode=data[0], hardware_touched=True)
            self._pending_edge = edge
            self.command_uncertain = False
        except RadioBackendError as error:
            raise RadioBackendError(replace(error.failure, outcome=Outcome.UNCERTAIN)) from error

    def _command(self, data, deadline, stage=Stage.WRITE_COMMAND):
        self._claim()
        result = self._raw(data, deadline, stage)
        try:
            status = self._confirm_status(deadline)
        except RadioBackendError as error:
            # A failed confirmation never proves the preceding effect absent.
            raise RadioBackendError(replace(error.failure, outcome=Outcome.UNCERTAIN)) from error
        self.command_uncertain = False
        return result, status

    def _read(self, opcode, size, deadline, stage=Stage.READ_COMMAND):
        result, _ = self._command(bytes((opcode, 0)) + bytes(size), deadline, stage)
        return result[2:]

    def read_register(self, address, size, deadline):
        integer(address, 0, 65535)
        integer(size, 1, 255)
        data = b"\x1d" + address.to_bytes(2, "big") + bytes(size + 1)
        return self._command(data, deadline, Stage.READ_COMMAND)[0][4:]

    def write_register(self, address, value, deadline):
        integer(address, 0, 65535)
        if type(value) is not bytes or not 1 <= len(value) <= 255:
            raise ValueError("invalid register bytes")
        self._command(b"\x0d" + address.to_bytes(2, "big") + value, deadline)

    def _bits(self, address, mask, value, deadline):
        old = self.read_register(address, 1, deadline)[0]
        self.write_register(address, bytes(((old & ~mask) | value,)), deadline)

    def standby(self, deadline):
        self._claim()
        self.profile = None
        _, status = self._command(b"\x80\x00", deadline)
        if (status >> 4) & 7 != 2:
            self._error(Error.COMMAND_STATUS, Stage.WRITE_COMMAND, opcode=0x80,
                        outcome=Outcome.UNCERTAIN, chip_status=status, hardware_touched=True)

    def read_irq(self, deadline):
        value = int.from_bytes(self._read(0x12, 2, deadline, Stage.READ_IRQ), "big")
        self.last_irq = value
        return value

    def clear_irq(self, mask, deadline):
        integer(mask, 0, 65535)
        self._command(b"\x02" + mask.to_bytes(2, "big"), deadline, Stage.CLEAR_IRQ)

    def read_device_errors(self, deadline):
        value = int.from_bytes(self._read(0x17, 2, deadline), "big")
        self.last_device_errors = value
        return value

    def check_device_errors(self, deadline):
        value = self.read_device_errors(deadline)
        if value:
            self._error(Error.DEVICE_ERROR, Stage.READ_COMMAND, opcode=0x17,
                        device_errors=value, hardware_touched=True)

    def account_stale_irqs(self, deadline, *, resynchronize=False):
        """Only called in confirmed standby, before a new RX/TX command."""
        self.read_irq(deadline)
        self.clear_irq(0xFFFF, deadline)
        if resynchronize:
            self._guard(deadline, Stage.CLEAR_IRQ)
            high = self.io.dio1()
            self._guard(deadline, Stage.CLEAR_IRQ, after=True)
            if high:
                self._error(Error.UNEXPECTED_IRQ, Stage.CLEAR_IRQ, irq_status=self.last_irq,
                            hardware_touched=True)
            self._pending_edge = None
            self._checkpoint()
            self._guard(deadline, Stage.WAIT_IRQ)
            self.io.resynchronize_events(deadline_monotonic_us=deadline)
            self._guard(deadline, Stage.WAIT_IRQ, after=True)
            self._checkpoint()
        else:
            for _ in range(64):
                if self.wait_edge(deadline_monotonic_us=self.clock.now_monotonic_us()) is None:
                    break
            else:
                self._error(Error.UNEXPECTED_IRQ, Stage.WAIT_IRQ, hardware_touched=True)
        self._guard(deadline, Stage.CLEAR_IRQ)
        high = self.io.dio1()
        self._guard(deadline, Stage.CLEAR_IRQ, after=True)
        if high:
            self._error(Error.UNEXPECTED_IRQ, Stage.CLEAR_IRQ, irq_status=self.last_irq,
                        hardware_touched=True)

    def initialize(self, deadline):
        self.reset(deadline)
        self._command(b"\x96\x01", deadline)  # DCDC, populated module inductor.
        self._bits(0x08D8, 0x1E, 0x1E, deadline)  # PA clamp workaround.
        self._command(b"\x97\x01\x00\x01\x40", deadline)  # 1.7 V, 320 ticks = 5 ms.
        self._command(b"\x07\x00\x00", deadline)
        self._command(b"\x89\x7f", deadline)
        self._command(b"\x98\xd7\xdb", deadline)  # 863..870 MHz image calibration.
        self._command(b"\x9d\x01", deadline)  # DIO2 RF switch.
        self._command(b"\x95\x04\x07\x00\x01", deadline)  # SX1262 standard PA.
        self._command(b"\x8f\x00\x00", deadline)
        self.check_device_errors(deadline)
        self.install_profile(transmit=False, payload_length=255, deadline=deadline)

    def install_profile(self, *, transmit, payload_length, deadline):
        self._claim()
        if type(transmit) is not bool:
            raise TypeError("profile direction must be Boolean")
        integer(payload_length, 0, 255)
        self.profile = None
        self.standby(deadline)
        for command in (
            b"\x8a\x01",                         # LoRa
            b"\x86\x36\x41\x99\x9a",         # 868.1 MHz, nearest PLL step
            b"\x8b\x07\x04\x01\x00",         # SF7/BW125/CR4/5, LDRO off
            b"\x8c\x00\x08\x00" + bytes((payload_length, 1, int(transmit))),
            b"\x8e\x0e\x02",                   # +14 dBm / 40 us ramp
            b"\x93\x20",                         # STDBY_RC fallback
            b"\x9f\x00",                         # Do not stop RX timer at preamble
            b"\xa0\x00",                         # No symbol-count timeout
            b"\x08\x02\x63\x02\x63\x00\x00\x00\x00",
        ):
            self._command(command, deadline, Stage.CONFIGURE_IRQ if command[0] == 8 else Stage.WRITE_COMMAND)
        self.write_register(0x0740, b"\x14\x24", deadline)
        self.write_register(0x08AC, b"\x96", deadline)  # Boosted RX in complete profile.
        self._bits(0x0889, 0x04, 0x04, deadline)  # BW125 modulation workaround.
        self._bits(0x0736, 0x04, 0 if transmit else 0x04, deadline)
        self.check_device_errors(deadline)
        self.profile = "tx" if transmit else "rx"

    def arm_receive(self, deadline, *, timeout_ticks=0, resynchronize=False):
        self._claim()
        if self.profile != "rx":
            raise RuntimeError("SetRx requires a complete confirmed RX profile")
        integer(timeout_ticks, 0, 0xFFFFFE)
        self.last_set_rx_issued_us = None
        self.account_stale_irqs(deadline, resynchronize=resynchronize)
        self._start_operation(b"\x82" + timeout_ticks.to_bytes(3, "big"), deadline, transmit=False)

    def write_buffer(self, frame, deadline):
        if type(frame) is not bytes or not 1 <= len(frame) <= 255:
            raise ValueError("radio TX payload must be 1..255 immutable bytes")
        self._command(b"\x0e\x00" + frame, deadline, Stage.WRITE_BUFFER)

    def start_tx(self, deadline):
        self._claim()
        if self.profile != "tx":
            raise RuntimeError("SetTx requires a complete confirmed TX profile")
        self.last_set_tx_issued_us = None
        self.set_tx_outcome = Outcome.DEFINITELY_NOT_APPLIED
        self.account_stale_irqs(deadline)
        try:
            self._start_operation(b"\x83\x00\x19\x00", deadline, transmit=True)
        except RadioBackendError as error:
            if self.last_set_tx_issued_us is not None:
                self.set_tx_outcome = error.failure.outcome
            raise
        self.set_tx_outcome = Outcome.CONFIRMED_APPLIED

    def copy_packet(self, deadline):
        length, offset = self._read(0x13, 2, deadline, Stage.READ_BUFFER)
        raw = self._raw(b"\x1e" + bytes((offset, 0)) + bytes(length), deadline, Stage.READ_BUFFER)
        frame = bytes(raw[3:])
        copied_at = self.clock.now_monotonic_us()
        return frame, copied_at

    def read_packet_status(self, deadline):
        self._confirm_status(deadline)
        status = self._read(0x14, 3, deadline, Stage.READ_PACKET_STATUS)
        rssi = -status[0]
        snr = status[1] if status[1] < 128 else status[1] - 256
        return rssi, snr

    def finish_receive(self, deadline):
        self.write_register(0x0902, b"\x00", deadline)
        self._bits(0x0944, 0x02, 0x02, deadline)  # Semtech RX-done RTC workaround.

    def read_packet(self, deadline):
        """Convenience for component peers that require only complete reads."""
        frame, copied_at = self.copy_packet(deadline)
        rssi, snr = self.read_packet_status(deadline)
        self.finish_receive(deadline)
        return frame, rssi, snr, copied_at

    def soft_restore(self, deadline):
        self.wait_busy(deadline)
        self.standby(deadline)
        self.read_irq(deadline)
        errors = self.read_device_errors(deadline)
        if errors:
            self._command(b"\x07\x00\x00", deadline)
        self.clear_irq(0xFFFF, deadline)
        self.install_profile(transmit=False, payload_length=255, deadline=deadline)
        self.arm_receive(deadline, resynchronize=True)

    def safe_standby(self, deadline):
        self.standby(deadline)
        self._command(b"\x08" + bytes(8), deadline, Stage.CONFIGURE_IRQ)
        self.account_stale_irqs(deadline)
        if (self._confirm_status(deadline) >> 4) & 7 != 2:
            self._error(Error.COMMAND_STATUS, Stage.READ_COMMAND, opcode=0xC0, hardware_touched=True)

    def close(self):
        self._claim()
        self._open = False
        self.io.close()
