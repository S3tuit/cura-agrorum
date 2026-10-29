"""Read-only evidence; neither acquisition nor interpretation authorizes recovery."""

from collections import deque
from dataclasses import replace
import json

from .generated import receiver_enums_generated as E
from .generated.receiver_entities_generated import RadioSnapshotV1
from .ports.radio import Error, RadioBackendError, RadioFailure, Stage


class RadioHistory:
    """Pilot-only bounded command/edge context, copied by incident snapshots.

    GPIO callbacks observe the existing consumer, including stale-edge draining.
    They never read the event queue themselves. Sequence gaps and ring eviction
    are distinct: a gap is a history entry; dropped counts evicted entries.
    Command entries record transfer attempts, including observation reads; they
    do not establish that a command reached the chip or completed successfully.
    """

    def __init__(self, clock):
        self.clock = clock
        self.entries = deque(maxlen=32)
        self.sequence = self.dropped = self.last_edge_sequence = 0

    def record(self, kind, value, detail=0):
        self.sequence += 1
        if len(self.entries) == self.entries.maxlen:
            self.dropped += 1
        self.entries.append((self.sequence, self.clock.now_monotonic_us(), kind, value, detail))

    def edge(self, timestamp_ns, sequence):
        if (type(timestamp_ns) is not int or type(sequence) is not int
                or not 0 <= timestamp_ns < 1 << 63 or not 0 < sequence < 1 << 63):
            self.record('invalid_edge', 0)
            return
        if sequence > self.last_edge_sequence + 1:
            self.record('edge_gap', sequence - self.last_edge_sequence - 1)
        self.last_edge_sequence = sequence
        self.record('edge', sequence, timestamp_ns)

    def snapshot(self):
        return json.dumps(tuple(self.entries), separators=(',', ':')).encode('ascii')


def capture_snapshot(backend, software_state, trigger, *, edge=None, event=None,
                     started=None, deadline=None):
    """Probe once without changing backend command ownership/cached TX facts.

    An event's original IRQ/status outranks another SPI read. Health reads use
    the physical port directly: ordinary command helpers mutate command-status
    ownership and BUSY counters. BUSY is sampled, never polled for telemetry.
    GPIO reads bracket SPI so a packet completing during sampling stays visible.
    """
    backend._claim()
    now = backend.clock.now_monotonic_us()
    start = now if started is None else started
    deadline = backend.deadline(50_000, deadline)
    values = dict(chip_status=None, chip_mode=None, irq_status=None, device_errors=None,
                  dio1_before=None, dio1_after=None, busy=None, gpio_event_pending=None)
    failure = event if isinstance(event, RadioFailure) else None
    # Keep context and loss at the same pre-probe boundary. This capture's
    # command attempts belong to subsequent snapshots, even when a read fails.
    history = backend.history.snapshot()
    history_dropped = backend.history.dropped

    def attempt(function):
        nonlocal failure
        try:
            backend._guard(deadline, Stage.READ_COMMAND)
            result = function()
            backend._guard(deadline, Stage.READ_COMMAND, after=True)
            return result
        except RadioBackendError as error:
            if failure is None:
                failure = error.failure
            return None

    def read(command, stage):
        # Do not issue SPI while BUSY is asserted, even if an earlier pin sample
        # was low. This preserves the reason for skipping reads in the snapshot.
        busy = backend.io.busy()
        if type(busy) is not bool:
            raise RadioBackendError(RadioFailure(Error.MALFORMED_RESPONSE, Stage.WAIT_BUSY))
        values['busy'] = busy
        if busy:
            return None
        try:
            backend.history.record('command', command[0])
            result = backend.io.transfer(command, deadline_monotonic_us=deadline)
        except RadioBackendError as error:
            raise RadioBackendError(replace(error.failure, opcode=command[0], stage=stage)) from error
        if type(result) is not bytes or len(result) != len(command):
            raise RadioBackendError(RadioFailure(Error.MALFORMED_RESPONSE, stage, opcode=command[0]))
        return result

    def pin(function, stage):
        result = function()
        if type(result) is not bool:
            raise RadioBackendError(RadioFailure(Error.MALFORMED_RESPONSE, stage))
        return result

    # These are pre-mutation facts from the original event, not last-event caches.
    if event is not None:
        values.update(chip_status=event.chip_status, irq_status=event.irq_status,
                      device_errors=event.device_errors)
    if backend._open:
        values['busy'] = attempt(lambda: pin(backend.io.busy, Stage.WAIT_BUSY))
        pending_before = attempt(lambda: pin(backend.io.edge_pending, Stage.WAIT_IRQ))
        values['dio1_before'] = attempt(lambda: pin(backend.io.dio1, Stage.READ_IRQ))
        if event is None and values['busy'] is False:
            status = attempt(lambda: read(b'\xc0\x00', Stage.READ_COMMAND))
            if status is not None:
                values['chip_status'] = status[1]
            irq = attempt(lambda: read(b'\x12\x00\x00\x00', Stage.READ_IRQ))
            if irq is not None:
                values['irq_status'] = int.from_bytes(irq[2:], 'big')
            errors = attempt(lambda: read(b'\x17\x00\x00\x00', Stage.READ_COMMAND))
            if errors is not None:
                values['device_errors'] = int.from_bytes(errors[2:], 'big')
        values['dio1_after'] = attempt(lambda: pin(backend.io.dio1, Stage.READ_IRQ))
        pending_after = attempt(lambda: pin(backend.io.edge_pending, Stage.WAIT_IRQ))
        values['gpio_event_pending'] = (True if pending_before or pending_after else
                                        False if pending_before is False and pending_after is False else None)
    status = values['chip_status']
    if status is not None:
        try:
            if status & 0x81 or (status >> 1) & 7 == 7:
                raise ValueError('malformed status')
            values['chip_mode'] = E.RadioChipMode((status >> 4) & 7)
        except ValueError:
            if failure is None:
                failure = RadioFailure(Error.MALFORMED_RESPONSE, Stage.READ_COMMAND, opcode=0xC0,
                                       chip_status=status)
    complete = all(value is not None for value in values.values()) and failure is None
    captured = (E.RadioCaptureStatus.COMPLETE if complete else E.RadioCaptureStatus.PARTIAL
                if backend._open or event is not None else E.RadioCaptureStatus.UNAVAILABLE)
    return RadioSnapshotV1(
        trigger=trigger, software_state=software_state,
        capture_started_at_monotonic_us=start,
        capture_finished_at_monotonic_us=backend.clock.now_monotonic_us(),
        capture_status=captured, **values,
        event_timestamp_ns=None if edge is None else edge.timestamp_ns,
        event_sequence=None if edge is None else edge.sequence,
        last_set_rx_issued_monotonic_us=backend.last_set_rx_issued_us,
        failure_code=None if failure is None else failure.code.value,
        failure_stage=None if failure is None else failure.stage.value,
        failure_opcode=None if failure is None else failure.opcode,
        os_errno=None if failure is None else failure.os_errno,
        history=history, history_dropped=history_dropped,
    )
