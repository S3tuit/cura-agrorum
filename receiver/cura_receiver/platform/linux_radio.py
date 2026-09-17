"""Single-owner libgpiod-v2/spidev implementation of the production radio port."""

from datetime import timedelta
import errno
import importlib
import threading

from ..ports.clocks import MonotonicClock
from ..ports.radio import (
    Dio1Edge, Error, Outcome, RadioBackendError, RadioConfiguration,
    RadioFailure, RadioLifecycleError, RadioLifecycleFailure, Stage, integer,
)

_MISSING = {errno.ENOENT, errno.ENODEV, errno.ENXIO, errno.ESHUTDOWN}
_EVENT_BUFFER_SIZE = 64


class LinuxRadioIo:
    def __init__(self, clock: MonotonicClock):
        self.clock = clock
        self._owner = None
        self._request = None
        self._spi = None
        self._configuration = None
        self._gpiod = None
        self._last_sequence = 0
        self._last_timestamp_ns = 0
        self._observed_edge = None
        self._observed_at_us = None
        self._events_need_sync = False
        self._events_untrusted = False
        self._used = False

    def _claim(self):
        current = threading.get_ident()
        if self._owner is None:
            self._owner = current
        elif self._owner != current:
            raise RuntimeError("radio I/O belongs to another thread")

    @staticmethod
    def _os_call(stage, call, *, outcome=Outcome.NOT_APPLICABLE):
        try:
            return call()
        except OSError as error:
            raise RadioBackendError(RadioFailure(
                Error.IO, stage, outcome,
                os_errno=error.errno,
                hardware_touched=True,
                hardware_missing=error.errno in _MISSING,
            )) from error

    def _deadline(self, deadline, stage, *, after=False):
        integer(deadline)
        now = self.clock.now_monotonic_us()
        if now > deadline or (now == deadline and not after):
            raise RadioBackendError(RadioFailure(
                Error.DEADLINE, stage,
                Outcome.UNCERTAIN if after else Outcome.DEFINITELY_NOT_APPLIED,
                hardware_touched=after,
            ))

    def open(self, configuration, *, deadline_monotonic_us):
        self._claim()
        if type(configuration) is not RadioConfiguration:
            raise TypeError("expected RadioConfiguration")
        if self._used:
            raise RuntimeError("radio I/O resources cannot be reopened")
        self._deadline(deadline_monotonic_us, Stage.CONFIGURE_GPIO)
        self._used = True
        self._configuration = configuration
        # Missing Python dependencies are deployment errors, not fabricated errno.
        self._gpiod = importlib.import_module("gpiod")
        spidev = importlib.import_module("spidev")
        line = self._gpiod.line
        settings = self._gpiod.LineSettings
        try:
            self._request = self._os_call(Stage.CONFIGURE_GPIO, lambda: self._gpiod.request_lines(
                configuration.gpio_chip,
                consumer="cura-radio",
                event_buffer_size=_EVENT_BUFFER_SIZE,
                config={
                    configuration.reset_line: settings(
                        direction=line.Direction.OUTPUT,
                        drive=line.Drive.OPEN_DRAIN,
                        output_value=line.Value.ACTIVE,
                        bias=line.Bias.AS_IS,
                    ),
                    configuration.busy_line: settings(
                        direction=line.Direction.INPUT, bias=line.Bias.AS_IS,
                    ),
                    configuration.dio1_line: settings(
                        direction=line.Direction.INPUT,
                        edge_detection=line.Edge.RISING,
                        event_clock=line.Clock.MONOTONIC,
                        bias=line.Bias.AS_IS,
                    ),
                },
            ))
            self._deadline(deadline_monotonic_us, Stage.CONFIGURE_SPI)
            self._spi = self._os_call(Stage.CONFIGURE_SPI, spidev.SpiDev)
            self._deadline(deadline_monotonic_us, Stage.CONFIGURE_SPI)
            self._os_call(Stage.CONFIGURE_SPI, lambda: self._spi.open_path(configuration.spi_device))
            for name, value in (
                ("mode", 0), ("max_speed_hz", configuration.spi_speed_hz),
                ("bits_per_word", 8), ("lsbfirst", False), ("no_cs", False),
                ("cshigh", False), ("threewire", False), ("loop", False),
            ):
                self._deadline(deadline_monotonic_us, Stage.CONFIGURE_SPI)
                self._os_call(Stage.CONFIGURE_SPI, lambda n=name, v=value: setattr(self._spi, n, v))
                self._deadline(deadline_monotonic_us, Stage.CONFIGURE_SPI, after=True)
        except BaseException as error:
            self._raise_lifecycle(error, self._release())

    def _ready(self):
        self._claim()
        if self._request is None or self._spi is None:
            raise RuntimeError("radio I/O is not open")

    def transfer(self, data, *, deadline_monotonic_us):
        self._ready()
        if type(data) is not bytes or not 1 <= len(data) <= 260:
            raise ValueError("SPI transaction must be 1..260 immutable bytes")
        self._deadline(deadline_monotonic_us, Stage.WRITE_COMMAND)
        received = self._os_call(
            Stage.WRITE_COMMAND,
            lambda: self._spi.xfer2(list(data)),
            outcome=Outcome.UNCERTAIN,
        )
        self._deadline(deadline_monotonic_us, Stage.WRITE_COMMAND, after=True)
        if not isinstance(received, (list, bytes, bytearray)) or len(received) != len(data):
            raise RadioBackendError(RadioFailure(
                Error.MALFORMED_RESPONSE, Stage.READ_COMMAND,
                Outcome.UNCERTAIN, hardware_touched=True,
            ))
        if any(type(value) is not int or not 0 <= value <= 255 for value in received):
            raise RadioBackendError(RadioFailure(
                Error.MALFORMED_RESPONSE, Stage.READ_COMMAND,
                Outcome.UNCERTAIN, hardware_touched=True,
            ))
        return bytes(received)

    def _value(self, offset, stage):
        self._ready()
        value = self._os_call(stage, lambda: self._request.get_value(offset))
        if value is self._gpiod.line.Value.ACTIVE:
            return True
        if value is self._gpiod.line.Value.INACTIVE:
            return False
        raise RadioBackendError(RadioFailure(Error.MALFORMED_RESPONSE, stage, hardware_touched=True))

    def busy(self):
        self._ready()
        return self._value(self._configuration.busy_line, Stage.WAIT_BUSY)

    def dio1(self):
        self._ready()
        return self._value(self._configuration.dio1_line, Stage.READ_IRQ)

    def set_reset(self, *, asserted):
        self._ready()
        if type(asserted) is not bool:
            raise TypeError("reset level must be Boolean")
        value = self._gpiod.line.Value.INACTIVE if asserted else self._gpiod.line.Value.ACTIVE
        self._os_call(
            Stage.RESET, lambda: self._request.set_value(self._configuration.reset_line, value),
            outcome=Outcome.UNCERTAIN,
        )

    def wait_edge(self, *, deadline_monotonic_us):
        self._ready()
        integer(deadline_monotonic_us)
        if self._events_need_sync:
            self._event_error()
        while True:
            remaining = max(0, deadline_monotonic_us - self.clock.now_monotonic_us())
            available = self._events_available(timedelta(microseconds=min(remaining, 1_000_000)))
            if available:
                edge = self._consume_edge()
                if edge.sequence != self._last_sequence + 1:
                    self._event_error()
                self._last_sequence = edge.sequence
                self._last_timestamp_ns = edge.timestamp_ns
                return edge
            if self.clock.now_monotonic_us() >= deadline_monotonic_us:
                return None

    def _event_error(self, *, untrusted=False, code=Error.MALFORMED_RESPONSE):
        self._events_need_sync = True
        self._events_untrusted |= untrusted
        raise RadioBackendError(RadioFailure(code, Stage.WAIT_IRQ, hardware_touched=True))

    def _events_available(self, timeout):
        try:
            return self._os_call(Stage.WAIT_IRQ, lambda: self._request.wait_edge_events(timeout=timeout))
        except RadioBackendError:
            self._events_need_sync = True
            raise

    def _consume_edge(self):
        try:
            events = self._os_call(Stage.WAIT_IRQ, lambda: self._request.read_edge_events(max_events=1))
        except RadioBackendError:
            self._events_need_sync = self._events_untrusted = True
            raise
        captured_at_us = self.clock.now_monotonic_us()
        if not isinstance(events, (list, tuple)) or len(events) != 1:
            self._event_error(untrusted=True)
        event = events[0]
        sequence = getattr(event, "line_seqno", None)
        timestamp = getattr(event, "timestamp_ns", None)
        previous_sequence = self._observed_edge.sequence if self._observed_edge else self._last_sequence
        previous_time = self._observed_edge.timestamp_ns if self._observed_edge else self._last_timestamp_ns
        if (
            getattr(event, "event_type", None) is not self._gpiod.EdgeEvent.Type.RISING_EDGE
            or type(getattr(event, "line_offset", None)) is not int
            or event.line_offset != self._configuration.dio1_line
            or type(sequence) is not int or not previous_sequence < sequence <= (1 << 64) - 1
            or type(timestamp) is not int or not previous_time <= timestamp <= (1 << 64) - 1
        ):
            self._event_error(untrusted=True)
        self._observed_edge = Dio1Edge(timestamp, sequence)
        self._observed_at_us = captured_at_us
        return self._observed_edge

    def resynchronize_events(self, *, deadline_monotonic_us):
        self._ready()
        self._events_need_sync = True
        self._deadline(deadline_monotonic_us, Stage.WAIT_IRQ)
        if self._events_untrusted:
            self._event_error()
        # One final empty poll follows at most one kernel buffer of stale events.
        for count in range(_EVENT_BUFFER_SIZE + 1):
            if self._observed_edge is not None and self._observed_edge.timestamp_ns > self._observed_at_us * 1000 + 999:
                self._event_error(untrusted=True)
            self._deadline(deadline_monotonic_us, Stage.WAIT_IRQ)
            available = self._events_available(timedelta(0))
            self._deadline(deadline_monotonic_us, Stage.WAIT_IRQ, after=True)
            if not available:
                if self._observed_edge is not None:
                    self._last_sequence = self._observed_edge.sequence
                    self._last_timestamp_ns = self._observed_edge.timestamp_ns
                self._events_need_sync = False
                return
            if count == _EVENT_BUFFER_SIZE:
                self._event_error(code=Error.UNEXPECTED_IRQ)
            self._deadline(deadline_monotonic_us, Stage.WAIT_IRQ)
            self._consume_edge()
            self._deadline(deadline_monotonic_us, Stage.WAIT_IRQ, after=True)

    def _release(self):
        failures = []
        for name, method, stage in (
            ("_spi", "close", Stage.CONFIGURE_SPI),
            ("_request", "release", Stage.DETACH_IRQ),
        ):
            resource = getattr(self, name)
            setattr(self, name, None)
            if resource is not None:
                try:
                    self._os_call(stage, getattr(resource, method))
                except BaseException as error:
                    failures.append(error)
        return tuple(failures)

    @staticmethod
    def _raise_lifecycle(primary, releases):
        errors = ((primary,) if primary is not None else ()) + releases
        if not errors:
            return
        unexpected = tuple(error for error in errors if not isinstance(error, RadioBackendError))
        primary_failure = primary.failure if isinstance(primary, RadioBackendError) else None
        release_failures = tuple(error.failure for error in releases if isinstance(error, RadioBackendError))
        normalized = ()
        if primary_failure is not None or release_failures:
            normalized = (RadioLifecycleError(RadioLifecycleFailure(primary_failure, release_failures)),)
        if unexpected:
            if len(errors) == 1:
                raise unexpected[0]
            raise BaseExceptionGroup("radio resource lifecycle failures", normalized + unexpected)
        raise normalized[0] from primary

    def close(self):
        self._claim()
        self._raise_lifecycle(None, self._release())
