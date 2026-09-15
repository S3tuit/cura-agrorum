"""Pinned chronyc CSV adapter with fixed commands and an explicit Unix socket."""

from decimal import Decimal, InvalidOperation, localcontext, ROUND_CEILING, ROUND_UP
import ipaddress
from pathlib import Path
import re

from ..ports.chrony import (
    ChronyTrackingResult,
    ChronyQueryStatus as Q,
    ChronyStepResult,
    ChronyStepDisposition as D,
)
from ..time_diagnostics import integer


from ._time_process import _ChildResult, _run_child

_NUMBER = re.compile(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d{1,3})?\Z")


def parse_tracking(data, start, finish):
    if type(data) is not bytes or len(data) > 4096:
        raise ValueError("invalid bounded tracking response")
    fields = data.decode("ascii").strip().split(",")
    if len(fields) != 14 or not re.fullmatch("[0-9A-F]{8}", fields[0]):
        raise ValueError("unsupported tracking layout")
    if not re.fullmatch(r"\d{1,2}", fields[2]) or not 0 <= int(fields[2]) <= 16:
        raise ValueError("invalid tracking stratum")
    if fields[1] not in ("", "[UNSPEC]", "0.0.0.0"):
        ipaddress.ip_address(fields[1])
    if fields[13] not in (
        "Normal",
        "Insert second",
        "Delete second",
        "Not synchronised",
    ):
        raise ValueError("invalid leap state")
    with localcontext() as ctx:
        ctx.prec = 80
        numbers = []
        for item in fields[3:13]:
            if len(item) > 64 or not _NUMBER.fullmatch(item):
                raise ValueError("invalid tracking numeric field")
            value = Decimal(item)
            if not value.is_finite() or abs(value) > Decimal(2) ** 64:
                raise ValueError("out-of-range tracking value")
            numbers.append(value)
        # Ref time, RMS offset, skew, delay, dispersion and interval are unsigned.
        for i in (0, 3, 6, 7, 8, 9):
            if numbers[i] < 0:
                raise ValueError("negative unsigned tracking input")
        correction = int((numbers[1] * 1_000_000).to_integral_value(rounding=ROUND_UP))
        distance = int(
            ((numbers[7] / 2 + numbers[8]) * 1_000_000).to_integral_value(
                rounding=ROUND_CEILING
            )
        )
        skew = int((numbers[6] * 1000).to_integral_value(rounding=ROUND_CEILING))
    selected = fields[0] not in ("00000000", "7F7F0101") and 1 <= int(fields[2]) <= 15
    return ChronyTrackingResult(
        Q.OK,
        start,
        finish,
        selected,
        selected and fields[13] == "Normal",
        correction,
        distance,
        skew,
    )


class LinuxChronyControl:
    def __init__(
        self,
        clock,
        *,
        socket_path,
        deadline_monotonic_us,
        executable="/usr/bin/chronyc",
    ):
        for path in (socket_path, executable):
            if (
                type(path) is not str
                or not Path(path).is_absolute()
                or any(c in path for c in ",\x00\n")
            ):
                raise ValueError("one fixed absolute executable/socket path required")
        self.clock, self.socket_path, self.executable = clock, socket_path, executable
        integer(deadline_monotonic_us)
        version = _run_child((executable, "-v"), clock, deadline_monotonic_us)
        if (
            version.returncode != 0
            or version.timed_out
            or version.overflow
            or version.stderr
            or not re.fullmatch(
                rb"chronyc \(chrony\) version 4\.6\.1 \([^\r\n]*\)\n?", version.stdout
            )
        ):
            raise ValueError("unsupported/unavailable chronyc 4.6.1 executable")

    def _invoke(self, command, deadline):
        return _run_child(
            (self.executable, "-n", "-c", "-h", self.socket_path, command),
            self.clock,
            deadline,
        )

    def read_tracking(self, *, deadline_monotonic_us):
        integer(deadline_monotonic_us)
        start = self.clock.now_monotonic_us()
        result = self._invoke("tracking", deadline_monotonic_us)
        finish = self.clock.now_monotonic_us()
        if result.timed_out or finish >= deadline_monotonic_us:
            return ChronyTrackingResult(Q.DEADLINE_EXCEEDED, start, finish)
        if not result.started or result.stdout.strip() == b"506 Cannot talk to daemon":
            return ChronyTrackingResult(Q.UNAVAILABLE, start, finish)
        if result.returncode != 0 or result.overflow or result.stderr:
            return ChronyTrackingResult(Q.INVALID_RESPONSE, start, finish)
        try:
            return parse_tracking(result.stdout, start, finish)
        except (ValueError, UnicodeError, InvalidOperation, OverflowError):
            return ChronyTrackingResult(Q.INVALID_RESPONSE, start, finish)

    def apply_pending_correction_by_step(self, *, deadline_monotonic_us):
        integer(deadline_monotonic_us)
        start = self.clock.now_monotonic_us()
        result = self._invoke("makestep", deadline_monotonic_us)
        finish = self.clock.now_monotonic_us()
        disposition = D.OUTCOME_UNKNOWN
        if not result.started:
            disposition = D.NOT_SUBMITTED
        elif (
            not result.timed_out
            and finish < deadline_monotonic_us
            and not result.overflow
            and not result.stderr
        ):
            if result.returncode == 0 and result.stdout.strip() == b"200 OK":
                disposition = D.SUBMITTED
            elif result.stdout.strip() in (
                b"501 Not authorised",
                b"502 Invalid command",
            ):
                disposition = D.NOT_SUBMITTED
        return ChronyStepResult(disposition, start, finish)
