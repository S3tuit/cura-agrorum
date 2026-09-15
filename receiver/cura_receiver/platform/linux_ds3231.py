"""Linux RTC class read and a pinned, capability-limited native write helper."""

from datetime import datetime, timezone
import errno
import fcntl
import hashlib
import os
from pathlib import Path
import stat
import struct

from ..ports.ds3231 import (
    Ds3231ReadResult,
    Ds3231ReadStatus as R,
    Ds3231WriteResult,
    Ds3231WriteDisposition as W,
    Ds3231Failure as F,
    RTC_MIN_UTC_S,
    RTC_MAX_UTC_S,
)
from ..time_diagnostics import integer
from .linux_kernel_clock import validate_native_abi
from ._time_process import _run_child

RTC_RD_TIME = 0x80247009
RTC_SET_TIME = 0x4024700A
_RTC_TIME = struct.Struct("=9i")
_MISSING = {errno.ENOENT, errno.ENODEV, errno.ENXIO, errno.EREMOTEIO}


def rtc_seconds(data):
    sec, minute, hour, day, month, year, weekday, yearday, isdst = _RTC_TIME.unpack(
        data
    )
    if not (
        0 <= sec <= 59
        and 0 <= minute <= 59
        and 0 <= hour <= 23
        and 0 <= month <= 11
        and 100 <= year <= 199
    ):
        raise ValueError("invalid RTC calendar")
    value = datetime(
        year + 1900, month + 1, day, hour, minute, sec, tzinfo=timezone.utc
    )
    # Pure datetime arithmetic avoids platform timestamp/timezone conversion.
    return (
        (value - datetime(1970, 1, 1, tzinfo=timezone.utc)).days * 86400
        + hour * 3600
        + minute * 60
        + sec
    )


def validate_helper(path, expected_sha256, receiver_gid):
    """Validate deployment evidence without executing any helper operation."""
    path = Path(path)
    if not path.is_absolute() or path.is_symlink():
        raise ValueError("helper must be a fixed absolute regular file")
    info = path.stat()
    if (
        not stat.S_ISREG(info.st_mode)
        or info.st_uid != 0
        or info.st_gid != receiver_gid
        or stat.S_IMODE(info.st_mode) != 0o750
    ):
        raise ValueError("helper requires root:receiver ownership and mode 0750")
    for parent in path.parents:
        metadata = parent.stat()
        if parent.is_symlink() or metadata.st_uid != 0 or metadata.st_mode & 0o022:
            raise ValueError("helper parent directory is writable outside root")
    if os.statvfs(path).f_flag & os.ST_NOSUID:
        raise ValueError("helper filesystem disables file capabilities")
    if type(expected_sha256) is not bytes or len(expected_sha256) != 32:
        raise ValueError("trusted helper SHA256 required")
    with path.open("rb") as stream:
        if stream.read(4) != b"\x7fELF":
            raise ValueError("native ELF helper required")
        stream.seek(0)
        digest = hashlib.file_digest(stream, "sha256").digest()
    if digest != expected_sha256:
        raise ValueError("helper content differs from deployment pin")
    caps = os.getxattr(path, "security.capability")
    words = struct.unpack("<" + "I" * (len(caps) // 4), caps)
    if words not in ((0x02000001, 1 << 25, 0, 0, 0), (0x03000001, 1 << 25, 0, 0, 0, 0)):
        raise ValueError("helper requires exactly cap_sys_time=ep")
    status = dict(
        line.split(":", 1)
        for line in Path("/proc/self/status").read_text().splitlines()
        if ":" in line
    )
    if any(int(status[name], 16) for name in ("CapInh", "CapPrm", "CapEff", "CapAmb")):
        raise ValueError("receiver parent must hold no capabilities")
    if not int(status["CapBnd"], 16) & (1 << 25) or int(status["NoNewPrivs"]) != 0:
        raise ValueError("helper capability cannot be acquired in this service context")


class LinuxDs3231Control:
    def __init__(
        self,
        clock,
        *,
        kernel_operation_bound_us,
        device_path="/dev/rtc-ds3231",
        sysfs_root="/sys/class/rtc",
        helper_path="/usr/libexec/cura-agrorum/ds3231-set",
        helper_sha256=None,
        receiver_gid=None,
    ):
        validate_native_abi()
        integer(kernel_operation_bound_us, 1)
        if not Path(device_path).is_absolute() or not Path(helper_path).is_absolute():
            raise ValueError("fixed absolute RTC paths required")
        self.clock, self.device_path, self.sysfs_root = (
            clock,
            device_path,
            Path(sysfs_root),
        )
        self.kernel_operation_bound_us = kernel_operation_bound_us
        self.helper_path = helper_path
        self.write_available = helper_sha256 is not None
        if self.write_available:
            validate_helper(helper_path, helper_sha256, receiver_gid)

    def _read_device(self):
        fd = os.open(self.device_path, os.O_RDONLY | os.O_CLOEXEC)
        try:
            info = os.fstat(fd)
            if not stat.S_ISCHR(info.st_mode):
                raise ValueError("configured RTC is not a character device")
            # Match the opened device number, never assume rtc0 is the DS3231.
            matched = [
                p
                for p in self.sysfs_root.glob("rtc*")
                if (p / "dev").read_text().strip()
                == f"{os.major(info.st_rdev)}:{os.minor(info.st_rdev)}"
            ]
            if len(matched) != 1 or not (matched[0] / "name").read_text().startswith(
                "rtc-ds1307 "
            ):
                raise ValueError("configured RTC driver does not match deployment")
            compatible = (
                (matched[0] / "device/of_node/compatible").read_bytes().split(b"\x00")
            )
            if b"maxim,ds3231" not in compatible:
                raise ValueError("configured RTC is not a DS3231")
            data = bytearray(_RTC_TIME.size)
            fcntl.ioctl(fd, RTC_RD_TIME, data, True)
            return rtc_seconds(data)
        finally:
            os.close(fd)

    def read_time(self, *, deadline_monotonic_us):
        integer(deadline_monotonic_us)
        start = self.clock.now_monotonic_us()
        if deadline_monotonic_us - start < self.kernel_operation_bound_us:
            return Ds3231ReadResult(R.DEADLINE_EXCEEDED, start, start)
        status, seconds, error = R.OK, None, None
        try:
            seconds = self._read_device()
        except OSError as exc:
            error = exc.errno
            status = (
                R.MISSING
                if error in _MISSING
                else R.INVALID if error == errno.EINVAL else R.IO_ERROR
            )
        except (ValueError, OverflowError):
            status = R.INVALID
        finish = self.clock.now_monotonic_us()
        if finish >= deadline_monotonic_us:
            status = R.DEADLINE_EXCEEDED
        return Ds3231ReadResult(
            status, start, finish, seconds if status is R.OK else None, error
        )

    def write_time(self, *, rtc_utc_s, deadline_monotonic_us):
        integer(rtc_utc_s, RTC_MIN_UTC_S, RTC_MAX_UTC_S)
        integer(deadline_monotonic_us)
        start = self.clock.now_monotonic_us()
        if start >= deadline_monotonic_us:
            return Ds3231WriteResult(W.NOT_APPLIED, F.DEADLINE_EXCEEDED, start, start)
        if not self.write_available:
            return Ds3231WriteResult(W.NOT_APPLIED, F.IO_ERROR, start, start)
        result = _run_child(
            (self.helper_path, str(rtc_utc_s)),
            self.clock,
            deadline_monotonic_us,
            reap_timeout_s=max(1, self.kernel_operation_bound_us / 1_000_000),
        )
        finish = self.clock.now_monotonic_us()
        failure = (
            F.DEADLINE_EXCEEDED
            if result.timed_out or finish >= deadline_monotonic_us
            else F.IO_ERROR
        )
        disposition = W.OUTCOME_UNKNOWN if result.started else W.NOT_APPLIED
        if not result.started and result.os_errno in _MISSING:
            failure = F.MISSING
        if (
            result.started
            and not result.timed_out
            and finish < deadline_monotonic_us
            and not result.overflow
            and not result.stdout
            and not result.stderr
        ):
            if result.returncode == 32:
                disposition, failure = W.COMPLETED, F.NONE
            elif result.returncode in (33, 34, 35):
                disposition = W.NOT_APPLIED
                failure = F.MISSING if result.returncode == 34 else F.IO_ERROR
        return Ds3231WriteResult(disposition, failure, start, finish, result.os_errno)
