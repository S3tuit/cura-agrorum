"""Best-effort Linux observations; none of these values controls time policy."""

from __future__ import annotations

import os
import re
from collections.abc import Callable
from pathlib import Path

from ..ports.host_observations import HostObservations

_INT64_MAX = (1 << 63) - 1


def _unsigned_decimal(value: str) -> int:
    if not value.isascii() or not value.isdecimal():
        raise ValueError("invalid unsigned counter")
    return int(value)


def _optional(read: Callable[[], int], minimum: int, maximum: int) -> int | None:
    try:
        value = read()
        return value if type(value) is int and minimum <= value <= maximum else None
    except (OSError, ValueError, OverflowError):
        return None


class LinuxHostObservations:
    def __init__(self, database_path: Path, *, microsd_device: str | None = None) -> None:
        self._path = database_path
        self._microsd_stat = self._bind_microsd(microsd_device)

    @staticmethod
    def _bind_microsd(device: str | None) -> Path | None:
        # Select only the configured native-slot SD card, never a partition or
        # another storage layer. Binding failure stays absent for this instance.
        if not isinstance(device, str) or re.fullmatch(r"mmcblk[0-9]+", device) is None:
            return None
        path = Path("/sys/block") / device
        try:
            if (path / "device/type").read_text(encoding="ascii").strip() != "SD":
                return None
        except (OSError, ValueError):
            return None
        return path / "stat"

    def _storage_write_bytes(self) -> int:
        if self._microsd_stat is None:
            raise ValueError("microSD observation unavailable")
        lines = self._microsd_stat.read_text(encoding="ascii").strip().splitlines()
        if len(lines) != 1:
            raise ValueError("invalid block statistics")
        parts = lines[0].split()
        if len(parts) < 11:
            raise ValueError("incomplete block statistics")
        counters = [_unsigned_decimal(part) for part in parts]
        # Kernel accounting always uses 512-byte sectors, regardless of the
        # card's logical/physical block size. Field 7 is sectors written.
        return counters[6] * 512

    @staticmethod
    def _process_write_bytes() -> int:
        value = None
        # /proc/self/io aggregates the process even from the persistence thread.
        for line in Path("/proc/self/io").read_text(encoding="ascii").splitlines():
            parts = line.split()
            if parts[:1] == ["write_bytes:"]:
                if len(parts) != 2 or value is not None:
                    raise ValueError("invalid process write counter")
                value = _unsigned_decimal(parts[1])
        if value is None:
            raise ValueError("process write counter unavailable")
        return value

    def _memory_available(self) -> int:
        for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
            parts = line.split()
            if parts[:1] == ["MemAvailable:"] and len(parts) == 3 and parts[2] == "kB":
                return int(parts[1]) * 1024
        raise ValueError("MemAvailable unavailable")

    def _filesystem_available(self) -> int:
        value = os.statvfs(self._path.parent)
        return value.f_bavail * value.f_frsize

    def _wal_size(self) -> int:
        try:
            return Path(str(self._path) + "-wal").stat().st_size
        except FileNotFoundError:
            return 0

    def sample(self) -> HostObservations:
        return HostObservations(
            linux_load_1m_milli=_optional(
                lambda: int(os.getloadavg()[0] * 1000), 0, (1 << 32) - 1
            ),
            cpu_temperature_milli_c=_optional(
                lambda: int(
                    Path("/sys/class/thermal/thermal_zone0/temp").read_text(
                        encoding="ascii"
                    )
                ),
                -(1 << 31),
                (1 << 31) - 1,
            ),
            memory_available_bytes=_optional(self._memory_available, 0, (1 << 63) - 1),
            sqlite_filesystem_available_bytes=_optional(
                self._filesystem_available, 0, (1 << 63) - 1
            ),
            sqlite_database_size_bytes=_optional(
                lambda: self._path.stat().st_size, 0, (1 << 63) - 1
            ),
            sqlite_wal_size_bytes=_optional(self._wal_size, 0, (1 << 63) - 1),
            storage_device_write_bytes=_optional(
                self._storage_write_bytes, 0, _INT64_MAX
            ),
            receiver_process_write_bytes=_optional(
                self._process_write_bytes, 0, _INT64_MAX
            ),
        )
