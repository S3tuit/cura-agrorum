"""Best-effort Linux observations; none of these values controls time policy."""

from __future__ import annotations

import os
from collections.abc import Callable
from pathlib import Path

from ..ports.host_observations import HostObservations


def _optional(read: Callable[[], int], minimum: int, maximum: int) -> int | None:
    try:
        value = read()
        return value if type(value) is int and minimum <= value <= maximum else None
    except (OSError, ValueError, OverflowError):
        return None


class LinuxHostObservations:
    def __init__(self, database_path: Path) -> None:
        self._path = database_path

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
        )
