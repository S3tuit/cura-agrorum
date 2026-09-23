"""One-time optional observations for persistence-owned health enrichment."""

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True, slots=True)
class HostObservations:
    linux_load_1m_milli: int | None = None
    cpu_temperature_milli_c: int | None = None
    memory_available_bytes: int | None = None
    sqlite_filesystem_available_bytes: int | None = None
    sqlite_database_size_bytes: int | None = None
    sqlite_wal_size_bytes: int | None = None
    ntp_offset_us: int | None = None

    def __post_init__(self) -> None:
        limits = {
            "linux_load_1m_milli": (0, (1 << 32) - 1),
            "cpu_temperature_milli_c": (-(1 << 31), (1 << 31) - 1),
            "memory_available_bytes": (0, (1 << 63) - 1),
            "sqlite_filesystem_available_bytes": (0, (1 << 63) - 1),
            "sqlite_database_size_bytes": (0, (1 << 63) - 1),
            "sqlite_wal_size_bytes": (0, (1 << 63) - 1),
            "ntp_offset_us": (-(1 << 63), (1 << 63) - 1),
        }
        for name, (minimum, maximum) in limits.items():
            value = getattr(self, name)
            if value is None:
                continue
            if type(value) is not int:
                raise TypeError("host observation must be an exact integer or absent")
            if not minimum <= value <= maximum:
                raise OverflowError("host observation exceeds its stored range")


class HostObservationSource(Protocol):
    def sample(self) -> HostObservations:
        """Sample once; unavailable individual observations are None."""
        ...
