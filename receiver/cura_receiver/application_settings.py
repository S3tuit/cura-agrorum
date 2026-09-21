"""Explicit approved pilot deployment profile; no file access or credentials."""

from dataclasses import dataclass, field
from pathlib import Path

from .communicator_state_persistence import CommunicatorStatePolicy
from .elapsed_duration import checked_duration_us
from .ports.radio import RadioConfiguration
from .runtime_time import RuntimeTimeSettings
from .time_policy import TimePolicy


@dataclass(frozen=True, slots=True)
class ApplicationSettings:
    configuration_path: Path = Path("/etc/cura-agrorum/receiver-group.json")
    database_path: Path = Path("/var/lib/cura-agrorum/receiver.sqlite3")
    sqlite_temporary_directory: Path = Path("/var/lib/cura-agrorum/tmp")
    minimum_free_bytes: int = 1 << 30
    health_interval_us: int = 60_000_000
    shutdown_budget_us: int = 10_000_000
    time_policy: TimePolicy = field(default_factory=TimePolicy)
    time_settings: RuntimeTimeSettings = field(default_factory=RuntimeTimeSettings)
    airtime_policy: CommunicatorStatePolicy = field(default_factory=CommunicatorStatePolicy)
    radio: RadioConfiguration = field(default_factory=RadioConfiguration)

    def __post_init__(self):
        paths = (self.configuration_path, self.database_path, self.sqlite_temporary_directory)
        for path in paths:
            if not isinstance(path, Path) or not path.is_absolute() or ".." in path.parts:
                raise ValueError("application paths must be absolute without parent traversal")
        if len(set(paths)) != len(paths):
            raise ValueError("configuration, database and temporary paths must be distinct")
        checked_duration_us(self.minimum_free_bytes)
        for duration in (self.health_interval_us, self.shutdown_budget_us):
            checked_duration_us(duration)
            if duration == 0:
                raise ValueError("application intervals must be positive")
        for value, expected in (
            (self.time_policy, TimePolicy),
            (self.time_settings, RuntimeTimeSettings),
            (self.airtime_policy, CommunicatorStatePolicy),
            (self.radio, RadioConfiguration),
        ):
            if type(value) is not expected:
                raise TypeError("application settings require validated component settings")
