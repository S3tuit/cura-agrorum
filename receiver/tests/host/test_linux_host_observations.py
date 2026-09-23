from pathlib import Path

from cura_receiver.platform.linux_host_observations import LinuxHostObservations


# Linux file units are normalized and an observed absent WAL is zero, not missing telemetry.
def test_host_observation_units(tmp_path, monkeypatch):
    path = tmp_path / "receiver.db"
    path.write_bytes(b"123")
    real_read = Path.read_text

    def read(self, *args, **kwargs):
        if str(self) == "/proc/meminfo":
            return "MemTotal: 10000 kB\nMemAvailable: 123 kB\n"
        if str(self) == "/sys/class/thermal/thermal_zone0/temp":
            return "42500\n"
        return real_read(self, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read)
    monkeypatch.setattr("os.getloadavg", lambda: (1.25, 0, 0))
    actual = LinuxHostObservations(path).sample()
    assert actual.linux_load_1m_milli == 1250
    assert actual.cpu_temperature_milli_c == 42500
    assert actual.memory_available_bytes == 125952
    assert actual.sqlite_database_size_bytes == 3
    assert actual.sqlite_wal_size_bytes == 0
    assert actual.sqlite_filesystem_available_bytes > 0
    assert actual.ntp_offset_us is None


# Independent missing host observations remain NULL without losing the complete health row.
def test_host_observation_failures_are_independent(tmp_path, monkeypatch):
    def fail(*args, **kwargs):
        raise OSError("unavailable")

    monkeypatch.setattr(Path, "read_text", fail)
    monkeypatch.setattr("os.getloadavg", fail)
    actual = LinuxHostObservations(tmp_path / "missing.db").sample()
    assert actual.linux_load_1m_milli is None
    assert actual.cpu_temperature_milli_c is None
    assert actual.memory_available_bytes is None
    assert actual.sqlite_database_size_bytes is None
    assert actual.sqlite_filesystem_available_bytes > 0
    assert actual.sqlite_wal_size_bytes == 0
