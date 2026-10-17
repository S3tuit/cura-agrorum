from pathlib import Path

import pytest

from cura_receiver.platform.linux_host_observations import LinuxHostObservations
from cura_receiver.ports.host_observations import HostObservations


@pytest.fixture
def linux_counters(monkeypatch):
    files = {
        "/sys/block/mmcblk7/device/type": "SD\n",
        "/sys/block/mmcblk7/stat": "1 2 3 4 5 6 12345 8 9 10 11 12 13 14 15 16 17\n",
        "/proc/self/io": (
            "rchar: 123456\nwchar: 654321\nsyscr: 30\nsyscw: 40\n"
            "read_bytes: 500\nwrite_bytes: 73\ncancelled_write_bytes: 50\n"
        ),
    }
    reads = []
    real_read = Path.read_text

    def read(path, *args, **kwargs):
        name = str(path)
        reads.append(name)
        if name not in files:
            return real_read(path, *args, **kwargs)
        value = files[name]
        if isinstance(value, Exception):
            raise value
        if isinstance(value, bytes):
            return value.decode(kwargs["encoding"])
        return value

    monkeypatch.setattr(Path, "read_text", read)
    return files, reads


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


# The selected card need not contain SQLite; process accounting ignores wchar and cancellations.
def test_write_counters_are_whole_card_and_process_accounting(tmp_path, linux_counters):
    _, reads = linux_counters
    source = LinuxHostObservations(tmp_path / "absent-database", microsd_device="mmcblk7")
    actual = source.sample()
    assert actual.storage_device_write_bytes == 6_320_640
    assert actual.receiver_process_write_bytes == 73
    assert actual.sqlite_database_size_bytes is None
    assert reads.count("/sys/block/mmcblk7/device/type") == 1
    assert "/proc/thread-self/io" not in reads


# Zero is observed, while invalid syntax and signed-64-bit overflow are absent telemetry.
@pytest.mark.parametrize("value,expected", [
    ("0", 0), (str((1 << 63) - 1), (1 << 63) - 1),
    (str(1 << 63), None), (str((1 << 64) - 1), None),
    ("-1", None), ("+1", None), ("1_024", None), ("1.5", None),
    ("0x12", None), ("１２", None), ("9" * 5000, None),
])
def test_process_counter_numeric_boundaries(tmp_path, linux_counters, value, expected):
    files, _ = linux_counters
    files["/proc/self/io"] = f"write_bytes: {value}\n"
    actual = LinuxHostObservations(tmp_path / "db", microsd_device="mmcblk7").sample()
    assert actual.receiver_process_write_bytes == expected
    assert actual.storage_device_write_bytes == 6_320_640


# The largest storable card value is sector-aligned; check bounds after conversion.
@pytest.mark.parametrize("sectors,expected", [
    ("0", 0), (str((1 << 54) - 1), (1 << 63) - 512),
    (str(1 << 54), None), ("-1", None), ("+1", None), ("1_024", None),
])
def test_device_counter_numeric_boundaries(tmp_path, linux_counters, sectors, expected):
    files, _ = linux_counters
    files["/sys/block/mmcblk7/stat"] = f"1 2 3 4 5 6 {sectors} 8 9 10 11\n"
    actual = LinuxHostObservations(tmp_path / "db", microsd_device="mmcblk7").sample()
    assert actual.storage_device_write_bytes == expected
    assert actual.receiver_process_write_bytes == 73


# New kernel fields append to the original eleven without shifting field seven.
@pytest.mark.parametrize("extra", ["", " 12 13 14 15", " 12 13 14 15 16 17", " 12 13 14 15 16 17 18 19"])
def test_block_stat_kernel_layouts(tmp_path, linux_counters, extra):
    files, _ = linux_counters
    files["/sys/block/mmcblk7/stat"] = "1 2 3 4 5 6 7 8 9 10 11" + extra + "\n"
    assert LinuxHostObservations(tmp_path / "db", microsd_device="mmcblk7").sample().storage_device_write_bytes == 3584


# A missing, duplicated or malformed exact key cannot be replaced by a similarly named field.
@pytest.mark.parametrize("contents", [
    "", "cancelled_write_bytes: 9\nwchar: 7\n", "write_bytes:\n",
    "write_bytes: 1 extra\n", "write_bytes: 0\nwrite_bytes: 1\n",
    "write_bytes:1\n", b"write_bytes: \xff\n",
])
def test_malformed_process_counter_is_independent(tmp_path, linux_counters, contents):
    files, _ = linux_counters
    files["/proc/self/io"] = contents
    actual = LinuxHostObservations(tmp_path / "db", microsd_device="mmcblk7").sample()
    assert actual.receiver_process_write_bytes is None
    assert actual.storage_device_write_bytes == 6_320_640


@pytest.mark.parametrize("contents", [
    "", "1 2 3 4 5 6 7 8 9 10\n", "1 2 3 4 5 6 bad 8 9 10 11\n",
    "1 2 3 4 5 6 7 8 9 10 11\n1 2 3 4 5 6 8 8 9 10 11\n",
    b"1 2 3 4 5 6 \xff 8 9 10 11\n",
])
def test_malformed_block_stat_is_independent(tmp_path, linux_counters, contents):
    files, _ = linux_counters
    files["/sys/block/mmcblk7/stat"] = contents
    actual = LinuxHostObservations(tmp_path / "db", microsd_device="mmcblk7").sample()
    assert actual.storage_device_write_bytes is None
    assert actual.receiver_process_write_bytes == 73


# Permission, disappearance and decoding faults remain local even during initial binding.
@pytest.mark.parametrize("path", ["/sys/block/mmcblk7/device/type", "/sys/block/mmcblk7/stat", "/proc/self/io"])
@pytest.mark.parametrize("failure", [FileNotFoundError(), PermissionError(), b"\xff"])
def test_write_source_failures_are_independent(tmp_path, linux_counters, path, failure):
    files, _ = linux_counters
    files[path] = failure
    actual = LinuxHostObservations(tmp_path / "db", microsd_device="mmcblk7").sample()
    assert actual.storage_device_write_bytes == (6_320_640 if path == "/proc/self/io" else None)
    assert actual.receiver_process_write_bytes == (None if path == "/proc/self/io" else 73)


# Unsupported selections are telemetry absence, never a fallback to another card or database device.
@pytest.mark.parametrize("device", [None, "", "mmcblk7p1", "mmcblk7boot0", "sda", "dm-0", "/dev/mmcblk7", "../mmcblk7", "mmcblk７", "mmcblk7\n"])
def test_unsupported_device_selection_is_absent(tmp_path, linux_counters, device):
    _, reads = linux_counters
    actual = LinuxHostObservations(tmp_path / "db", microsd_device=device).sample()
    assert actual.storage_device_write_bytes is None
    assert actual.receiver_process_write_bytes == 73
    assert not any(name.startswith("/sys/block/") for name in reads)


@pytest.mark.parametrize("card_type", ["MMC\n", "SDIO\n", "SDcombo\n", "", "SD\nMMC\n"])
def test_only_sd_card_type_is_supported(tmp_path, linux_counters, card_type):
    files, _ = linux_counters
    files["/sys/block/mmcblk7/device/type"] = card_type
    assert LinuxHostObservations(tmp_path / "db", microsd_device="mmcblk7").sample().storage_device_write_bytes is None


# A failed binding stays absent for this source's lifetime even if a device later appears.
def test_failed_binding_is_not_retried(tmp_path, linux_counters):
    files, reads = linux_counters
    files["/sys/block/mmcblk7/device/type"] = FileNotFoundError()
    source = LinuxHostObservations(tmp_path / "db", microsd_device="mmcblk7")
    files["/sys/block/mmcblk7/device/type"] = "SD\n"
    assert source.sample().storage_device_write_bytes is None
    assert source.sample().storage_device_write_bytes is None
    assert reads.count("/sys/block/mmcblk7/device/type") == 1
    assert "/sys/block/mmcblk7/stat" not in reads


# Later source failure cannot trigger selection of another readable SD card.
def test_bound_device_never_switches(tmp_path, linux_counters):
    files, reads = linux_counters
    source = LinuxHostObservations(tmp_path / "db", microsd_device="mmcblk7")
    assert source.sample().storage_device_write_bytes == 6_320_640
    files["/sys/block/mmcblk7/stat"] = FileNotFoundError()
    files["/sys/block/mmcblk0/device/type"] = "SD\n"
    files["/sys/block/mmcblk0/stat"] = "1 2 3 4 5 6 999 8 9 10 11\n"
    actual = source.sample()
    assert actual.storage_device_write_bytes is None
    assert actual.receiver_process_write_bytes == 73
    assert reads.count("/sys/block/mmcblk7/device/type") == 1
    assert "/sys/block/mmcblk0/stat" not in reads


@pytest.mark.parametrize("field", ["storage_device_write_bytes", "receiver_process_write_bytes"])
@pytest.mark.parametrize("value", [None, 0, (1 << 63) - 1])
def test_host_write_counter_stored_range(field, value):
    assert getattr(HostObservations(**{field: value}), field) == value


@pytest.mark.parametrize("field", ["storage_device_write_bytes", "receiver_process_write_bytes"])
@pytest.mark.parametrize("value,error", [(-1, OverflowError), (1 << 63, OverflowError), (True, TypeError), (1.5, TypeError), ("12", TypeError)])
def test_host_write_counter_rejects_invalid_values(field, value, error):
    with pytest.raises(error):
        HostObservations(**{field: value})


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
    assert actual.storage_device_write_bytes is None
    assert actual.receiver_process_write_bytes is None
