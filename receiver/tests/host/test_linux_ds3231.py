from dataclasses import replace
import errno
from pathlib import Path
import struct
import subprocess

import pytest

from cura_receiver.platform import linux_ds3231 as L
from cura_receiver.platform._time_process import _ChildResult
from cura_receiver.ports.ds3231 import (
    Ds3231ReadResult,
    Ds3231ReadStatus as R,
    Ds3231WriteResult,
    Ds3231WriteDisposition as W,
    Ds3231Failure as F,
)
from tests.support.fakes.os_clock import FakeOsClock


def adapter():
    return L.LinuxDs3231Control(
        FakeOsClock(monotonic_us=100), kernel_operation_bound_us=10
    )


# Calendar conversion is timezone-independent, rejects invalid dates and preserves the driver range.
@pytest.mark.parametrize(
    "calendar,expected",
    [
        ((0, 0, 0, 1, 0, 100, 0, 0, 0), 946684800),
        ((59, 59, 23, 31, 11, 199, 0, 0, 0), 4102444799),
        ((60, 0, 0, 1, 0, 100, 0, 0, 0), None),
        ((0, 0, 0, 29, 1, 101, 0, 0, 0), None),
        ((0, 0, 0, 1, 0, 99, 0, 0, 0), None),
    ],
)
def test_calendar(calendar, expected):
    data = struct.pack("=9i", *calendar)
    if expected is None:
        with pytest.raises(ValueError):
            L.rtc_seconds(data)
    else:
        assert L.rtc_seconds(data) == expected


# Read statuses preserve missing, invalid oscillator/calendar, exceptional IO and actual late return.
@pytest.mark.parametrize(
    "error,status",
    [
        (errno.ENOENT, R.MISSING),
        (errno.ENXIO, R.MISSING),
        (errno.EREMOTEIO, R.MISSING),
        (errno.EINVAL, R.INVALID),
        (errno.EIO, R.IO_ERROR),
        (errno.EACCES, R.IO_ERROR),
    ],
)
def test_read_errors(error, status):
    rtc = adapter()

    def read():
        raise OSError(error, "fixture")

    rtc._read_device = read
    result = rtc.read_time(deadline_monotonic_us=200)
    assert (
        result.status is status
        and result.os_errno == error
        and result.rtc_utc_s is None
    )


# Entry expiry/insufficient kernel bound performs no IO; equality at completion rejects a late value.
def test_read_deadlines():
    rtc = adapter()
    calls = []

    def read():
        calls.append(1)
        rtc.clock.advance_elapsed_us(100)
        return 946684800

    rtc._read_device = read
    assert rtc.read_time(deadline_monotonic_us=109).status is R.DEADLINE_EXCEEDED
    assert not calls
    result = rtc.read_time(deadline_monotonic_us=200)
    assert (
        result.status is R.DEADLINE_EXCEEDED
        and result.operation_finished_at_monotonic_us == 200
    )


# All valid write effect/failure pairs construct; impossible combinations cannot enter policy.
def test_write_result_matrix():
    for disposition in W:
        for failure in F:
            if (disposition is W.COMPLETED) == (failure is F.NONE):
                Ds3231WriteResult(disposition, failure, 1, 2)
            else:
                with pytest.raises(ValueError):
                    Ds3231WriteResult(disposition, failure, 1, 2)


# Only V1 success proves completion; every lost/malformed possibly executed reply stays unknown.
@pytest.mark.parametrize(
    "child,expected,failure",
    [
        (_ChildResult(32, started=True), W.COMPLETED, F.NONE),
        (_ChildResult(33, started=True), W.NOT_APPLIED, F.IO_ERROR),
        (_ChildResult(34, started=True), W.NOT_APPLIED, F.MISSING),
        (_ChildResult(35, started=True), W.NOT_APPLIED, F.IO_ERROR),
        (_ChildResult(36, started=True), W.OUTCOME_UNKNOWN, F.IO_ERROR),
        (_ChildResult(0, started=True), W.OUTCOME_UNKNOWN, F.IO_ERROR),
        (_ChildResult(32, b"bad", started=True), W.OUTCOME_UNKNOWN, F.IO_ERROR),
        (_ChildResult(-9, started=True), W.OUTCOME_UNKNOWN, F.IO_ERROR),
        (
            _ChildResult(32, started=True, timed_out=True),
            W.OUTCOME_UNKNOWN,
            F.DEADLINE_EXCEEDED,
        ),
        (_ChildResult(timed_out=True), W.NOT_APPLIED, F.DEADLINE_EXCEEDED),
        (_ChildResult(os_errno=errno.ENOENT), W.NOT_APPLIED, F.MISSING),
    ],
)
def test_write_outcomes(monkeypatch, child, expected, failure):
    rtc = adapter()
    rtc.write_available = True
    calls = []

    def run(argv, clock, deadline, *, reap_timeout_s):
        calls.append(argv)
        return child

    monkeypatch.setattr(L, "_run_child", run)
    result = rtc.write_time(rtc_utc_s=946684800, deadline_monotonic_us=200)
    assert result.disposition is expected and result.failure is failure
    assert calls == [("/usr/libexec/cura-agrorum/ds3231-set", "946684800")]


# Invalid write arguments and expired entry fail before helper execution.
def test_write_entry_rejection(monkeypatch):
    rtc = adapter()
    rtc.write_available = True
    monkeypatch.setattr(
        L, "_run_child", lambda *a: pytest.fail("helper unexpectedly called")
    )
    for value in (True, 0, -1, 946684799, 4102444800, "946684800"):
        with pytest.raises((ValueError, TypeError)):
            rtc.write_time(rtc_utc_s=value, deadline_monotonic_us=200)
    assert (
        rtc.write_time(rtc_utc_s=946684800, deadline_monotonic_us=100).failure
        is F.DEADLINE_EXCEEDED
    )


@pytest.fixture(scope="module")
def helper(tmp_path_factory):
    root = tmp_path_factory.mktemp("rtc-native")
    binary = root / "ds3231-set"
    source = Path(__file__).resolve().parents[2] / "native/ds3231-set.c"
    subprocess.run(
        [
            "cc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-O2",
            str(source),
            "-o",
            str(binary),
        ],
        check=True,
    )
    return binary


# Native argument parsing rejects noncanonical and out-of-range input before opening any device.
@pytest.mark.parametrize(
    "args",
    [
        [],
        [""],
        ["+946684800"],
        ["0946684800"],
        ["946684800x"],
        ["946684799"],
        ["4102444800"],
        ["9" * 100],
        ["946684800", "extra"],
    ],
)
def test_native_argument_protocol(helper, args):
    result = subprocess.run([str(helper), *args], capture_output=True, timeout=2)
    assert result.returncode == 33 and result.stdout == result.stderr == b""


# Native syscall interception verifies exactly one fixed-device write and all effect boundaries.
def test_native_fixed_ioctl(tmp_path):
    source = Path(__file__).resolve().parents[2] / "native/ds3231-set.c"
    wrapper = tmp_path / "wrapper.c"
    wrapper.write_text("""#define _GNU_SOURCE
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/rtc.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
static int mode;
static int fixture_open(const char *path,int flags,...) {
 if(strcmp(path,"/dev/rtc-ds3231") || flags!=(O_RDWR|O_CLOEXEC)) abort();
 mode=atoi(getenv("FIXTURE_MODE"));
 if(mode==1){errno=ENOENT;return -1;} if(mode==2){errno=EACCES;return -1;} return 123;
}
static int fixture_ioctl(int fd,unsigned long command,...) {
 if(fd!=123 || command!=RTC_SET_TIME) abort();
 va_list args;va_start(args,command);struct rtc_time *rtc=va_arg(args,struct rtc_time*);va_end(args);
 if(rtc->tm_year!=100 || rtc->tm_mon!=0 || rtc->tm_mday!=1) abort();
 if(mode==3){errno=EIO;return -1;} return 0;
}
static int fixture_close(int fd){if(fd!=123)abort();return 0;}
#define open fixture_open
#define ioctl fixture_ioctl
#define close fixture_close
""" + '#include "' + str(source) + '"\n')
    binary = tmp_path / "fixture"
    subprocess.run(
        ["cc", "-Wall", "-Wextra", "-Werror", str(wrapper), "-o", str(binary)],
        check=True,
    )
    for mode, expected in [(0, 32), (1, 34), (2, 35), (3, 36)]:
        result = subprocess.run(
            [str(binary), "946684800"],
            env={"FIXTURE_MODE": str(mode)},
            capture_output=True,
            timeout=2,
        )
        assert result.returncode == expected and result.stdout == result.stderr == b""


# Deployment validation checks the actual bytes while OS-owned metadata is supplied at its boundary.
@pytest.mark.parametrize(
    "fault",
    [
        None,
        "owner",
        "group",
        "mode",
        "parent",
        "hash",
        "elf",
        "capability",
        "parent_cap",
        "bounding",
        "nnp",
        "nosuid",
    ],
)
def test_helper_deployment_contract(tmp_path, monkeypatch, fault):
    from types import SimpleNamespace
    import hashlib
    import stat

    binary = tmp_path / "helper"
    binary.write_bytes((b"wrong" if fault == "elf" else b"\x7fELF") + b"fixture")
    trusted = hashlib.sha256(binary.read_bytes()).digest()
    original_stat, original_text = Path.stat, Path.read_text

    def metadata(path, *args, **kwargs):
        if path == binary:
            return SimpleNamespace(
                st_mode=stat.S_IFREG | (0o770 if fault == "mode" else 0o750),
                st_uid=1000 if fault == "owner" else 0,
                st_gid=0 if fault == "group" else 1000,
            )
        if path in binary.parents:
            return SimpleNamespace(
                st_uid=0, st_mode=stat.S_IFDIR | (0o777 if fault == "parent" else 0o755)
            )
        return original_stat(path, *args, **kwargs)

    def status(path, *args, **kwargs):
        if str(path) == "/proc/self/status":
            values = {
                "CapInh": "0",
                "CapPrm": "0",
                "CapEff": "0",
                "CapAmb": "0",
                "CapBnd": "02000000",
                "NoNewPrivs": "0",
            }
            if fault == "parent_cap":
                values["CapEff"] = "02000000"
            if fault == "bounding":
                values["CapBnd"] = "0"
            if fault == "nnp":
                values["NoNewPrivs"] = "1"
            return "\n".join(f"{key}:\t{value}" for key, value in values.items())
        return original_text(path, *args, **kwargs)

    monkeypatch.setattr(
        L.os,
        "statvfs",
        lambda path: SimpleNamespace(f_flag=L.os.ST_NOSUID if fault == "nosuid" else 0),
    )
    monkeypatch.setattr(Path, "stat", metadata)
    monkeypatch.setattr(Path, "read_text", status)
    monkeypatch.setattr(
        L.os,
        "getxattr",
        lambda *args: struct.pack(
            "<5I", 0x02000001, (1 << 25) | (1 if fault == "capability" else 0), 0, 0, 0
        ),
    )
    if fault == "hash":
        trusted = b"\x00" * 32
    if fault:
        with pytest.raises(ValueError):
            L.validate_helper(binary, trusted, 1000)
    else:
        L.validate_helper(binary, trusted, 1000)


# Match the opened device number and device-tree compatible string, independent of rtcN numbering.
@pytest.mark.parametrize(
    "fault", [None, "regular", "driver", "compatible", "duplicate"]
)
def test_opened_device_identity(tmp_path, monkeypatch, fault):
    from types import SimpleNamespace
    import stat

    device = tmp_path / "rtc7"
    (device / "device/of_node").mkdir(parents=True)
    (device / "dev").write_text("251:7\n")
    (device / "name").write_text(
        "other 1-0068" if fault == "driver" else "rtc-ds1307 1-0068"
    )
    (device / "device/of_node/compatible").write_bytes(
        b"other\0" if fault == "compatible" else b"maxim,ds3231\0"
    )
    if fault == "duplicate":
        (tmp_path / "rtc8").mkdir()
        (tmp_path / "rtc8/dev").write_text("251:7\n")
    rtc = adapter()
    rtc.sysfs_root = tmp_path
    opened, closed = [], []

    def open_device(path, flags):
        opened.append((path, flags))
        return 123

    monkeypatch.setattr(L.os, "open", open_device)
    monkeypatch.setattr(L.os, "close", closed.append)
    monkeypatch.setattr(
        L.os,
        "fstat",
        lambda fd: SimpleNamespace(
            st_mode=stat.S_IFREG if fault == "regular" else stat.S_IFCHR,
            st_rdev=L.os.makedev(251, 7),
        ),
    )

    def ioctl(fd, command, data, mutate):
        assert fd == 123 and command == L.RTC_RD_TIME and mutate
        data[:] = struct.pack("=9i", 0, 0, 0, 1, 0, 100, 0, 0, 0)

    monkeypatch.setattr(L.fcntl, "ioctl", ioctl)
    result = rtc.read_time(deadline_monotonic_us=200)
    assert result.status is (R.INVALID if fault else R.OK)
    assert closed == [123] and opened == [
        ("/dev/rtc-ds3231", L.os.O_RDONLY | L.os.O_CLOEXEC)
    ]
