"""Trusted launch procedure: policy checking and the real deployment-audit entrypoint."""

import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from tools import check_chrony as check
from tests.hardware import test_runtime_time as fixture

CONFIG = """pool 2.debian.pool.ntp.org iburst
leapsecmode slew
maxslewrate 3500
cmdport 0
bindcmdaddress /run/chrony/chronyd.sock
"""
LAUNCH = ["/usr/sbin/chronyd", "-F", "1", "-f", "/etc/chrony/chrony.conf"]
PRECHECK = ["/usr/bin/python3", "/usr/libexec/cura-agrorum/check-chrony.py", "/etc/chrony/chrony.conf"]


@pytest.mark.parametrize("directive", ["makestep 1 3", "initstepslew 1 localhost", "rtcsync", "rtcfile /tmp/rtc"])
def test_policy_rejects_other_clock_writers(directive):
    with pytest.raises(ValueError, match="forbidden"):
        check.validate_configuration(CONFIG + directive + "\n")


@pytest.mark.parametrize("name,value", [
    ("leapsecmode", "slew"), ("maxslewrate", "3500"), ("cmdport", "0"),
    ("bindcmdaddress", "/run/chrony/chronyd.sock"),
])
@pytest.mark.parametrize("change", ["missing", "duplicate", "override"])
def test_policy_requires_unambiguous_settings(name, value, change):
    line = name + " " + value + "\n"
    effective = CONFIG.replace(line, "") if change == "missing" else CONFIG + (
        line if change == "duplicate" else name + " unexpected\n"
    )
    with pytest.raises(ValueError, match="require exactly one"):
        check.validate_configuration(effective)


# The host daemon only prints configuration: include expansion must reveal a forbidden nested directive.
def test_real_chronyd_expands_includes(tmp_path):
    executable = shutil.which("chronyd")
    if executable is None:
        pytest.skip("host chronyd is not installed")
    included = tmp_path / "included.conf"
    included.write_text("# initially empty\n")
    configuration = tmp_path / "chrony.conf"
    configuration.write_text(CONFIG + f"include {included}\n")
    assert "maxslewrate 3500" in check.check_configuration(configuration, chronyd=executable)
    included.write_text("makestep 1 3\n")
    with pytest.raises(ValueError, match="forbidden"):
        check.check_configuration(configuration, chronyd=executable)


# Run the installed-style CLI against the real host parser, without touching either clock or any service.
def test_prestart_cli_exit_status(tmp_path):
    if not Path("/usr/sbin/chronyd").exists():
        pytest.skip("pilot chronyd path is not present on this host")
    configuration = tmp_path / "chrony.conf"
    for body, expected in ((CONFIG, 0), (CONFIG + "rtcsync\n", 1)):
        configuration.write_text(body)
        result = subprocess.run(
            [sys.executable, str(Path(check.__file__)), str(configuration)],
            capture_output=True, text=True, timeout=15,
        )
        assert result.returncode == expected, result.stderr
        assert ("Chrony policy check failed" in result.stderr) == bool(expected)


@pytest.mark.parametrize("error", [FileNotFoundError("missing"), subprocess.TimeoutExpired("chronyd", 10), subprocess.CalledProcessError(1, "chronyd")])
def test_prestart_reports_execution_failure(monkeypatch, capsys, error):
    def failed(argv, **kwargs):
        assert argv == ["/usr/sbin/chronyd", "-p", "-f", "/etc/chrony/chrony.conf"]
        assert kwargs["timeout"] == 10 and kwargs["check"]
        raise error

    monkeypatch.setattr(check.subprocess, "run", failed)
    assert check.main(["/etc/chrony/chrony.conf"]) == 1
    assert "Chrony policy check failed" in capsys.readouterr().err


def service_command(argv, *, ignore="no"):
    return "{ path=" + argv[0] + " ; argv[]=" + " ".join(argv) + " ; ignore_errors=" + ignore + " ; start_time=[n/a] ; stop_time=[n/a] ; pid=0 ; code=(null) ; status=0/0 }"


def configure_audit(monkeypatch, *, launch=LAUNCH, process=LAUNCH, precheck=PRECHECK, ignore="no", active="active", competing=""):
    values = {
        "ExecStart": service_command(launch),
        "ExecStartPre": service_command(precheck, ignore=ignore),
        "ExecStartEx": service_command(launch).replace("ignore_errors=no", "flags=no-setuid"),
        "User": "_chrony",
        "MainPID": "123", "ActiveState": active, "Type": "forking", "Restart": "on-failure",
    }

    def command(*argv, **kwargs):
        assert argv[0] == "systemctl"
        if argv[1] == "show":
            output = "\n".join(name + "=" + value for name, value in values.items()) + "\n"
        elif argv[1] == "list-unit-files":
            output = competing
        else:
            assert argv[1] == "list-units"
            output = ""
        return subprocess.CompletedProcess(argv, 0, output, "")

    def process_arguments(pid):
        assert pid == 123
        return process

    def configuration(path):
        assert path == "/etc/chrony/chrony.conf"
        check.validate_configuration(CONFIG)
        return CONFIG

    monkeypatch.setattr(fixture, "command", command)
    monkeypatch.setattr(fixture, "chrony_process_arguments", process_arguments)
    monkeypatch.setattr(fixture, "check_configuration", configuration)
    return values


def test_documented_launch_passes_and_retains_arguments(tmp_path, monkeypatch):
    configure_audit(monkeypatch)
    fixture.test_deployment_time_writer_audit(tmp_path)
    assert json.loads((tmp_path / "writer-audit.json").read_text())["process_arguments"] == LAUNCH


@pytest.mark.parametrize("flags", ["", "ignore-failure", "privileged", "no-setuid ignore-failure"])
def test_audit_rejects_changed_launch_privilege_even_with_matching_arguments(tmp_path, monkeypatch, flags):
    values = configure_audit(monkeypatch)
    values["ExecStartEx"] = values["ExecStartEx"].replace("flags=no-setuid", "flags=" + flags)
    with pytest.raises(AssertionError, match="privileged launch"):
        fixture.test_deployment_time_writer_audit(tmp_path)


def test_shipped_dropin_preserves_vendor_privileged_launch():
    dropin = Path(__file__).resolve().parents[2] / "hardware/ds3231/chrony-runtime.conf"
    launch = [line for line in dropin.read_text().splitlines() if line.startswith("ExecStart=")]
    assert launch == ["ExecStart=", "ExecStart=!" + " ".join(LAUNCH)]


# F-002: a safe default file cannot hide a different configured or actually running launch.
@pytest.mark.parametrize("which", ["launch", "process", "precheck"])
def test_audit_rejects_different_configuration(tmp_path, monkeypatch, which):
    different = list(PRECHECK if which == "precheck" else LAUNCH)
    different[-1] = "/etc/chrony/unsafe.conf"
    configure_audit(monkeypatch, **{which: different})
    with pytest.raises(AssertionError):
        fixture.test_deployment_time_writer_audit(tmp_path)


@pytest.mark.parametrize("option", ["-s", "-q", "-x", "makestep 1 3"])
def test_audit_rejects_unapproved_launch_options(tmp_path, monkeypatch, option):
    configure_audit(monkeypatch, launch=LAUNCH + [option], process=LAUNCH + [option])
    with pytest.raises(AssertionError):
        fixture.test_deployment_time_writer_audit(tmp_path)


@pytest.mark.parametrize("invalid", ["ignored-check", "missing-check", "extra-check", "inactive", "competing"])
def test_audit_requires_enforced_check_and_sole_writer(tmp_path, monkeypatch, invalid):
    values = configure_audit(
        monkeypatch, ignore="yes" if invalid == "ignored-check" else "no",
        active="inactive" if invalid == "inactive" else "active",
        competing="ntp.service enabled enabled\n" if invalid == "competing" else "",
    )
    if invalid == "missing-check":
        values["ExecStartPre"] = ""
    if invalid == "extra-check":
        values["ExecStartPre"] += " " + service_command(["/bin/true"])
    with pytest.raises(AssertionError):
        fixture.test_deployment_time_writer_audit(tmp_path)


def test_reply_socket_permissions_use_one_post_start_command():
    root = Path(__file__).resolve().parents[2]
    dropin = (root / 'hardware/ds3231/chrony-runtime.conf').read_text()
    commands = [line for line in dropin.splitlines() if line.startswith('ExecStartPost=')]
    # systemd resets RuntimeDirectory ownership/mode before every command.
    assert len(commands) == 1
    assert commands[0].startswith('ExecStartPost=+/bin/sh -c ')
    assert '/usr/bin/chgrp cura-receiver /run/chrony /run/chrony/chronyd.sock' in commands[0]
    assert '/usr/bin/chmod 01770 /run/chrony' in commands[0]
    assert '/usr/bin/chmod 0660 /run/chrony/chronyd.sock' in commands[0]
    assert 'RuntimeDirectoryMode=0700' in dropin
    unit = (root / 'deploy/systemd/cura-receiver.service').read_text()
    assert 'ReadWritePaths=/var/lib/cura-agrorum -/run/chrony' in unit
    assert 'After=local-fs.target cura-rtc-bootstrap.service chrony.service' in unit
    assert 'Requires=chrony.service' not in unit
