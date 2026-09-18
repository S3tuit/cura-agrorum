"""Installed entry points must share isolated paths and fail before device access."""

import os
from pathlib import Path
import shlex
import sqlite3
import subprocess
import sys

import pytest

from cura_receiver.application_environment import settings_from_environment
from cura_receiver.application_settings import ApplicationSettings


def isolated_environment(root):
    return {
        "CURA_RECEIVER_CONFIGURATION": str(root / "group.json"),
        "CURA_RECEIVER_DATABASE": str(root / "data" / "readings.sqlite3"),
        "SQLITE_TMPDIR": str(root / "data" / "tmp"),
        "CURA_RECEIVER_TEST_ROOT": str(root),
    }


def test_default_and_isolated_profiles_keep_policy_unchanged(tmp_path):
    assert settings_from_environment({}) == ApplicationSettings()
    env = isolated_environment(tmp_path)
    settings = settings_from_environment(env)
    assert settings.database_path == tmp_path / "data/readings.sqlite3"
    assert settings.configuration_path == tmp_path / "group.json"
    assert settings.sqlite_temporary_directory == tmp_path / "data/tmp"
    assert settings.time_policy == ApplicationSettings().time_policy
    assert settings.radio == ApplicationSettings().radio
    assert env == isolated_environment(tmp_path)


def test_unit_does_not_mask_partial_environment_file_overrides():
    unit = Path(__file__).resolve().parents[2] / "deploy/systemd/cura-receiver.service"
    environment = {}
    for line in unit.read_text().splitlines():
        if line.startswith("Environment="):
            for assignment in shlex.split(line.removeprefix("Environment=")):
                name, value = assignment.split("=", 1)
                environment[name] = value
    environment["CURA_RECEIVER_DATABASE"] = "/tmp/isolated/readings.sqlite3"
    with pytest.raises(ValueError, match="supplied together"):
        settings_from_environment(environment)


@pytest.mark.parametrize("missing", ["CURA_RECEIVER_CONFIGURATION", "CURA_RECEIVER_DATABASE", "SQLITE_TMPDIR"])
def test_partial_override_never_falls_back_to_production(tmp_path, missing):
    env = isolated_environment(tmp_path)
    del env[missing]
    with pytest.raises(ValueError):
        settings_from_environment(env)


@pytest.mark.parametrize("name", ["CURA_RECEIVER_CONFIGURATION", "CURA_RECEIVER_DATABASE", "SQLITE_TMPDIR", "CURA_RECEIVER_TEST_ROOT"])
@pytest.mark.parametrize("value", ["", "relative", "/tmp/../production", "/tmp/invalid\0name"])
def test_invalid_path_rejected_without_echoing_values(tmp_path, name, value):
    env = isolated_environment(tmp_path)
    env[name] = value
    with pytest.raises(ValueError) as error:
        settings_from_environment(env)
    assert value not in str(error.value) if value else True


@pytest.mark.parametrize("root", ["/", "/etc", "/etc/cura-agrorum", "/etc/cura-agrorum/test", "/var/lib", "/var/lib/cura-agrorum", "//var/lib/cura-agrorum/test"])
def test_test_root_cannot_cover_or_enter_production(root):
    with pytest.raises(ValueError):
        settings_from_environment(isolated_environment(Path(root)))


@pytest.mark.parametrize("name", ["CURA_RECEIVER_CONFIGURATION", "CURA_RECEIVER_DATABASE", "SQLITE_TMPDIR"])
def test_outside_or_root_itself_rejected(tmp_path, name):
    for path in (tmp_path, tmp_path.parent / "another-epoch"):
        env = isolated_environment(tmp_path)
        env[name] = str(path)
        with pytest.raises(ValueError):
            settings_from_environment(env)


def test_test_root_requires_explicit_paths_and_distinct_files(tmp_path):
    with pytest.raises(ValueError):
        settings_from_environment({"CURA_RECEIVER_TEST_ROOT": str(tmp_path)})
    env = isolated_environment(tmp_path)
    env["CURA_RECEIVER_DATABASE"] = env["CURA_RECEIVER_CONFIGURATION"]
    with pytest.raises(ValueError):
        settings_from_environment(env)


def process_environment(paths):
    env = dict(os.environ)
    for key in isolated_environment(Path("/unused")):
        env.pop(key, None)
    env.update(paths)
    repo = Path(__file__).resolve().parents[3]
    env["PYTHONPATH"] = os.pathsep.join((str(repo / "receiver"), str(repo / "protocol/protocol-v2-lora/python")))
    return env


def test_real_preflight_uses_isolated_database_and_temp_directory(tmp_path):
    paths = isolated_environment(tmp_path)
    Path(paths["SQLITE_TMPDIR"]).mkdir(parents=True)
    with sqlite3.connect(paths["CURA_RECEIVER_DATABASE"]) as connection:
        connection.execute("CREATE TABLE isolated_marker(value)")
    result = subprocess.run([sys.executable, "-m", "cura_receiver.storage_preflight"],
                            env=process_environment(paths), capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    assert not list(Path(paths["SQLITE_TMPDIR"]).iterdir())


@pytest.mark.parametrize("module,args", [
    ("cura_receiver.storage_preflight", []),
    ("cura_receiver", ["--rtc-helper-sha256", "0" * 64, "--rtc-kernel-bound-us", "1"]),
])
def test_entry_points_reject_partial_paths_before_storage_or_hardware(tmp_path, module, args):
    secret_like_path = str(tmp_path / "do-not-echo-this-input")
    result = subprocess.run([sys.executable, "-m", module, *args],
                            env=process_environment({"CURA_RECEIVER_DATABASE": secret_like_path}),
                            capture_output=True, text=True, timeout=10)
    assert result.returncode != 0
    assert secret_like_path not in result.stdout + result.stderr
    assert not list(tmp_path.iterdir())
