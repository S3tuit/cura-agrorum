from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from tests import conftest as receiver_test_config
from tests.hardware import conftest as hardware_test_config


RECEIVER_ROOT = Path(__file__).resolve().parents[2]


def _run_hardware_probe(
    tmp_path: Path,
    test_source: str,
    *pytest_args: str,
) -> tuple[subprocess.CompletedProcess[str], Path]:
    project_root = tmp_path / "receiver_hardware_harness"
    hardware_root = project_root / "hardware"
    hardware_root.mkdir(parents=True)
    (project_root / "raspberry_pi_model").write_text(
        "Raspberry Pi Review Probe\n",
        encoding="utf-8",
    )
    (project_root / "pytest.ini").write_text(
        """\
[pytest]
addopts = --strict-config --strict-markers
markers =
    hardware: requires receiver hardware
    destructive: may change target state
""",
        encoding="utf-8",
    )
    (project_root / "conftest.py").write_text(
        f"""\
from pathlib import Path

from tests import conftest as receiver_config
from tests.hardware import conftest as hardware_config

PROJECT_ROOT = Path({str(project_root)!r})
receiver_config.TESTS_ROOT = PROJECT_ROOT
receiver_config.HOST_ROOT = PROJECT_ROOT / "host"
receiver_config.HARDWARE_ROOT = PROJECT_ROOT / "hardware"
hardware_config.RASPBERRY_PI_MODEL = PROJECT_ROOT / "raspberry_pi_model"

pytest_addoption = receiver_config.pytest_addoption
pytest_configure = receiver_config.pytest_configure
pytest_collection_modifyitems = receiver_config.pytest_collection_modifyitems
pytest_collection_finish = hardware_config.pytest_collection_finish
pytest_runtest_makereport = hardware_config.pytest_runtest_makereport
""",
        encoding="utf-8",
    )
    test_path = hardware_root / "test_probe.py"
    test_path.write_text(test_source, encoding="utf-8")

    environment = os.environ.copy()
    environment["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    result = subprocess.run(
        (
            sys.executable,
            "-m",
            "pytest",
            "-c",
            str(project_root / "pytest.ini"),
            str(test_path),
            *pytest_args,
        ),
        cwd=RECEIVER_ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=20,
    )
    return result, project_root


# Rejects an unconfirmed destructive item before a session fixture can run.
def test_destructive_authorization_precedes_session_fixture_setup(
    tmp_path: Path,
) -> None:
    result, project_root = _run_hardware_probe(
        tmp_path,
        """\
from pathlib import Path

import pytest


pytestmark = [pytest.mark.hardware, pytest.mark.destructive]


@pytest.fixture(scope="session", autouse=True)
def destructive_session_fixture() -> None:
    (Path(__file__).resolve().parents[1] / "fixture-ran").write_text("unsafe")


def test_destructive_probe() -> None:
    pass
""",
        "--receiver-hardware",
    )

    output = result.stdout + result.stderr
    assert result.returncode == pytest.ExitCode.USAGE_ERROR
    assert "require --confirm-receiver-destructive" in output
    assert not (project_root / "fixture-ran").exists()


# Makes a selected hardware fixture's missing-component skip fail the run.
def test_selected_hardware_fixture_skip_is_a_failure(tmp_path: Path) -> None:
    result, _project_root = _run_hardware_probe(
        tmp_path,
        """\
import pytest


pytestmark = pytest.mark.hardware


@pytest.fixture
def missing_ds3231() -> None:
    pytest.skip("DS3231 fixture is missing")


def test_ds3231_probe(missing_ds3231: None) -> None:
    pass
""",
        "--receiver-hardware",
    )

    output = result.stdout + result.stderr
    assert result.returncode == pytest.ExitCode.TESTS_FAILED
    assert "selected receiver hardware test skipped during setup" in output
    assert "DS3231 fixture is missing" in output
    assert "1 error" in output


# Rejects every invalid shared destructive-root shape before fixture use.
@pytest.mark.parametrize(
    ("case", "message"),
    (
        ("missing-option", "require --receiver-test-root"),
        ("relative", "must be absolute"),
        ("missing-path", "invalid --receiver-test-root"),
        ("filesystem-root", "must be a dedicated directory"),
        ("plain-file", "must be a dedicated directory"),
        ("missing-sentinel", "missing receiver test-root sentinel"),
        ("invalid-sentinel", "sentinel content is invalid"),
    ),
)
def test_destructive_root_validator_rejects_unsafe_inputs(
    tmp_path: Path,
    case: str,
    message: str,
) -> None:
    configured_root: str | None
    if case == "missing-option":
        configured_root = None
    elif case == "relative":
        configured_root = "relative/test-root"
    elif case == "missing-path":
        configured_root = str(tmp_path / "absent")
    elif case == "filesystem-root":
        configured_root = "/"
    elif case == "plain-file":
        plain_file = tmp_path / "plain-file"
        plain_file.write_text("not a directory", encoding="utf-8")
        configured_root = str(plain_file)
    else:
        test_root = tmp_path / case
        test_root.mkdir()
        if case == "invalid-sentinel":
            (test_root / hardware_test_config.TEST_ROOT_SENTINEL).write_text(
                "wrong marker\n",
                encoding="utf-8",
            )
        configured_root = str(test_root)

    with pytest.raises(pytest.UsageError, match=message):
        hardware_test_config._validated_destructive_test_root(configured_root)


# Accepts only an existing absolute directory with the exact sentinel content.
def test_destructive_root_validator_accepts_marked_directory(tmp_path: Path) -> None:
    test_root = tmp_path / "destructive-root"
    test_root.mkdir()
    (test_root / hardware_test_config.TEST_ROOT_SENTINEL).write_text(
        hardware_test_config.TEST_ROOT_SENTINEL_CONTENT,
        encoding="utf-8",
    )

    assert hardware_test_config._validated_destructive_test_root(str(test_root)) == (
        test_root.resolve()
    )


# Rejects every requested worker count that would parallelize hardware cases.
@pytest.mark.parametrize("worker_count", (1, 2, "auto"))
def test_hardware_configuration_rejects_parallel_execution(
    worker_count: int | str,
) -> None:
    config = SimpleNamespace(
        option=SimpleNamespace(numprocesses=worker_count),
        getoption=lambda name: name == "receiver_hardware",
    )

    with pytest.raises(pytest.UsageError, match="must run serially"):
        receiver_test_config.pytest_configure(config)  # type: ignore[arg-type]
