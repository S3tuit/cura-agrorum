from __future__ import annotations

from pathlib import Path

import pytest


TESTS_ROOT = Path(__file__).resolve().parent
HOST_ROOT = TESTS_ROOT / "host"
HARDWARE_ROOT = TESTS_ROOT / "hardware"


def pytest_addoption(parser: pytest.Parser) -> None:
    group = parser.getgroup("receiver hardware")
    group.addoption(
        "--receiver-hardware",
        action="store_true",
        help="confirm this pytest invocation intentionally targets receiver hardware",
    )
    group.addoption(
        "--confirm-receiver-destructive",
        action="store_true",
        help="confirm this invocation may run destructive receiver hardware tests",
    )
    group.addoption(
        "--receiver-test-root",
        metavar="PATH",
        help="dedicated marked root for destructive receiver test files and artifacts",
    )


def pytest_configure(config: pytest.Config) -> None:
    if not config.getoption("receiver_hardware"):
        return

    worker_count = getattr(config.option, "numprocesses", None)
    if worker_count not in (None, 0, "0"):
        raise pytest.UsageError("receiver hardware tests must run serially")


def pytest_collection_modifyitems(items: list[pytest.Item]) -> None:
    violations: list[str] = []
    for item in items:
        path = Path(str(item.path)).resolve()
        marker_names = {marker.name for marker in item.iter_markers()}
        in_host = path.is_relative_to(HOST_ROOT)
        in_hardware = path.is_relative_to(HARDWARE_ROOT)

        if not in_host and not in_hardware:
            violations.append(
                f"{item.nodeid}: receiver test is outside host/ or hardware/"
            )
            continue
        if in_hardware and "hardware" not in marker_names:
            violations.append(
                f"{item.nodeid}: hardware test lacks the hardware marker"
            )
        if in_host and marker_names.intersection(
            {"hardware", "destructive", "rf_peer"}
        ):
            violations.append(
                f"{item.nodeid}: host test carries a hardware-only marker"
            )
        if "destructive" in marker_names and "hardware" not in marker_names:
            violations.append(
                f"{item.nodeid}: destructive test is not marked hardware"
            )
        if "rf_peer" in marker_names and "hardware" not in marker_names:
            violations.append(f"{item.nodeid}: RF-peer test is not marked hardware")

    if violations:
        raise pytest.UsageError("\n".join(violations))
