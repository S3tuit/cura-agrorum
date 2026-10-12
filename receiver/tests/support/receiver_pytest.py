from __future__ import annotations

from pathlib import Path

import pytest

RECEIVER_ROOT = Path(__file__).resolve().parents[2]
TESTS_ROOT = RECEIVER_ROOT / "tests"
HOST_ROOT = TESTS_ROOT / "host"
HARDWARE_ROOT = TESTS_ROOT / "hardware"
COMPONENT_HARDWARE_ROOT = RECEIVER_ROOT / "hardware"


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
    group.addoption("--airtime-reboot-phase", choices=("prepare", "verify"))
    group.addoption("--airtime-reboot-mode", choices=("trusted", "unavailable"))
    group.addoption("--airtime-reboot-session", metavar="PATH")
    group.addoption("--receiver-radio-fixture", metavar="PATH")
    group.addoption("--receiver-radio-evidence", metavar="PATH")
    group.addoption("--receiver-storage-user", default="cura", metavar="USER",
                    help="unprivileged account for root-supervised storage faults (default: cura)")
    group.addoption("--receiver-time-user", default="cura", metavar="USER",
                    help="unprivileged account for nominal/step time components (default: cura)")


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
        in_component = path.is_relative_to(COMPONENT_HARDWARE_ROOT) and "tests" in path.relative_to(COMPONENT_HARDWARE_ROOT).parts[:-1]
        in_hardware = path.is_relative_to(HARDWARE_ROOT) or in_component

        if not in_host and not in_hardware:
            violations.append(
                f"{item.nodeid}: receiver test is outside host/, hardware/, or hardware component tests/"
            )
            continue
        if in_hardware and "hardware" not in marker_names:
            violations.append(f"{item.nodeid}: hardware test lacks the hardware marker")
        if in_host and marker_names.intersection(
            {"hardware", "destructive", "rf_peer"}
        ):
            violations.append(
                f"{item.nodeid}: host test carries a hardware-only marker"
            )
        if "destructive" in marker_names and "hardware" not in marker_names:
            violations.append(f"{item.nodeid}: destructive test is not marked hardware")
        if "rf_peer" in marker_names and "hardware" not in marker_names:
            violations.append(f"{item.nodeid}: RF-peer test is not marked hardware")

    if violations:
        raise pytest.UsageError("\n".join(violations))


RASPBERRY_PI_MODEL = Path("/proc/device-tree/model")
TEST_ROOT_SENTINEL = ".cura-receiver-test-root"
TEST_ROOT_SENTINEL_CONTENT = "CURA AGRORUM RECEIVER TEST ROOT\n"


def _require_target_raspberry_pi() -> None:
    """Reject hardware execution anywhere other than a target Raspberry Pi."""

    try:
        model = RASPBERRY_PI_MODEL.read_text(encoding="utf-8").rstrip("\n\x00")
    except OSError as error:
        raise pytest.UsageError(
            f"receiver hardware tests require a target Raspberry Pi: {error}"
        ) from error
    if "Raspberry Pi" not in model:
        raise pytest.UsageError(
            f"receiver hardware tests require a target Raspberry Pi, found {model!r}"
        )


def _validated_destructive_test_root(configured_root: str | None) -> Path:
    """Resolve and validate the shared destructive-test root interlock."""

    if configured_root is None:
        raise pytest.UsageError(
            "destructive receiver tests require --receiver-test-root"
        )

    supplied_root = Path(configured_root)
    if not supplied_root.is_absolute():
        raise pytest.UsageError("--receiver-test-root must be absolute")
    try:
        test_root = supplied_root.resolve(strict=True)
    except OSError as error:
        raise pytest.UsageError(f"invalid --receiver-test-root: {error}") from error
    if not test_root.is_dir() or test_root == Path("/"):
        raise pytest.UsageError(
            "--receiver-test-root must be a dedicated directory"
        )

    sentinel = test_root / TEST_ROOT_SENTINEL
    try:
        sentinel_content = sentinel.read_text(encoding="utf-8")
    except OSError as error:
        raise pytest.UsageError(
            f"missing receiver test-root sentinel: {error}"
        ) from error
    if sentinel_content != TEST_ROOT_SENTINEL_CONTENT:
        raise pytest.UsageError("receiver test-root sentinel content is invalid")
    return test_root


def pytest_collection_finish(session: pytest.Session) -> None:
    """Authorize selected hardware items before pytest sets up any fixture."""

    hardware_items = [
        item
        for item in session.items
        if item.get_closest_marker("hardware") is not None
    ]
    if not hardware_items:
        return

    config = session.config
    if not config.getoption("receiver_hardware"):
        raise pytest.UsageError(
            "receiver hardware tests require the --receiver-hardware option"
        )

    radio_items = [item for item in hardware_items if item.get_closest_marker("radio")]
    if radio_items:
        from tests.hardware.radio_fixture import validate_selection
        if any(item.get_closest_marker("radio_fault") for item in radio_items):
            raise pytest.UsageError("radio_fault gate tests are deferred; the manual carrier cannot run them")
        held_items = [item for item in radio_items if item.get_closest_marker("radio_busy_held")]
        if held_items and len(session.items) != 1:
            raise pytest.UsageError("radio_busy_held must run alone; power-off nominal restoration is required afterward")
        validate_selection(config, required_state="radio_busy_held" if held_items else "radio_nominal")

    if any(item.get_closest_marker("destructive") for item in hardware_items):
        if not config.getoption("confirm_receiver_destructive"):
            raise pytest.UsageError(
                "destructive receiver tests require --confirm-receiver-destructive"
            )
        _validated_destructive_test_root(config.getoption("receiver_test_root"))

    _require_target_raspberry_pi()


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(
    item: pytest.Item,
    call: pytest.CallInfo[None],
):
    """Turn a selected hardware runtime skip into an explicit failure."""

    outcome = yield
    report = outcome.get_result()
    if (
        report.skipped
        and not hasattr(report, "wasxfail")
        and item.get_closest_marker("hardware") is not None
    ):
        if isinstance(report.longrepr, tuple):
            skip_reason = str(report.longrepr[2])
        else:
            skip_reason = str(report.longrepr)
        report.outcome = "failed"
        report.longrepr = (
            f"{item.nodeid}: selected receiver hardware test skipped during "
            f"{report.when}: {skip_reason}. Select intentional suite variants "
            "through registered markers instead of runtime skips."
        )
