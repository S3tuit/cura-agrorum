from __future__ import annotations

from pathlib import Path

import pytest


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
