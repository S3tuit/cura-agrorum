from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

import pytest


RASPBERRY_PI_MODEL = Path("/proc/device-tree/model")
TEST_ROOT_SENTINEL = ".cura-receiver-test-root"
TEST_ROOT_SENTINEL_CONTENT = "CURA AGRORUM RECEIVER TEST ROOT\n"


@pytest.fixture(scope="session", autouse=True)
def require_explicit_receiver_hardware_run(
    pytestconfig: pytest.Config,
) -> None:
    if not pytestconfig.getoption("receiver_hardware"):
        pytest.fail(
            "receiver hardware tests require the --receiver-hardware option",
            pytrace=False,
        )

    try:
        model = RASPBERRY_PI_MODEL.read_text(encoding="utf-8").rstrip("\n\x00")
    except OSError as error:
        pytest.fail(
            f"receiver hardware tests require a target Raspberry Pi: {error}",
            pytrace=False,
        )
    if "Raspberry Pi" not in model:
        pytest.fail(
            f"receiver hardware tests require a target Raspberry Pi, found {model!r}",
            pytrace=False,
        )


@pytest.fixture(autouse=True)
def require_destructive_test_interlocks(
    request: pytest.FixtureRequest,
    pytestconfig: pytest.Config,
) -> Iterator[None]:
    if request.node.get_closest_marker("destructive") is None:
        yield
        return

    if not pytestconfig.getoption("confirm_receiver_destructive"):
        pytest.fail(
            "destructive receiver tests require --confirm-receiver-destructive",
            pytrace=False,
        )

    configured_root = pytestconfig.getoption("receiver_test_root")
    if configured_root is None:
        pytest.fail(
            "destructive receiver tests require --receiver-test-root",
            pytrace=False,
        )

    supplied_root = Path(configured_root)
    if not supplied_root.is_absolute():
        pytest.fail("--receiver-test-root must be absolute", pytrace=False)
    try:
        test_root = supplied_root.resolve(strict=True)
    except OSError as error:
        pytest.fail(f"invalid --receiver-test-root: {error}", pytrace=False)
    if not test_root.is_dir() or test_root == Path("/"):
        pytest.fail(
            "--receiver-test-root must be a dedicated directory",
            pytrace=False,
        )

    sentinel = test_root / TEST_ROOT_SENTINEL
    try:
        sentinel_content = sentinel.read_text(encoding="utf-8")
    except OSError as error:
        pytest.fail(
            f"missing receiver test-root sentinel: {error}",
            pytrace=False,
        )
    if sentinel_content != TEST_ROOT_SENTINEL_CONTENT:
        pytest.fail("receiver test-root sentinel content is invalid", pytrace=False)

    yield
