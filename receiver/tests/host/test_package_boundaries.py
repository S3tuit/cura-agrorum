from __future__ import annotations

import ast
from collections.abc import Iterator
from pathlib import Path


RECEIVER_ROOT = Path(__file__).resolve().parents[2]
PRODUCTION_ROOT = RECEIVER_ROOT / "cura_receiver"
LINUX_CLOCK_ADAPTER = Path("platform/linux_clocks.py")


def production_python_files() -> tuple[Path, ...]:
    return tuple(sorted(PRODUCTION_ROOT.rglob("*.py")))


def imported_modules(source: Path) -> Iterator[str]:
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                yield alias.name
        elif isinstance(node, ast.ImportFrom):
            if node.module is not None:
                yield node.module
            elif node.level:
                for alias in node.names:
                    yield alias.name


def is_test_module(module: str) -> bool:
    return (
        module == "tests"
        or module.startswith("tests.")
        or module == "receiver.tests"
        or module.startswith("receiver.tests.")
    )


# Prevents the deployed receiver package from depending on test-only support.
def test_production_package_never_imports_receiver_tests() -> None:
    violations = {
        source.relative_to(PRODUCTION_ROOT): tuple(
            module for module in imported_modules(source) if is_test_module(module)
        )
        for source in production_python_files()
    }

    assert {path: modules for path, modules in violations.items() if modules} == {}


# Confines direct access to Python's OS-clock module to the Linux clock adapter.
def test_only_linux_clock_adapter_imports_time_module() -> None:
    importers = {
        source.relative_to(PRODUCTION_ROOT)
        for source in production_python_files()
        if any(
            module == "time" or module.startswith("time.")
            for module in imported_modules(source)
        )
    }

    assert importers == {LINUX_CLOCK_ADAPTER}
