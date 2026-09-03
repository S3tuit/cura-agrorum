from __future__ import annotations

import ast
from collections.abc import Iterator
from pathlib import Path

import pytest


RECEIVER_ROOT = Path(__file__).resolve().parents[2]
PRODUCTION_ROOT = RECEIVER_ROOT / "cura_receiver"
LINUX_CLOCK_ADAPTER = Path("platform/linux_clocks.py")
DIRECT_DATETIME_CLOCK_READERS = frozenset(
    {
        "datetime.date.today",
        "datetime.datetime.now",
        "datetime.datetime.today",
        "datetime.datetime.utcnow",
    }
)


def production_python_files() -> tuple[Path, ...]:
    return tuple(sorted(PRODUCTION_ROOT.rglob("*.py")))


def _import_bindings(tree: ast.AST) -> dict[str, str]:
    bindings = {"__import__": "builtins.__import__"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                bound_name = alias.asname or alias.name.partition(".")[0]
                bindings[bound_name] = (
                    alias.name if alias.asname else alias.name.partition(".")[0]
                )
        elif isinstance(node, ast.ImportFrom) and node.module is not None:
            for alias in node.names:
                if alias.name != "*":
                    bindings[alias.asname or alias.name] = (
                        f"{node.module}.{alias.name}"
                    )
    return bindings


def _constant_string(node: ast.AST) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = _constant_string(node.left)
        right = _constant_string(node.right)
        if left is not None and right is not None:
            return left + right
    return None


def _resolved_symbol(node: ast.AST, bindings: dict[str, str]) -> str | None:
    if isinstance(node, ast.Name):
        return bindings.get(node.id, node.id)
    if isinstance(node, ast.Attribute):
        value = _resolved_symbol(node.value, bindings)
        return None if value is None else f"{value}.{node.attr}"
    if isinstance(node, ast.Call):
        function = _resolved_symbol(node.func, bindings)
        if function in {"importlib.import_module", "builtins.__import__"} and node.args:
            return _constant_string(node.args[0])
    return None


def imported_modules(source: Path) -> Iterator[str]:
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    bindings = _import_bindings(tree)
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
        elif isinstance(node, ast.Call):
            function = _resolved_symbol(node.func, bindings)
            if function in {"importlib.import_module", "builtins.__import__"}:
                if node.args and (module := _constant_string(node.args[0])) is not None:
                    yield module


def direct_datetime_clock_references(source: Path) -> tuple[str, ...]:
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    bindings = _import_bindings(tree)
    return tuple(
        sorted(
            {
                symbol
                for node in ast.walk(tree)
                if isinstance(node, ast.Attribute)
                if (symbol := _resolved_symbol(node, bindings))
                in DIRECT_DATETIME_CLOCK_READERS
            }
        )
    )


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


# Recognizes ordinary literal dynamic imports of either receiver test namespace.
@pytest.mark.parametrize(
    "source_text",
    (
        'import importlib\nimportlib.import_module("tests.support.fakes")\n',
        'import importlib as loader\nloader.import_module("receiver.tests")\n',
        'from importlib import import_module as load\nload("tests" + ".support")\n',
        '__import__("receiver.tests.support")\n',
    ),
)
def test_import_scanner_recognizes_literal_dynamic_test_imports(
    tmp_path: Path,
    source_text: str,
) -> None:
    source = tmp_path / "dynamic_import.py"
    source.write_text(source_text, encoding="utf-8")

    assert any(is_test_module(module) for module in imported_modules(source))


# Confines standard-library OS-clock reads to the Linux clock adapter.
def test_only_linux_clock_adapter_accesses_os_clocks() -> None:
    importers = {
        source.relative_to(PRODUCTION_ROOT)
        for source in production_python_files()
        if any(
            module == "time" or module.startswith("time.")
            for module in imported_modules(source)
        )
    }

    assert importers == {LINUX_CLOCK_ADAPTER}
    alternative_readers = {
        source.relative_to(PRODUCTION_ROOT): direct_datetime_clock_references(source)
        for source in production_python_files()
        if source.relative_to(PRODUCTION_ROOT) != LINUX_CLOCK_ADAPTER
    }
    assert {
        path: readers for path, readers in alternative_readers.items() if readers
    } == {}


# Recognizes aliased datetime APIs that read the host's realtime clock.
@pytest.mark.parametrize(
    ("source_text", "expected_reader"),
    (
        (
            "import datetime\ndatetime.datetime.now(datetime.UTC)\n",
            "datetime.datetime.now",
        ),
        (
            "import datetime as dt\ndt.datetime.today()\n",
            "datetime.datetime.today",
        ),
        (
            "from datetime import datetime as DateTime\nDateTime.utcnow()\n",
            "datetime.datetime.utcnow",
        ),
        (
            "from datetime import date\ndate.today()\n",
            "datetime.date.today",
        ),
    ),
)
def test_clock_boundary_scanner_recognizes_datetime_realtime_reads(
    tmp_path: Path,
    source_text: str,
    expected_reader: str,
) -> None:
    source = tmp_path / "datetime_read.py"
    source.write_text(source_text, encoding="utf-8")

    assert direct_datetime_clock_references(source) == (expected_reader,)
