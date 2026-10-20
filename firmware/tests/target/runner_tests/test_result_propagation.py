"""A dynamically acquired DUT must propagate Unity failures to pytest."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from pytest_embedded.unity import TestSuite as UnitySuite


path = Path(__file__).resolve().parents[1] / "pytest_node_persistence.py"
spec = importlib.util.spec_from_file_location("integration_result_runner", path)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.mark.parametrize("outcomes", [
    ("PASS", "PASS"), ("PASS", "FAIL"), ("PASS", "IGNORE"), ("PASS", None),
])
def test_entry_point_requires_every_selected_case_to_pass(monkeypatch, outcomes):
    menu = [
        runner.UnittestMenuCase(index=index, name=f"case {index}", type="normal",
                               keywords=[], groups=[], attributes={}, subcases=[])
        for index in (1, 2)
    ]
    suite = UnitySuite("device")
    executed = []

    def run_case(name, *, reset, timeout):
        assert reset and timeout > 0
        result = outcomes[len(executed)]
        executed.append(name)
        if result is not None:
            suite.add_unity_test_cases(f"device.c:1:{name}:{result}\n")

    dut = SimpleNamespace(testsuite=suite, run_single_board_case=run_case)
    request = SimpleNamespace(
        config=SimpleNamespace(getoption=lambda name: "/dev/ttyACM0"),
        getfixturevalue=lambda name: dut,
    )
    monkeypatch.setattr(runner, "usb_identity", lambda port: None)
    monkeypatch.setattr(runner, "attach_usb_console", lambda dut: None)
    monkeypatch.setattr(runner, "_load_test_menu", lambda dut, timeout: menu)
    monkeypatch.setenv("CURAG_HARDWARE_TEST_SET", "all")

    if outcomes == ("PASS", "PASS"):
        runner.test_node_persistence_on_device(request)
    else:
        with pytest.raises(AssertionError, match="Unity"):
            runner.test_node_persistence_on_device(request)
    assert executed == [case.name for case in menu]
