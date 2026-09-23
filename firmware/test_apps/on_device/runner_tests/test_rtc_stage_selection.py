"""Exercise repeated RTC orchestration when Unity's submenu arrives late."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pexpect
import pytest


_RUNNER_PATH = Path(__file__).resolve().parents[1] / "pytest_node_persistence.py"
_SPEC = importlib.util.spec_from_file_location("rtc_hardware_runner", _RUNNER_PATH)
runner = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(runner)

_CASE = runner.UnittestMenuCase(
    index=37,
    name="RTC repeats exactly 20 retained deep-sleep round trips",
    type="multi_stage",
    keywords=[],
    groups=["node_rtc", "slow"],
    attributes={},
    subcases=[
        {"index": 1, "name": "rtc_repeated_round_trip_stage_1"},
        {"index": 2, "name": "rtc_repeated_round_trip_stage_2"},
    ],
)
_SUBMENU = (
    '\t(1)\t"rtc_repeated_round_trip_stage_1"\r\n'
    '\t(2)\t"rtc_repeated_round_trip_stage_2"\r\n'
)
_RESULT = "1 Tests 0 Failures 0 Ignored"


class DelayedSubmenuDut:
    """Model Unity's RX flush at a UART boundary, without RTC or sleep fakes.

    At the chosen selection, only Running is initially available. Completing
    the submenu discards any selector already sent, matching unity_gets().
    A later selector advances the device; a lost one produces no next boot
    or final result. Other selections model a submenu already fully received.
    """

    confirm_write = runner.IdfDut.confirm_write

    def __init__(self, delayed_selection):
        self.delayed_selection = delayed_selection
        self.serial = SimpleNamespace(hard_reset=self.hard_reset)
        self.reset_count = 0
        self.selections = []
        self.discarded = []
        self.result_reads = 0

    def hard_reset(self):
        self.reset_count += 1
        self.state = "main_menu"
        self.output = runner.UNITY_READY_PROMPT

    def write(self, value):
        if self.state == "main_menu":
            assert value == str(_CASE.index)
            self.output += f"Running {_CASE.name}...\r\n"
            if len(self.selections) == self.delayed_selection:
                self.state = "printing_submenu"
            else:
                self.output += _SUBMENU
                self.state = "stage_selection"
        elif self.state == "printing_submenu":
            self.discarded.append(value)
        else:
            assert self.state == "stage_selection"
            assert value == ("1" if not self.selections else "2")
            self.selections.append(value)
            if len(self.selections) <= 20:
                self.output += runner.UNITY_READY_PROMPT
                self.state = "main_menu"
            else:
                self.output += _RESULT
                self.state = "finished"

    def expect_exact(self, pattern, *, timeout):
        assert timeout > 0
        if pattern not in self.output and self.state == "printing_submenu":
            self.output += _SUBMENU
            self.state = "stage_selection"
        if pattern not in self.output:
            raise pexpect.TIMEOUT(
                f"No {pattern!r}; selectors discarded by Unity: {self.discarded}"
            )
        before, self.output = self.output.split(pattern, 1)
        return before.encode()

    def expect_unity_test_output(self, *, timeout):
        self.expect_exact(_RESULT, timeout=timeout)
        self.result_reads += 1


@pytest.mark.parametrize("delayed_selection", [None, *range(21)])
def test_repeated_rtc_completes_with_delayed_submenu(delayed_selection):
    dut = DelayedSubmenuDut(delayed_selection)

    runner._run_repeated_rtc_round_trip_case(dut, _CASE, timeout=300)

    assert dut.reset_count == 1
    assert dut.selections == ["1"] + ["2"] * 20
    assert dut.discarded == []
    assert dut.result_reads == 1
    assert dut.state == "finished"
