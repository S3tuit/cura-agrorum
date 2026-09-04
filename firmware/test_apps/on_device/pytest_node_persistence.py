import os
import time

from pytest_embedded_idf.dut import IdfDut, UnittestMenuCase


PLATFORM_DEEP_SLEEP_CASE = (
    "platform timer deep sleep lasts one minute and reports deep-sleep reset"
)
PLATFORM_DEEP_SLEEP_START_MARKER = "CURAG_PLATFORM_DEEP_SLEEP_START"
PLATFORM_DEEP_SLEEP_END_MARKER = "CURAG_PLATFORM_DEEP_SLEEP_END"
PLATFORM_DEEP_SLEEP_SECONDS = 60.0
PLATFORM_DEEP_SLEEP_TOLERANCE_SECONDS = PLATFORM_DEEP_SLEEP_SECONDS * 0.15 + 2.0
UNITY_READY_PROMPT = "Press ENTER to see the list of tests"


def _run_platform_deep_sleep_case(
    dut: IdfDut, case: UnittestMenuCase, timeout: int
) -> None:
    assert case.type == "multi_stage"
    assert len(case.subcases) == 2

    dut.serial.hard_reset()
    dut.expect_exact(UNITY_READY_PROMPT, timeout=timeout)
    dut.confirm_write(case.index, expect_str=f"Running {case.name}...")
    dut.write(str(case.subcases[0]["index"]))
    dut.expect_exact(PLATFORM_DEEP_SLEEP_START_MARKER, timeout=timeout)
    started_at = time.monotonic()

    dut.expect_exact(UNITY_READY_PROMPT, timeout=timeout)
    dut.confirm_write(case.index, expect_str=f"Running {case.name}...")
    dut.write(str(case.subcases[1]["index"]))
    dut.expect_exact(PLATFORM_DEEP_SLEEP_END_MARKER, timeout=timeout)
    observed_duration = time.monotonic() - started_at
    dut.expect_unity_test_output(timeout=timeout)

    assert (
        abs(observed_duration - PLATFORM_DEEP_SLEEP_SECONDS)
        <= PLATFORM_DEEP_SLEEP_TOLERANCE_SECONDS
    ), (
        f"timer deep sleep lasted {observed_duration:.3f}s; expected "
        f"{PLATFORM_DEEP_SLEEP_SECONDS:.0f}s +/- "
        f"{PLATFORM_DEEP_SLEEP_TOLERANCE_SECONDS:.0f}s"
    )


def test_node_persistence_on_device(dut: IdfDut) -> None:
    requested = os.environ.get("CURAG_HARDWARE_TEST_SET", "fast")
    if requested not in {"fast", "slow", "all"}:
        raise ValueError(f"unknown hardware test set: {requested}")

    selected = [
        case
        for case in dut.test_menu
        if requested == "all"
        or (requested == "slow" and "slow" in case.groups)
        or (requested == "fast" and "slow" not in case.groups)
    ]
    if not selected:
        raise RuntimeError(f"hardware test set {requested!r} selected no Unity cases")

    timeout = 300 if requested == "slow" else 120
    if requested == "all":
        timeout = 300

    for case in selected:
        if case.name == PLATFORM_DEEP_SLEEP_CASE:
            _run_platform_deep_sleep_case(dut, case, timeout)
        else:
            dut.run_single_board_case(case.name, reset=True, timeout=timeout)
