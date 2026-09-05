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
RTC_REPEATED_ROUND_TRIP_CASE = (
    "RTC repeats exactly 20 retained deep-sleep round trips"
)
RTC_REPEATED_ROUND_TRIPS = 20
RTC_CONSUMPTION_RESTART_CASE = (
    "node core restart after RTC consumption does not reuse metrics"
)
UNITY_READY_PROMPT = "Press ENTER to see the list of tests"
UNITY_MENU_HEADING = "Here's the test menu, pick your combo:"
UNITY_MENU_END = "Enter test for running."


def _load_test_menu(
    dut: IdfDut, timeout: int
) -> list[UnittestMenuCase]:
    dut.expect_exact(UNITY_READY_PROMPT, timeout=timeout)
    dut.write("")
    dut.expect_exact(UNITY_MENU_HEADING, timeout=10)
    menu_text = dut.expect_exact(
        UNITY_MENU_END,
        timeout=10,
        return_what_before_match=True,
    )
    menu = dut._parse_unity_menu_from_str(menu_text.decode("utf8"))
    dut._test_menu = menu
    return menu


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


def _run_repeated_rtc_round_trip_case(
    dut: IdfDut, case: UnittestMenuCase, timeout: int
) -> None:
    assert case.type == "multi_stage"
    assert len(case.subcases) == 2

    dut.serial.hard_reset()
    dut.expect_exact(UNITY_READY_PROMPT, timeout=timeout)
    dut.confirm_write(case.index, expect_str=f"Running {case.name}...")
    dut.write(str(case.subcases[0]["index"]))

    for iteration in range(RTC_REPEATED_ROUND_TRIPS):
        dut.expect_exact(UNITY_READY_PROMPT, timeout=timeout)
        dut.confirm_write(case.index, expect_str=f"Running {case.name}...")
        if iteration + 1 == RTC_REPEATED_ROUND_TRIPS:
            dut.write(str(case.subcases[1]["index"]))
            dut.expect_unity_test_output(timeout=timeout)
        else:
            dut.write(str(case.subcases[1]["index"]))


def _run_rtc_consumption_restart_case(
    dut: IdfDut, case: UnittestMenuCase, timeout: int
) -> None:
    assert case.type == "multi_stage"
    assert len(case.subcases) == 4

    final_subcase = case.subcases[-1]
    final_menu_line = (
        f'\t({final_subcase["index"]})\t"{final_subcase["name"]}"'
    )
    dut.serial.hard_reset()
    dut.expect_exact(UNITY_READY_PROMPT, timeout=timeout)
    for stage, subcase in enumerate(case.subcases):
        dut.confirm_write(case.index, expect_str=f"Running {case.name}...")
        dut.expect_exact(final_menu_line, timeout=timeout)
        dut.write(str(subcase["index"]))
        if stage + 1 < len(case.subcases):
            dut.expect_exact(UNITY_READY_PROMPT, timeout=timeout)
    dut.expect_unity_test_output(timeout=timeout)


def test_node_persistence_on_device(dut: IdfDut) -> None:
    requested = os.environ.get("CURAG_HARDWARE_TEST_SET", "fast")
    if requested not in {"fast", "slow", "all"}:
        raise ValueError(f"unknown hardware test set: {requested}")

    timeout = 300 if requested == "slow" else 120
    if requested == "all":
        timeout = 300

    selected = [
        case
        for case in _load_test_menu(dut, timeout)
        if requested == "all"
        or (requested == "slow" and "slow" in case.groups)
        or (requested == "fast" and "slow" not in case.groups)
    ]
    if not selected:
        raise RuntimeError(f"hardware test set {requested!r} selected no Unity cases")

    for case in selected:
        if case.name == PLATFORM_DEEP_SLEEP_CASE:
            _run_platform_deep_sleep_case(dut, case, timeout)
        elif case.name == RTC_CONSUMPTION_RESTART_CASE:
            _run_rtc_consumption_restart_case(dut, case, timeout)
        elif case.name == RTC_REPEATED_ROUND_TRIP_CASE:
            _run_repeated_rtc_round_trip_case(dut, case, timeout)
        else:
            dut.run_single_board_case(case.name, reset=True, timeout=timeout)
