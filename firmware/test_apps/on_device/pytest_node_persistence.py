import os

from pytest_embedded_idf.dut import IdfDut


def test_node_persistence_on_device(dut: IdfDut) -> None:
    requested = os.environ.get("CURAG_HARDWARE_TEST_SET", "fast")
    groups = {
        "fast": "!slow",
        "slow": "slow",
        "all": None,
    }
    if requested not in groups:
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
    dut.run_all_single_board_cases(
        group=groups[requested], reset=True, timeout=timeout
    )
