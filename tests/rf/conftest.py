"""Collection guards run before requesting any pytest-embedded DUT fixture."""
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pytest

from inputs import APP, finish_session, verify_build, write_json
from spec import select_cases, validate_fixture


def pytest_addoption(parser):
    group = parser.getgroup("joint-rf")
    for name in ("cases", "fixture", "run", "output", "manual-record", "session"):
        group.addoption("--rf-" + name)
    group.addoption("--rf-host", default="cura-receiver")
    group.addoption("--rf-host-key-alias")
    group.addoption("--rf-peer-python", default="python3")
    group.addoption("--rf-ready-run", help="operator's explicit batch readiness, bound to this exact run ID")
    group.addoption("--rf-confirm-flash", action="store_true")


def pytest_generate_tests(metafunc):
    if "episode_name" in metafunc.fixturenames:
        value = metafunc.config.getoption("rf_cases")
        names = value.split(",") if value else ["selection-required"]
        metafunc.parametrize("episode_name", names, ids=names)


@pytest.fixture
def port_mac():
    # Our guarded no-reset probe runs before requesting dut. The plugin's own
    # MAC selector invokes read_mac with a reset that could run an old image.
    return None


def validate_config(config):
    option = config.getoption
    if not option("rf_fixture"):
        raise ValueError("--rf-fixture is required")
    fixture = validate_fixture(json.loads(Path(option("rf_fixture")).read_text()))
    episodes = select_cases(option("rf_cases"), fixture["c6_fixture"])
    if not re.fullmatch("[0-9a-f]{32}", option("rf_run") or ""):
        raise ValueError("--rf-run must be a new 32-character lowercase hex ID")
    output = Path(option("rf_output") or "")
    if not output.is_absolute() or output.exists() or not output.parent.is_dir():
        raise ValueError("--rf-output must be a new absolute directory")
    session = Path(option("rf_session") or "")
    if not session.is_absolute() or not session.parent.is_dir() or session.is_dir() or session == output:
        raise ValueError("--rf-session must be an absolute temporary file path with an existing parent")
    record = Path(option("rf_manual_record") or "")
    if not record.is_file() or not record.read_bytes().strip():
        raise ValueError("--rf-manual-record must reference the operator's retained batch sheet")
    if option("rf_ready_run") not in (None, option("rf_run")):
        raise ValueError("operator readiness belongs to a different run")
    if not option("rf_confirm_flash"):
        raise ValueError("--rf-confirm-flash acknowledges factory-app replacement")
    if (any(option(name) for name in ("erase_all", "erase_nvs", "skip_autoflash", "esp_flash_force", "encrypt", "keyfile", "flash_port")) or option("count") != 1 or
            set((option("embedded_services") or "").split(",")) != {"esp", "idf"} or
            option("target") != "esp32c6" or Path(option("app_path") or "").resolve() != APP):
        raise ValueError("require one esp,idf ESP32-C6 radio app, normal flash; no erase-all or skipped flash")
    if option("port") != fixture["c6_uart"]:
        raise ValueError("selected UART differs from fixture")
    expected_mac = ":".join(fixture["c6_dut"][i:i+2] for i in range(0, 12, 2))
    if option("port_mac") != expected_mac:
        raise ValueError("--port-mac must bind the actual factory MAC before flashing")
    build = Path(option("build_dir") or "build")
    if not build.is_absolute():
        build = APP / build
    seal = verify_build(build)
    logdir = Path(option("root_logdir") or "")
    xml = Path(option("xmlpath") or "")
    if not logdir.is_relative_to(output) or not xml.is_relative_to(output):
        raise ValueError("raw embedded logs and --junitxml must be inside --rf-output")
    return dict(fixture=fixture, episodes=episodes, output=output, session=session, manual=record, build=build, seal=seal)


def pytest_collection_finish(session):
    hardware = [item for item in session.items if item.get_closest_marker("rf_component")]
    if not hardware and not session.config.getoption("rf_cases"):
        return
    try:
        context = validate_config(session.config)
        selected = [e.name for e in context["episodes"]]
        if [item.callspec.params["episode_name"] for item in hardware] != selected or len(hardware) != len(session.items):
            raise ValueError("requested RF selection is missing, filtered, reordered or mixed with unrelated tests")
        session.config._rf_context = context
        session.config.option.maxfail = 1
        session.config.option.unity_test_report_mode = "merge"
        if not session.config.option.collectonly:
            context["output"].mkdir(mode=0o700)
    except (ValueError, TypeError, KeyError, OSError) as exc:
        raise pytest.UsageError(str(exc)) from exc


@pytest.hookimpl(wrapper=True, tryfirst=True)
def pytest_sessionfinish(session, exitstatus):
    yield
    run = getattr(session.config, "_rf_run", None)
    if run is None:
        return
    xml = Path(session.config.getoption("xmlpath"))
    if xml.is_file():
        tree = ET.parse(xml)
        failed = False
        for node in tree.iter():
            if node.tag in {"testsuite", "testsuites"}:
                cases = node.findall(".//testcase")
                node.set("tests", str(len(cases)))
                for attr, tag in (("failures", "failure"), ("errors", "error"), ("skipped", "skipped")):
                    count = sum(case.find(tag) is not None for case in cases)
                    node.set(attr, str(count))
                    failed |= bool(count)
        tree.write(xml, encoding="utf-8", xml_declaration=True)
        if failed:
            session.exitstatus = pytest.ExitCode.TESTS_FAILED
    else:
        session.exitstatus = pytest.ExitCode.TESTS_FAILED
    run["pytest_exit"] = int(session.exitstatus)
    run["status"] = "PASS" if session.exitstatus == 0 and len(run["results"]) == len(run["selected"]) else "FAIL"
    root = session.config._rf_context["output"]
    write_json(root / "run.json", run)
    if state := getattr(session.config, "_rf_session", None):
        finish_session(session.config._rf_context["session"], *state, run)
