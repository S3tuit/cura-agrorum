import pytest
from pathlib import Path
import xml.etree.ElementTree as ET

from carrier_runner import APP, OPERATIONS, validate_selection


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config):
    if config.getoption('sensor_operation'):
        # The default replacement drops failed host meter/orchestration cases
        # when only the Unity preflight passed. Keep both layers in the report.
        config.option.unity_test_report_mode = 'merge'


@pytest.hookimpl(wrapper=True, tryfirst=True)
def pytest_sessionfinish(session, exitstatus):
    yield
    if not session.config.getoption('sensor_operation'):
        return
    path = session.config.getoption('xmlpath')
    if not path or not Path(path).is_file():
        return
    # Run after pytest-embedded's merger. The installed merger retains the
    # Python case in merge mode but subtracts it from its aggregate counts.
    # Recount the actual cases, preserving every failure/error and its text.
    report = ET.parse(path)
    failed = False
    for node in report.iter():
        if node.tag not in {'testsuite', 'testsuites'}:
            continue
        cases = node.findall('.//testcase')
        node.set('tests', str(len(cases)))
        for attribute, element in [('failures', 'failure'), ('errors', 'error'), ('skipped', 'skipped')]:
            count = sum(case.find(element) is not None for case in cases)
            node.set(attribute, str(count))
            failed |= bool(count) and attribute in {'failures', 'errors'}
    report.write(path, encoding='utf-8', xml_declaration=True)
    if failed and session.exitstatus == 0:
        session.exitstatus = pytest.ExitCode.TESTS_FAILED


def pytest_addoption(parser):
    group = parser.getgroup("sensor-carrier")
    group.addoption("--sensor-operation", choices=sorted(OPERATIONS))
    group.addoption("--sensor-fixture", choices=["nominal", "adc_reference", "missing_ds0", "missing_ds1"])
    group.addoption("--carrier-revision")
    group.addoption("--sensor-dut", default="cc8da2fc0224")
    group.addoption("--sensor-repeat-count", type=int, default=100)
    group.addoption("--sensor-guided", action="store_true")
    group.addoption("--exploration", action="store_true",
                    help="non-accepting, operator-ended free-text electrical observations")
    group.addoption("--sensor-position", choices=["A", "B"])
    group.addoption("--sensor-prior-evidence")
    group.addoption("--sensor-fixture-ready", action="store_true")


def pytest_collection_finish(session):
    hardware = [item for item in session.items
                if item.path == APP / "pytest_sensor_carrier.py"]
    if session.config.getoption("sensor_operation") and not hardware:
        raise pytest.UsageError("requested sensor operation selected no hardware test")
    if hardware:
        try:
            validate_selection(session.config.getoption("sensor_operation"),
                               session.config.getoption("sensor_fixture"),
                               session.config.getoption("carrier_revision"),
                               session.config.getoption("sensor_fixture_ready"))
        except ValueError as exc:
            raise pytest.UsageError(str(exc)) from exc
