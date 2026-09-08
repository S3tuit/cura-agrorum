import pytest

from carrier_runner import APP, validate_selection


def pytest_addoption(parser):
    group = parser.getgroup("sensor-carrier")
    group.addoption("--sensor-operation", choices=["discover", "gate-on", "gate-off", "acquire"])
    group.addoption("--sensor-fixture", choices=["nominal"])
    group.addoption("--carrier-revision")
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
