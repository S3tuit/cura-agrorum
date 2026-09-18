"""The explicit pilot profile accepts isolated overrides and rejects unsafe paths."""

from dataclasses import replace
from pathlib import Path

import pytest

from cura_receiver.application_settings import ApplicationSettings
from cura_receiver.time_policy import TimePolicy


# Dedicated tests can replace every persistent path without mutating the pilot profile.
def test_isolated_paths_and_time_policy_do_not_change_pilot_settings(tmp_path):
    pilot = ApplicationSettings()
    isolated = replace(
        pilot,
        configuration_path=tmp_path / "test-group.json",
        database_path=tmp_path / "test.sqlite3",
        sqlite_temporary_directory=tmp_path / "sqlite-tmp",
        time_policy=TimePolicy(maximum_network_skew_ppb=1000),
    )
    assert isolated.configuration_path.parent == tmp_path
    assert isolated.database_path.parent == tmp_path
    assert isolated.sqlite_temporary_directory.parent == tmp_path
    assert pilot.time_policy.maximum_network_skew_ppb == 10_000
    assert isolated.time_policy.maximum_network_skew_ppb == 1000
    assert pilot.database_path != isolated.database_path


# Invalid placement must fail before startup can touch configuration or storage.
@pytest.mark.parametrize("field", ("configuration_path", "database_path", "sqlite_temporary_directory"))
@pytest.mark.parametrize("path", (Path("relative"), Path("/var/lib/../etc/data"), "/absolute/string"))
def test_reject_invalid_application_paths(field, path):
    with pytest.raises(ValueError):
        replace(ApplicationSettings(), **{field: path})


# Distinct configuration, database and temporary locations cannot alias by name.
@pytest.mark.parametrize("field", ("database_path", "sqlite_temporary_directory"))
def test_reject_overlapping_application_paths(field):
    settings = ApplicationSettings()
    with pytest.raises(ValueError, match="distinct"):
        replace(settings, **{field: settings.configuration_path})


# Invalid budgets cannot silently disable bounded lifecycle behavior.
@pytest.mark.parametrize("field", ("health_interval_us", "shutdown_budget_us"))
@pytest.mark.parametrize("value,error", ((0, ValueError), (-1, OverflowError), (True, TypeError), (1.5, TypeError)))
def test_reject_invalid_application_intervals(field, value, error):
    with pytest.raises(error):
        replace(ApplicationSettings(), **{field: value})
