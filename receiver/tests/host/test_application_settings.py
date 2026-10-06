"""The explicit pilot profile accepts isolated overrides and rejects unsafe paths."""

from dataclasses import replace
from pathlib import Path

import pytest

from cura_receiver.application_settings import ApplicationSettings
from cura_receiver.time_policy import TimePolicy


def test_startup_budget_has_no_implicit_default():
    with pytest.raises(TypeError, match='persistence_startup_budget_us'):
        ApplicationSettings()


@pytest.mark.parametrize('value', [float('inf'), float('nan'), 1 << 64])
def test_startup_budget_rejects_nonfinite_or_overflowing_values(value):
    with pytest.raises((TypeError, OverflowError)):
        ApplicationSettings(persistence_startup_budget_us=value)


# Dedicated tests can replace every persistent path without mutating the pilot profile.
def test_isolated_paths_and_time_policy_do_not_change_pilot_settings(tmp_path):
    pilot = ApplicationSettings(persistence_startup_budget_us=12_000_000)
    isolated = replace(
        pilot,
        configuration_path=tmp_path / "test-group.json",
        database_path=tmp_path / "test.sqlite3",
        sqlite_temporary_directory=tmp_path / "sqlite-tmp",
        time_policy=TimePolicy(network_trust_error_threshold_us=30_000_000),
    )
    assert isolated.configuration_path.parent == tmp_path
    assert isolated.database_path.parent == tmp_path
    assert isolated.sqlite_temporary_directory.parent == tmp_path
    assert pilot.time_policy.network_trust_error_threshold_us == 35_000_000
    assert isolated.time_policy.network_trust_error_threshold_us == 30_000_000
    assert pilot.database_path != isolated.database_path


# Invalid placement must fail before startup can touch configuration or storage.
@pytest.mark.parametrize("field", ("configuration_path", "database_path", "sqlite_temporary_directory"))
@pytest.mark.parametrize("path", (Path("relative"), Path("/var/lib/../etc/data"), "/absolute/string"))
def test_reject_invalid_application_paths(field, path):
    with pytest.raises(ValueError):
        replace(ApplicationSettings(persistence_startup_budget_us=12_000_000), **{field: path})


# Distinct configuration, database and temporary locations cannot alias by name.
@pytest.mark.parametrize("field", ("database_path", "sqlite_temporary_directory"))
def test_reject_overlapping_application_paths(field):
    settings = ApplicationSettings(persistence_startup_budget_us=12_000_000)
    with pytest.raises(ValueError, match="distinct"):
        replace(settings, **{field: settings.configuration_path})


# Invalid budgets cannot silently disable bounded lifecycle behavior.
@pytest.mark.parametrize("field", ("persistence_startup_budget_us", "health_interval_us", "shutdown_budget_us"))
@pytest.mark.parametrize("value,error", ((0, ValueError), (-1, OverflowError), (True, TypeError), (1.5, TypeError)))
def test_reject_invalid_application_intervals(field, value, error):
    with pytest.raises(error):
        replace(ApplicationSettings(persistence_startup_budget_us=12_000_000), **{field: value})
