"""Shared startup paths; parsing does not access storage or credentials."""

from dataclasses import replace
from pathlib import Path

from .application_settings import ApplicationSettings


PATH_VARIABLES = {
    "CURA_RECEIVER_CONFIGURATION": "configuration_path",
    "CURA_RECEIVER_DATABASE": "database_path",
    "SQLITE_TMPDIR": "sqlite_temporary_directory",
}


def _absolute_path(value):
    if not isinstance(value, str) or not value or "\0" in value:
        raise ValueError("invalid deployment path")
    path = Path(value)
    if not path.is_absolute() or ".." in path.parts:
        raise ValueError("invalid deployment path")
    # Linux treats // like /; pathlib preserves it as a distinct anchor.
    return Path("/" + value.lstrip("/"))


def settings_from_environment(environment):
    settings = ApplicationSettings()
    present = [name in environment for name in PATH_VARIABLES]
    if any(present) and not all(present):
        raise ValueError("deployment paths must be supplied together")
    if all(present):
        settings = replace(settings, **{
            field: _absolute_path(environment[name])
            for name, field in PATH_VARIABLES.items()
        })
    if "CURA_RECEIVER_TEST_ROOT" in environment:
        if not all(present):
            raise ValueError("test deployment requires explicit paths")
        root = _absolute_path(environment["CURA_RECEIVER_TEST_ROOT"])
        for protected in (Path("/etc/cura-agrorum"), Path("/var/lib/cura-agrorum")):
            if root.is_relative_to(protected) or protected.is_relative_to(root):
                raise ValueError("test root overlaps production storage")
        for field in PATH_VARIABLES.values():
            path = getattr(settings, field)
            if path == root or not path.is_relative_to(root):
                raise ValueError("test paths must remain below the test root")
    return settings
