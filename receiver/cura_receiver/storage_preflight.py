"""Service ExecStartPre: validate the installed writable storage paths."""

import os
import tempfile
from .application_settings import ApplicationSettings


def check_storage(settings):
    data = settings.database_path.parent
    temporary = settings.sqlite_temporary_directory
    if not data.is_dir() or not temporary.is_dir():
        raise ValueError('required storage directory missing')
    if data.stat().st_dev != temporary.stat().st_dev:
        raise ValueError('SQLite temporary directory must share the data filesystem')
    for directory in (data, temporary):
        if not os.access(directory, os.W_OK | os.X_OK):
            raise ValueError('required storage directory is not writable')
        with tempfile.TemporaryFile(dir=directory):
            pass
    if not settings.database_path.is_file():
        raise ValueError('database must be initialized offline before service startup')
    space = os.statvfs(data)
    if space.f_bavail * space.f_frsize < settings.minimum_free_bytes:
        raise ValueError('preventive free-space reserve unavailable')


def main():
    try:
        check_storage(ApplicationSettings())
    except (OSError, ValueError):
        print('receiver storage preflight failed')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
