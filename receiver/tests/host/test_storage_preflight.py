from dataclasses import replace

import pytest

from cura_receiver.application_settings import ApplicationSettings
from cura_receiver.storage_preflight import check_storage


def test_storage_preflight_requires_initialized_database_and_temp_path(tmp_path):
    settings = replace(ApplicationSettings(), database_path=tmp_path/'receiver.db',
        sqlite_temporary_directory=tmp_path/'sqlite-temp', minimum_free_bytes=0)
    with pytest.raises(ValueError, match='directory missing'):
        check_storage(settings)
    settings.sqlite_temporary_directory.mkdir()
    with pytest.raises(ValueError, match='initialized offline'):
        check_storage(settings)
    settings.database_path.touch()
    check_storage(settings)
    assert list(settings.sqlite_temporary_directory.iterdir()) == []
    with pytest.raises(ValueError, match='free-space reserve'):
        check_storage(replace(settings, minimum_free_bytes=(1 << 63)-1))
