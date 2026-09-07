import pytest
from tests.support.coordination.persistence_crash import BOUNDARIES, exercise_pair_crash


# Named SIGKILL boundaries leave no partial pair; replay uses explicitly test-held immutable input.
@pytest.mark.parametrize("boundary", BOUNDARIES)
def test_process_crash_durable_pair(tmp_path, boundary):
    exercise_pair_crash(tmp_path, boundary)
