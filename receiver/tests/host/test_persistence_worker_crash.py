import pytest
from tests.support.coordination.worker_crash import BOUNDARIES, exercise_worker_crash


# Named worker SIGKILL boundaries preserve atomic effects and restart with a new empty queue.
@pytest.mark.parametrize("boundary", BOUNDARIES)
def test_worker_process_crash(worker_files, boundary):
    exercise_worker_crash(worker_files, boundary)
