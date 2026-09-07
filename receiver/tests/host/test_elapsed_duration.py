import pytest
from cura_receiver.elapsed_duration import (
    checked_monotonic_deadline,
    minimum_wait_monotonic_us,
)


# Retry waits and published architecture examples use the conservative physical-to-monotonic ceiling.
@pytest.mark.parametrize(
    "duration,expected",
    [
        (0, 0),
        (1, 2),
        (250000, 250925),
        (5000000, 5018500),
        (3600000000, 3613320000),
        (10800000000, 10839960000),
    ],
)
def test_minimum_wait_examples(duration, expected):
    assert minimum_wait_monotonic_us(duration) == expected


# Integer boundaries reject overflow before multiplication or deadline addition changes meaning.
def test_checked_elapsed_overflow():
    with pytest.raises(OverflowError):
        minimum_wait_monotonic_us(1 << 63)
    with pytest.raises(OverflowError):
        checked_monotonic_deadline((1 << 64) - 1, 1)
    assert checked_monotonic_deadline((1 << 64) - 2, 1) == (1 << 64) - 1
    assert minimum_wait_monotonic_us(17, rate_bound_ppm=0) == 17


# Booleans, negative durations and out-of-range rate bounds are not elapsed arithmetic inputs.
@pytest.mark.parametrize(
    "duration,rate", [(True, 0), (-1, 0), (0, -1), (0, 1000000), (0, True)]
)
def test_elapsed_invalid_inputs(duration, rate):
    with pytest.raises((TypeError, ValueError, OverflowError)):
        minimum_wait_monotonic_us(duration, rate_bound_ppm=rate)
