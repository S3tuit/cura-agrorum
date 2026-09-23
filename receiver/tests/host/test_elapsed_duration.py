import pytest
from hypothesis import given, strategies as st
from cura_receiver import elapsed_duration as arithmetic
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


# Maximum lifetimes round down even when the shortest physical duration disappears.
@pytest.mark.parametrize(
    "duration,expected", [(0, 0), (1, 0), (271, 269), (30_000_000, 29_889_000)]
)
def test_maximum_lifetime_examples(duration, expected):
    assert arithmetic.maximum_lifetime_monotonic_us(duration) == expected


# The slow-clock denominator and rounded half-bracket match independent literal examples.
@pytest.mark.parametrize(
    "rate,elapsed,growth",
    [
        (0, 100, 0),
        (3700, 1, 1),
        (3700, 3_600_000_000, 13_369_468),
        (10, 100_000, 2),
        (999999, 1, 999999),
    ],
)
def test_rate_growth_examples(rate, elapsed, growth):
    assert arithmetic.rate_growth_us(rate, elapsed) == growth


# Odd brackets and the pilot one-second bound retain conservative physical uncertainty.
@pytest.mark.parametrize(
    "bracket,expected", [(0, 0), (1, 1), (2, 2), (1_000_000, 501_857)]
)
def test_half_bracket_examples(bracket, expected):
    assert arithmetic.maximum_physical_half_bracket_us(bracket) == expected


# A one-microsecond remaining budget lasts only until the first rounded error increment.
def test_strict_horizon_regression():
    assert arithmetic.exclusive_trust_distance_us(39_999_999, 40_000_000) == 1
    assert (
        arithmetic.exclusive_trust_distance_us(5_500_000, 40_000_000) == 9_289_824_056
    )
    assert arithmetic.exclusive_trust_distance_us(40_000_000, 40_000_000) == 0
    assert arithmetic.exclusive_trust_distance_us(40_000_001, 40_000_000) == 0
    assert arithmetic.exclusive_trust_distance_us(0, 1, rate_bound_ppm=0) is None
    assert arithmetic.exclusive_trust_distance_us(1, 1, rate_bound_ppm=0) == 0


# Rational inequalities independently prove that every calculated boundary is the first unsafe unit.
@given(
    error=st.integers(0, 40_000_000),
    remaining=st.integers(1, 40_000_000),
    rate=st.integers(1, 999999),
)
def test_horizon_is_exact_integer_inverse(error, remaining, rate):
    boundary = arithmetic.exclusive_trust_distance_us(
        error, error + remaining, rate_bound_ppm=rate
    )
    assert (boundary - 1) * rate <= (remaining - 1) * (1_000_000 - rate)
    assert boundary * rate > (remaining - 1) * (1_000_000 - rate)


# Converted waits and lifetimes enclose their physical durations at both rate extremes.
@given(duration=st.integers(0, 1_000_000_000), rate=st.integers(0, 999999))
def test_duration_rounding_inequalities(duration, rate):
    minimum = minimum_wait_monotonic_us(duration, rate_bound_ppm=rate)
    maximum = arithmetic.maximum_lifetime_monotonic_us(duration, rate_bound_ppm=rate)
    assert minimum * 1_000_000 >= duration * (1_000_000 + rate)
    assert (minimum - 1) * 1_000_000 < duration * (1_000_000 + rate)
    assert maximum * 1_000_000 <= duration * (1_000_000 - rate)
    assert (maximum + 1) * 1_000_000 > duration * (1_000_000 - rate)


# UTC zero is a timestamp, signed offsets cross the epoch, and brackets can repeat near u64 max.
def test_checked_clock_scalars():
    assert arithmetic.checked_correlated_utc(0, 100, 99) == -1
    assert arithmetic.checked_utc_difference(-1, -2) == 1
    assert arithmetic.checked_absolute_us(-3) == 3
    assert arithmetic.checked_whole_second_midpoint(-1) == -500_000
    assert arithmetic.checked_whole_second_midpoint(0) == 500_000
    assert (
        arithmetic.checked_monotonic_midpoint((1 << 64) - 2, (1 << 64) - 1)
        == (1 << 64) - 2
    )
    assert arithmetic.checked_monotonic_elapsed(0, 0) == 0
    assert arithmetic.checked_utc_offset(-(1 << 63), (1 << 63) - 1) == -1


# Every checked operation rejects overflow even when a later division or offset could hide it.
@pytest.mark.parametrize(
    "function,args",
    [
        (arithmetic.maximum_lifetime_monotonic_us, (1 << 63,)),
        (arithmetic.maximum_physical_half_bracket_us, (1 << 63,)),
        (arithmetic.rate_growth_us, (3700, 1 << 63)),
        (arithmetic.exclusive_trust_distance_us, (0, 1 << 63)),
        (arithmetic.checked_utc_offset, ((1 << 63) - 1, 1)),
        (arithmetic.checked_utc_offset, (-(1 << 63), -1)),
        (arithmetic.checked_utc_difference, ((1 << 63) - 1, -1)),
        (arithmetic.checked_utc_difference, (-(1 << 63), 1)),
        (arithmetic.checked_absolute_us, (-(1 << 63),)),
        (arithmetic.checked_whole_second_midpoint, ((1 << 63) - 1,)),
        (arithmetic.checked_whole_second_midpoint, (-(1 << 63),)),
        (arithmetic.checked_correlated_utc, (0, 0, (1 << 64) - 1)),
        (arithmetic.checked_monotonic_elapsed, (2, 1)),
        (arithmetic.checked_duration_product, (True, 1)),
        (arithmetic.checked_utc_us, (True,)),
    ],
)
def test_checked_arithmetic_rejects_invalid_results(function, args):
    with pytest.raises((TypeError, ValueError, OverflowError)):
        function(*args)


# The largest safe product succeeds and its next input fails before evaluation.
def test_product_exact_limit():
    limit = ((1 << 64) - 1) // 1_000_000
    assert arithmetic.checked_duration_product(limit, 1_000_000) == limit * 1_000_000
    with pytest.raises(OverflowError):
        arithmetic.checked_duration_product(limit + 1, 1_000_000)
