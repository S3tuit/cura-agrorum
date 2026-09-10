"""Checked integer clock arithmetic; policy callers decide how failures affect trust."""

_UINT64_MAX = (1 << 64) - 1
_INT64_MIN = -(1 << 63)
_INT64_MAX = (1 << 63) - 1
_PPM = 1_000_000
MONOTONIC_ELAPSED_RATE_BOUND_PPM = 3700


def _unsigned(value: int) -> int:
    if type(value) is not int:
        raise TypeError("elapsed arithmetic requires exact integers")
    if not 0 <= value <= _UINT64_MAX:
        raise OverflowError("elapsed value exceeds unsigned 64-bit range")
    return value


def checked_monotonic_deadline(now_us: int, wait_us: int) -> int:
    now_us, wait_us = _unsigned(now_us), _unsigned(wait_us)
    if wait_us > _UINT64_MAX - now_us:
        raise OverflowError("monotonic deadline addition overflow")
    return now_us + wait_us


def minimum_wait_monotonic_us(
    duration_us: int, *, rate_bound_ppm: int = MONOTONIC_ELAPSED_RATE_BOUND_PPM
) -> int:
    return _ceiling(
        checked_duration_product(duration_us, _PPM + _rate(rate_bound_ppm)), _PPM
    )


def checked_duration_us(value: int) -> int:
    return _unsigned(value)


def checked_utc_us(value: int) -> int:
    if type(value) is not int:
        raise TypeError("UTC arithmetic requires exact integers")
    if not _INT64_MIN <= value <= _INT64_MAX:
        raise OverflowError("UTC value exceeds signed 64-bit range")
    return value


def checked_utc_offset(utc_us: int, offset_us: int) -> int:
    utc_us, offset_us = checked_utc_us(utc_us), checked_utc_us(offset_us)
    if offset_us > 0 and utc_us > _INT64_MAX - offset_us:
        raise OverflowError("UTC addition overflow")
    if offset_us < 0 and utc_us < _INT64_MIN - offset_us:
        raise OverflowError("UTC addition overflow")
    return utc_us + offset_us


def checked_utc_difference(left_us: int, right_us: int) -> int:
    left_us, right_us = checked_utc_us(left_us), checked_utc_us(right_us)
    if right_us < 0 and left_us > _INT64_MAX + right_us:
        raise OverflowError("UTC subtraction overflow")
    if right_us > 0 and left_us < _INT64_MIN + right_us:
        raise OverflowError("UTC subtraction overflow")
    return left_us - right_us


def checked_absolute_us(value: int) -> int:
    value = checked_utc_us(value)
    if value == _INT64_MIN:
        raise OverflowError("signed absolute value overflow")
    return abs(value)


def checked_duration_product(value: int, factor: int) -> int:
    value, factor = _unsigned(value), _unsigned(factor)
    if factor and value > _UINT64_MAX // factor:
        raise OverflowError("duration multiplication overflow")
    return value * factor


def checked_monotonic_elapsed(start_us: int, finish_us: int) -> int:
    start_us, finish_us = _unsigned(start_us), _unsigned(finish_us)
    if finish_us < start_us:
        raise ValueError("monotonic bracket moved backwards")
    return finish_us - start_us


def checked_monotonic_midpoint(start_us: int, finish_us: int) -> int:
    return checked_monotonic_deadline(
        start_us, checked_monotonic_elapsed(start_us, finish_us) // 2
    )


def checked_correlated_utc(
    utc_us: int, observation_monotonic_us: int, event_monotonic_us: int
) -> int:
    observation = _unsigned(observation_monotonic_us)
    event = _unsigned(event_monotonic_us)
    # A signed displacement is required by the UTC arithmetic contract.
    offset = checked_utc_us(event - observation)
    return checked_utc_offset(utc_us, offset)


def checked_whole_second_midpoint(utc_seconds: int) -> int:
    seconds = checked_utc_us(utc_seconds)
    if not -((-_INT64_MIN) // _PPM) <= seconds <= _INT64_MAX // _PPM:
        raise OverflowError("UTC seconds multiplication overflow")
    return checked_utc_offset(seconds * _PPM, 500_000)


def _rate(rate_bound_ppm: int) -> int:
    if type(rate_bound_ppm) is not int or not 0 <= rate_bound_ppm < _PPM:
        raise ValueError("rate bound must be an integer in 0..999999")
    return rate_bound_ppm


def _ceiling(numerator: int, denominator: int) -> int:
    quotient, remainder = divmod(numerator, denominator)
    return quotient + bool(remainder)


def maximum_lifetime_monotonic_us(
    duration_us: int, *, rate_bound_ppm: int = MONOTONIC_ELAPSED_RATE_BOUND_PPM
) -> int:
    return checked_duration_product(duration_us, _PPM - _rate(rate_bound_ppm)) // _PPM


def rate_growth_us(rate_bound_ppm: int, observed_elapsed_us: int) -> int:
    rate = _rate(rate_bound_ppm)
    return _ceiling(checked_duration_product(observed_elapsed_us, rate), _PPM - rate)


def maximum_physical_half_bracket_us(
    observed_bracket_us: int,
    *,
    rate_bound_ppm: int = MONOTONIC_ELAPSED_RATE_BOUND_PPM,
) -> int:
    denominator = 2 * (_PPM - _rate(rate_bound_ppm))
    return _ceiling(checked_duration_product(observed_bracket_us, _PPM), denominator)


def exclusive_trust_distance_us(
    observation_error_us: int,
    budget_us: int,
    *,
    rate_bound_ppm: int = MONOTONIC_ELAPSED_RATE_BOUND_PPM,
) -> int | None:
    """Return 0 when exhausted, None at zero rate, otherwise the first unsafe distance."""
    error, budget = _unsigned(observation_error_us), _unsigned(budget_us)
    rate = _rate(rate_bound_ppm)
    if error >= budget:
        return 0
    if rate == 0:
        return None
    last = checked_duration_product(budget - error - 1, _PPM - rate) // rate
    return checked_monotonic_deadline(last, 1)
