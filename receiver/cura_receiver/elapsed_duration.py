"""Centralized checked elapsed-duration conversion for minimum physical waits."""

_UINT64_MAX = (1 << 64) - 1
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
    duration_us = _unsigned(duration_us)
    if type(rate_bound_ppm) is not int or not 0 <= rate_bound_ppm < _PPM:
        raise ValueError("rate bound must be an integer in 0..999999")
    factor = _PPM + rate_bound_ppm
    if duration_us > _UINT64_MAX // factor:
        raise OverflowError("minimum wait multiplication overflow")
    product = duration_us * factor
    # divmod performs ceiling division without an unchecked addition of P-1.
    quotient, remainder = divmod(product, _PPM)
    return quotient + bool(remainder)
