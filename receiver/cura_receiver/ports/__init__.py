"""Narrow capabilities consumed by receiver policy and orchestration."""

from .clocks import MonotonicClock, RealtimeClock

__all__ = ["MonotonicClock", "RealtimeClock"]
