"""Production adapters for the receiver's deployed platform."""

from .linux_clocks import LinuxOsClock

__all__ = ["LinuxOsClock"]
