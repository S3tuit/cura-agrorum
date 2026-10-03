"""Explicit, single-use authorization for known-empty airtime commissioning."""

from enum import Enum, auto

AIRTIME_COMMISSIONING_TOKEN = b"cura-airtime-known-empty-v1"


class AirtimeCommissioningState(Enum):
    ABSENT = auto()
    PENDING = auto()
    INVALID = auto()
