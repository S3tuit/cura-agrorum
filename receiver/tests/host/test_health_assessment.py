from dataclasses import replace

import pytest

from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.health_assessment import assess_health
from tests.support.builders.persistence import _radio_observation


def test_silent_radio_standby_contradicts_software_rx():
    snapshot = replace(_radio_observation().snapshot, chip_status=0x24,
                       chip_mode=E.RadioChipMode.STBY_RC)
    assert assess_health(snapshot, E.RtcHealth.PRESENT, E.RtcReadStatus.OK) is E.HealthConsistency.INCONSISTENT


@pytest.mark.parametrize('context', [dict(irq_status=2), dict(dio1_after=True),
                                    dict(event_sequence=1, event_timestamp_ns=13000)])
def test_packet_arrival_during_snapshot_is_not_a_mode_fault(context):
    snapshot = replace(_radio_observation().snapshot, chip_mode=E.RadioChipMode.STBY_RC, **context)
    assert assess_health(snapshot, E.RtcHealth.PRESENT, E.RtcReadStatus.OK) is E.HealthConsistency.INDETERMINATE


@pytest.mark.parametrize('status', [E.RtcReadStatus.MISSING, E.RtcReadStatus.IO_ERROR,
                                  E.RtcReadStatus.DEADLINE_EXCEEDED, E.RtcReadStatus.NOT_ATTEMPTED])
def test_failed_access_does_not_prove_physical_absence(status):
    assert assess_health(_radio_observation().snapshot, E.RtcHealth.PRESENT, status) is E.HealthConsistency.INDETERMINATE


def test_positive_contradiction_takes_precedence_over_failed_other_probe():
    snapshot = replace(_radio_observation().snapshot, capture_status=E.RadioCaptureStatus.UNAVAILABLE)
    assert assess_health(snapshot, E.RtcHealth.MISSING, E.RtcReadStatus.OK) is E.HealthConsistency.INCONSISTENT


def test_consistency_includes_agreement_about_invalid_rtc():
    snapshot = _radio_observation().snapshot
    assert assess_health(snapshot, E.RtcHealth.PRESENT, E.RtcReadStatus.OK) is E.HealthConsistency.CONSISTENT
    assert assess_health(snapshot, E.RtcHealth.INVALID, E.RtcReadStatus.INVALID) is E.HealthConsistency.CONSISTENT
    assert assess_health(snapshot, E.RtcHealth.PRESENT, E.RtcReadStatus.INVALID) is E.HealthConsistency.INCONSISTENT


@pytest.mark.parametrize('status', [0x56, 0x58, 0x5a])
def test_chip_in_rx_with_command_failure_does_not_count_as_consistent(status):
    snapshot = replace(_radio_observation().snapshot, chip_status=status)
    assert assess_health(snapshot, E.RtcHealth.PRESENT, E.RtcReadStatus.OK) is E.HealthConsistency.INCONSISTENT
