"""Availability costs are measured separately from independent safety checks."""
import json
import pytest
from tests.support.coordination.airtime import PhysicalRun


@pytest.mark.parametrize('name,seconds,interval,rtc', [
    ('one_ack_per_minute_8h', 8*3600, 60, False),
    ('one_ack_per_second_2h', 2*3600, 1, False),
    ('rtc_save_after_every_ack_2h', 2*3600, 60, True),
])
def test_traffic_profiles(airtime_component, tmp_path, name, seconds, interval, rtc):
    run = PhysicalRun(airtime_component)
    previous = None
    max_gap = max_occupied = 0
    for second in range(0, seconds, interval):
        run.advance(second*1_000_000-int(run.physical))
        run.trust()
        if run.transmit():
            if previous is not None:
                max_gap = max(max_gap, second-previous)
            previous = second
            if rtc or run.policy.save_required:
                run.save(rtc=rtc)
        max_occupied = max(max_occupied, run.policy.total_used//2_000_000)
    if name == 'one_ack_per_minute_8h':
        assert run.sent == 480 and run.denied == 0 and max_gap == 60
    elif rtc:
        assert run.denied > 0 and run.sent < 60
    else:
        assert 800 < run.sent < 1100 and run.denied > 6000
    (tmp_path/(name+'.json')).write_text(json.dumps(dict(
        profile=name, seconds=seconds, requests=seconds//interval,
        sent=run.sent, suppressed=run.denied, saves=run.saves,
        max_ack_gap_s=max_gap, max_occupied_entries=max_occupied,
        hypothetical_pause_seconds_at_2s_per_save=2*run.saves,
        pause_note='Calculated scenario; not measured disk latency or physical RX loss.'),indent=2))


def test_twenty_short_restarts_exhaust_without_transmission(airtime_component, tmp_path):
    run = PhysicalRun(airtime_component)
    for _ in range(19):
        run.advance(10_000_000)
        run.restart()
    assert run.policy.total_used == 36_000_000 and run.policy.available_charge_us == 0
    next_deadline = min(run.policy._ledger.deadlines)
    wait = next_deadline-run.clock.now_monotonic_us()
    run.advance(wait-1)
    assert run.policy.available_charge_us == 0
    run.advance(1)
    assert run.policy.available_charge_us == 2_000_000
    assert run.sent == 0
    (tmp_path/'restart_storm.json').write_text(json.dumps(dict(
        startups=20, tx=0, occupied_after_last_start=18,
        first_available_after_last_start_monotonic_us=wait,
        earliest_expiration_monotonic_us=next_deadline),indent=2))


def test_idle_and_roundtrip_availability_costs(airtime_component):
    run = PhysicalRun(airtime_component)
    assert run.transmit()
    before = run.policy.used_since_save_us
    run.advance(3_700_000_000)
    assert run.policy.total_used == 0
    assert run.policy.used_since_save_us == before
    assert run.transmit()
    assert run.policy.used_since_save_us == 2*before
    run.save()
    old_lifetime = max(e.remaining_us for e in run.policy.state.entries)
    for _ in range(5):
        run.restart()
    assert max(e.remaining_us for e in run.policy.state.entries) > old_lifetime
