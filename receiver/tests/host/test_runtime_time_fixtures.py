import sqlite3

import pytest

from cura_receiver.ports.ds3231 import Ds3231ReadResult, Ds3231ReadStatus
from tests.hardware import test_runtime_time as fixture
from tests.support.fakes.ds3231 import FakeDs3231Control
from tests.support.fakes.os_clock import FakeOsClock


# F-005: run the actual offline fixture with a moving read bracket and real lifecycle/observation persistence.
@pytest.mark.parametrize("proven", [False, True])
def test_offline_fixture_observation_follows_instance(tmp_path, monkeypatch, proven):
    clock = FakeOsClock(monotonic_us=1000, realtime_us=0)
    rtc = FakeDs3231Control()

    def probe():
        start = clock.now_monotonic_us()
        clock.advance_elapsed_us(1000)
        return Ds3231ReadResult(
            Ds3231ReadStatus.OK, start, clock.now_monotonic_us(), 1_789_200_000
        )

    rtc.read_results.append(probe)
    monkeypatch.setattr(fixture, "LinuxOsClock", lambda: clock)
    monkeypatch.setattr(fixture, "rtc_port", lambda unused: rtc)
    fixture.test_offline_component_startup(tmp_path, proven)
    with sqlite3.connect(tmp_path / "worker.db") as db:
        start = db.execute(
            "SELECT started_at_monotonic_us FROM receiver_instances"
        ).fetchone()[0]
        observed, utc = db.execute(
            "SELECT sampled_at_monotonic_us, sampled_at_utc_us FROM clock_observations"
        ).fetchone()
    assert start == 1000
    assert observed == (1500 if proven else 2000)
    assert (utc is not None) == proven
    assert observed >= start
    assert [call[0] for call in rtc.calls] == ["read"]
