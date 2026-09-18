from cura_receiver.rtc_bootstrap import copy_rtc_once, BootstrapOutcome as O
from cura_receiver.ports.ds3231 import Ds3231ReadResult, Ds3231ReadStatus as R
from cura_receiver.ports.kernel_clock import KernelClockResult, KernelSampleStatus as K
from tests.support.fakes.ds3231 import FakeDs3231Control
from tests.support.fakes.kernel_clock import FakeKernelClock
from tests.support.fakes.os_clock import FakeOsClock


def test_bootstrap_attempt_cannot_repeat_after_copy_failure():
    clock, kernel, rtc = FakeOsClock(), FakeKernelClock(), FakeDs3231Control()
    kernel.results.append(KernelClockResult(K.OK, 0, 0, 1_800_000_000_000_000, 5, 0x2040))
    rtc.read_results.extend([Ds3231ReadResult(R.IO_ERROR, 0, 0, os_errno=5)] * 3)
    claims = []
    def claim():
        claims.append(True)
        return len(claims) == 1
    copied = []
    def run():
        return copy_rtc_once(clock=clock, kernel=kernel, rtc=rtc, claim_attempt=claim, set_system_utc=copied.append)
    assert run() is O.IO_ERROR
    assert len(rtc.calls) == 3
    assert run() is O.ALREADY_ATTEMPTED
    assert len(rtc.calls) == 3 and not copied


def test_synchronized_system_clock_is_never_overwritten():
    clock, kernel, rtc = FakeOsClock(), FakeKernelClock(), FakeDs3231Control()
    kernel.results.append(KernelClockResult(K.OK, 0, 0, 1_800_000_000_000_000, 0, 0x2000))
    copied = []
    assert copy_rtc_once(clock=clock, kernel=kernel, rtc=rtc,
        claim_attempt=lambda: True, set_system_utc=copied.append) is O.ALREADY_SYNCHRONIZED
    assert not copied and not rtc.calls


def test_valid_rtc_copy_is_provisional_and_has_no_storage_dependency():
    clock, kernel, rtc = FakeOsClock(), FakeKernelClock(), FakeDs3231Control()
    kernel.results.append(KernelClockResult(K.OK, 0, 0, 1_800_000_000_000_000, 5, 0x2040))
    rtc.read_results.append(Ds3231ReadResult(R.OK, 0, 0, 1_800_000_000))
    copied = []
    assert copy_rtc_once(clock=clock, kernel=kernel, rtc=rtc,
        claim_attempt=lambda: True, set_system_utc=copied.append) is O.COPIED
    assert copied == [1_800_000_000]


def test_late_successful_read_does_not_copy_or_retry():
    clock, kernel, rtc = FakeOsClock(), FakeKernelClock(), FakeDs3231Control()
    kernel.results.append(KernelClockResult(K.OK, 0, 0, 1_800_000_000_000_000, 5, 0x2040))
    def slow():
        clock.advance_elapsed_us(5_000_000)
        return Ds3231ReadResult(R.OK, 0, 5_000_000, 1_800_000_000)
    rtc.read_results.append(slow)
    copied = []
    assert copy_rtc_once(clock=clock, kernel=kernel, rtc=rtc,
        claim_attempt=lambda: True, set_system_utc=copied.append) is O.DEADLINE_EXCEEDED
    assert len(rtc.calls) == 1 and not copied


def test_boot_claim_survives_process_reopen_and_changes_with_boot(tmp_path, monkeypatch):
    import cura_receiver.rtc_bootstrap as bootstrap
    monkeypatch.setattr(bootstrap, 'read_linux_boot_id', lambda: bytes.fromhex('01' * 16))
    assert bootstrap.claim_boot_attempt(tmp_path)
    assert not bootstrap.claim_boot_attempt(tmp_path)
    monkeypatch.setattr(bootstrap, 'read_linux_boot_id', lambda: bytes.fromhex('02' * 16))
    assert bootstrap.claim_boot_attempt(tmp_path)
    assert len(tuple(tmp_path.iterdir())) == 2
