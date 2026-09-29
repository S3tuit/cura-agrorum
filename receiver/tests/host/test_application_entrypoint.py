"""CLI inputs must reach the real adapter boundary in its contracted type."""

from types import SimpleNamespace
from unittest.mock import Mock, MagicMock

import pytest

from cura_receiver import __main__ as entry


@pytest.mark.parametrize('digest', ['', '0' * 63, '0' * 65, 'g' * 64,
                                   'A' * 64, '00 ' * 32, '0x' + '0' * 64])
def test_invalid_helper_digest_fails_before_device_construction(monkeypatch, capsys, digest):
    monkeypatch.setattr('sys.argv', ['receiver', '--rtc-helper-sha256', digest,
                                   '--rtc-kernel-bound-us', '3000000'])
    clock = Mock(side_effect=AssertionError('clock constructed before input rejection'))
    monkeypatch.setattr(entry, 'LinuxOsClock', clock)
    with pytest.raises(SystemExit) as result:
        entry.main()
    assert result.value.code == 2
    clock.assert_not_called()
    error = capsys.readouterr().err
    assert 'helper digest must be' in error
    if digest:
        assert digest not in error


def test_valid_cli_passes_exact_digest_bytes_through_successful_composition(monkeypatch):
    digest = bytes(range(32))
    monkeypatch.setattr('sys.argv', ['receiver', '--rtc-helper-sha256', digest.hex(),
                                   '--rtc-kernel-bound-us', '3000000'])
    monkeypatch.setattr(entry.os, 'environ', {})
    clock = SimpleNamespace(now_monotonic_us=lambda: 0)
    monkeypatch.setattr(entry, 'LinuxOsClock', lambda: clock)
    monkeypatch.setattr(entry, 'create_receiver_instance', Mock(return_value=object()))
    rtc = Mock(return_value=object())
    monkeypatch.setattr(entry, 'LinuxDs3231Control', rtc)
    for name in ('LinuxChronyControl', 'PersistenceWorker', 'LinuxRadioIo',
                 'Sx1262', 'Radio', 'LinuxKernelClock'):
        monkeypatch.setattr(entry, name, Mock(return_value=object()))
    application = Mock()
    application.start.return_value = SimpleNamespace(ready=True)
    application.run.return_value = 0
    application.shutdown.return_value = SimpleNamespace(failure=None)
    monkeypatch.setattr(entry, 'ReceiverApplication', Mock(return_value=application))
    stop = Mock()
    stop.is_requested.return_value = True
    monkeypatch.setattr(entry, 'StopIntent', Mock(return_value=stop))
    waiter = MagicMock()
    monkeypatch.setattr(entry, 'LinuxSignalWait', Mock(return_value=waiter))
    assert entry.main() == 0
    rtc.assert_called_once_with(clock, kernel_operation_bound_us=3000000,
                                helper_sha256=digest, receiver_gid=entry.os.getegid())
    application.shutdown.assert_called_once_with(clean_requested=True)
    application.run.assert_called_once_with(wait=waiter.__enter__.return_value)
    entry.LinuxSignalWait.assert_called_once_with(clock, stop)
    assert entry.ReceiverApplication.call_args.kwargs['stop_intent'] is stop
    assert entry.Radio.call_args.kwargs['stop_requested'] == stop.is_requested
    waiter.__exit__.assert_called_once_with(None, None, None)
    assert entry.os.environ['SQLITE_TMPDIR'] == '/var/lib/cura-agrorum/tmp'
