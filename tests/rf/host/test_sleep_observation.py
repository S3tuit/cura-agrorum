import pytest
from sleep_observation import sleep_count, verify_wake_readings
from test_apps.radio_peer.ack_cases import AckCase, episode


def marker(index, *, reason=None, nonce=None):
    reason = (1 if index == 1 else 8) if reason is None else reason
    nonce = f"{index:08x}" if nonce is None else nonce
    return (f"\x1eRF_NODE_SLEEP boot={index:016x} reset_reason={reason} "
            f"duration_us=10000000 finalize={nonce}\r\n").encode()


def greeting(index):
    return f"RF_FINALIZE {index:08x}\n\x1f\n".encode()


def uart(wakes=3, *, parked=True):
    raw = b''.join(marker(i) + greeting(i) for i in range(1, wakes + 1))
    return raw + (f"\x1eRF_PARKED {wakes:08x}\n\x1f\n".encode() if parked else b'')


def test_complete_and_fragmented_uart_without_rom_banners():
    assert sleep_count(uart(), 10, 3) == 3
    assert sleep_count(marker(1) + greeting(1)[:-2], 10, 3) == 0
    assert sleep_count(marker(1) + greeting(1), 10, 3) == 1
    assert len(verify_wake_readings(uart(), 10, 3, [1, 8, 8])) == 3


def test_identical_regreetings_count_once_and_recover_late_capture():
    repeated = (marker(1) + greeting(1)) * 3
    assert sleep_count(repeated, 10, 3) == 1
    assert sleep_count(repeated + marker(2) + greeting(2), 10, 3) == 2
    # The first emitted greeting may be lost during USB re-enumeration.
    assert sleep_count(marker(1) + greeting(1), 10, 3) == 1


@pytest.mark.parametrize('raw', [
    uart(4), uart().replace(b'reset_reason=8', b'reset_reason=3', 1),
    uart().replace(b'10000000', b'900000000', 1), uart() + b'panic\n',
    marker(1, reason=8) + greeting(1), marker(1, reason=256) + greeting(1),
    marker(1) + marker(2) + greeting(2), marker(1) + greeting(2),
    marker(1) + greeting(1) + marker(1, reason=3) + greeting(1),
    marker(1) + greeting(1) + marker(1, nonce='deadbeef') + greeting(1),
    marker(1) + greeting(1) + marker(2, nonce='00000001') + greeting(1),
    uart(2, parked=False) + marker(1) + greeting(1),
    uart() + marker(4) + greeting(4), b'\x1eRF_PARKED 00000001\n\x1f',
    uart().replace(b'RF_PARKED 00000003', b'RF_PARKED deadbeef'),
    b'\x1eRF_FINALIZE_FAILED 00000001\n\x1f',
    b'\x1eRF_NODE_SLEEP duration_us=10000000\n\x1f',
])
def test_invalid_uart(raw):
    with pytest.raises(ValueError):
        sleep_count(raw, 10, 3)


@pytest.mark.parametrize('cut', range(1, len(marker(1) + greeting(1))))
def test_late_usb_capture_resynchronizes_at_every_fragment_boundary(cut):
    frame = marker(1) + greeting(1)
    assert sleep_count(frame[:cut] + frame, 10, 3) == 1
    assert sleep_count(frame[cut:] + frame, 10, 3) == 1


def test_incomplete_final_record_cannot_finish_or_acknowledge_a_wake():
    assert sleep_count(marker(1), 10, 3) == 0
    with pytest.raises(ValueError, match='missing terminal park'):
        verify_wake_readings(uart(parked=False) + b'\x1eRF_PARKED 00000003\n', 10, 3, [1, 8, 8])


@pytest.mark.parametrize('raw,reasons', [
    (uart(), [11, 8, 8]), (uart(), [1, 8]),
    (uart(parked=False), [1, 8, 8]), (uart(2), [1, 8, 8]),
])
def test_final_reconciliation_requires_matching_readings_and_park(raw, reasons):
    with pytest.raises(ValueError):
        verify_wake_readings(raw, 10, 3, reasons)


def test_accelerated_completion_requires_bound_final_marker():
    p = AckCase('node.current.accepted', bytes(8), bytes(16), 10)
    assert p.plan['lease_seconds'] == 160
    assert not p.complete(10**12)
    command = 'SLEEP ' + 'a'*32 + ' node.current.accepted 3\n'
    with pytest.raises(ValueError):
        p.observe_sleep(command, 'a'*32, 'node.current.accepted')
    p.currents = [{}, {}, {}]
    p.observation_started_at = 123
    with pytest.raises(ValueError):
        p.observe_sleep(command, 'b'*32, 'node.current.accepted')
    p.observe_sleep(command, 'a'*32, 'node.current.accepted')
    assert p.complete(124)
    with pytest.raises(ValueError):
        p.observe_sleep(command, 'a'*32, 'node.current.accepted')


def test_production_completion_keeps_original_bound():
    p = AckCase('node.current.accepted', bytes(8), bytes(16))
    p.observation_started_at = 100
    assert not p.complete(35000099)
    assert p.complete(35000100)
    with pytest.raises(ValueError):
        episode('node.current.accepted', 11)
