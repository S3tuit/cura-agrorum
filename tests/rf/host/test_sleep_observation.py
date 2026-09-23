import pytest
from sleep_observation import sleep_count
from test_apps.radio_peer.ack_cases import AckCase, episode


def uart(wakes=3):
    return b''.join((b'rst:0x1 (POWERON)\n' if i == 0 else b'rst:0x5 (SLEEP_WAKEUP)\n') +
                    b'RF_NODE_SLEEP duration_us=10000000\r\n' for i in range(wakes))


def test_complete_and_fragmented_uart():
    raw = uart()
    assert sleep_count(raw, 10, 3) == 3
    assert sleep_count(raw[:-1], 10, 3) == 2


@pytest.mark.parametrize('raw', [uart(4), uart().replace(b'SLEEP_WAKEUP', b'POWERON', 1),
    uart().replace(b'10000000', b'900000000', 1), uart()+b'RF_NODE_SLEEP duration_us=10000000\n',
    uart()+b'panic\n', uart().replace(b'RF_NODE_SLEEP duration_us=10000000\r\n', b'', 1)])
def test_invalid_uart(raw):
    with pytest.raises(ValueError):
        sleep_count(raw, 10, 3)


def test_accelerated_completion_requires_bound_final_marker():
    p = AckCase('RF-019.current.accepted', bytes(8), bytes(16), 10)
    assert p.plan['lease_seconds'] == 160
    assert not p.complete(10**12)
    command = 'SLEEP ' + 'a'*32 + ' RF-019.current.accepted 3\n'
    with pytest.raises(ValueError):
        p.observe_sleep(command, 'a'*32, 'RF-019.current.accepted')
    p.currents = [{}, {}, {}]
    p.observation_started_at = 123
    with pytest.raises(ValueError):
        p.observe_sleep(command, 'b'*32, 'RF-019.current.accepted')
    p.observe_sleep(command, 'a'*32, 'RF-019.current.accepted')
    assert p.complete(124)
    with pytest.raises(ValueError):
        p.observe_sleep(command, 'a'*32, 'RF-019.current.accepted')


def test_production_completion_keeps_original_bound():
    p = AckCase('RF-019.current.accepted', bytes(8), bytes(16))
    p.observation_started_at = 100
    assert not p.complete(35000099)
    assert p.complete(35000100)
    with pytest.raises(ValueError):
        episode('RF-019.current.accepted', 11)
