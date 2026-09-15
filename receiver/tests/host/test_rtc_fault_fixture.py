"""The fault holder's subprocess protocol distinguishes release from bus level."""

import pytest

from tests.hardware import test_time_mutations as fixture


# Released-low replies must let the real child finish instead of killing its cleanup.
@pytest.mark.parametrize("released_value", ["ACTIVE", "INACTIVE"])
def test_gpio_holder_release_does_not_require_high(monkeypatch, released_value):
    program = '''
import json, sys
print('READY', flush=True)
for line in sys.stdin:
    command = line.strip()
    value = 'INACTIVE' if command == 'LOW' else RELEASED_VALUE
    print(json.dumps({'command': command, 'value': value}), flush=True)
    if command == 'EXIT':
        break
'''.replace("RELEASED_VALUE", repr(released_value))
    monkeypatch.setattr(fixture, "GPIO_HOLDER", program)
    evidence = {"gpio_commands": []}
    with fixture.bus_fault(17, evidence) as send:
        send("LOW")
        send("RELEASE")
    assert evidence["gpio_exit"] == 0 and evidence["gpio_stderr"] == ""
    assert evidence["gpio_commands"] == [
        {"command": "RELEASE", "value": released_value},
        {"command": "LOW", "value": "INACTIVE"},
        {"command": "RELEASE", "value": released_value},
        {"command": "EXIT", "value": released_value},
    ]


# A failed LOW command remains a failure, with normal child exit rather than masked cleanup.
def test_gpio_holder_still_requires_low_injection(monkeypatch):
    monkeypatch.setattr(fixture, "GPIO_HOLDER", '''
import json, sys
print('READY', flush=True)
for line in sys.stdin:
    command = line.strip()
    print(json.dumps({'command': command, 'value': 'ACTIVE'}), flush=True)
    if command == 'EXIT':
        break
''')
    evidence = {"gpio_commands": []}
    with pytest.raises(AssertionError):
        with fixture.bus_fault(17, evidence) as send:
            send("LOW")
    assert evidence["gpio_exit"] == 0
    assert evidence["gpio_commands"][-1] == {"command": "EXIT", "value": "ACTIVE"}
