# DS3231 I2C controller faults

This family checks production-adapter completion, helper termination and recovery
when the RTC bus is held low. It needs the physically prepared Pi/DS3231 carrier.
The [findings](RESULTS.md) describe the previously investigated kernel and the
unresolved controller behavior. Changing kernels requires new target execution.

## Prepare the fixture

Start with valid, advancing RTC time and normal Chrony operation. Use the recorded
[boot-cycle result](../boot_cycle/RESULTS.md#reuse) for a matching setup; rerun its
procedure if hardware or OS/kernel differs. Establish the ordinary
[time configuration](../../../../deploy/CHRONY.md) before this test.
Normal Chrony remains running throughout controller injection; the test checks
its process identity and configuration and does not step Linux time.

Stage/hash the current receiver and protocol source in an isolated directory
with its test environment outside `/tmp`. Use a fresh root-owned marked test root
as described in [TESTING.md](../../../../TESTING.md#suite-layout).

Follow the [carrier schematic and fault states](../../../TEST_CARRIER.md).
Verify GPIO17/GPIO27 are unused inputs and inspect GPIO2/GPIO3 pad levels with
`pinctrl get`. Shut down, disconnect every external supply, retain the coin cell,
and fit both `JP_RTC_SCL_FAULT` and `JP_RTC_SDA_FAULT` while unpowered. Reconnect
and verify the devices. The fixture uses open-drain control: it pulls low or
releases a line, never drives it high. No driver unbinding or reset is required.

After operator confirmation, create root-owned `i2c-fault-wiring.json`, not
writable by group/others, in the marked root. The complete required fields are:

```json
{
  "operator_response": "REPLACE with the operator's actual confirmation",
  "boot_id": "REPLACE with the current Linux boot ID",
  "carrier_revision": "receiver-time-r1",
  "fixture_state": "rtc_fault_ready",
  "operator_confirmed": true,
  "spare_gpios_previously_unconnected": true,
  "wired_while_unpowered": true,
  "gpio17_to_scl": true,
  "gpio27_to_sda": true
}
```

Set the booleans only after verifying those actions and connections. Missing or
stale confirmation fails before GPIO or service changes.

## Run the four cases

From the verified stage's repository root on the Pi:

```bash
sudo env PYTHONPATH=receiver:protocol/protocol-v2-lora/python \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest \
  -c receiver/pytest.ini receiver/hardware/ds3231/tests/i2c_faults/test_controller_faults.py \
  -x --receiver-hardware --confirm-receiver-destructive \
  --receiver-test-root=/absolute/path/to/marked-test-root
```

| Case | Required observation |
| --- | --- |
| SCL-low read | Actual in-flight RTC ioctl; failed read without trusted UTC |
| SCL-low write | Pending SIGKILL inside the native write ioctl; unknown outcome/deadline exceeded; helper reaped before fault release |
| SDA-low read | Failed/invalid read without trusted UTC |
| SDA-low write | Retain reported outcome; syscall success alone does not verify RTC contents |

All reported operations must complete in less than 3 seconds. Read calls receive
a 5-second deadline; write calls receive 500 milliseconds to exercise termination.
The capability-free component uses the pinned native helper for writes. Physical
pad levels, real syscalls, durations and restoration are checked by the code.

## Restore and assess

The fixture releases and reaps the GPIO holder and verifies unused input GPIOs.
Release does not guarantee high bus pads. Bounded production read recovery may
start with low pads after confirmed release, but high pads and a valid RTC read
are required before a restoration write. Read cases restore through reads only;
write cases restore once after an authorized or uncertain submission, then verify
read-back. Recovery time is separate from the original operation allowance.

An invalid RTC, failed recovery, low pads or uncertain cleanup stops dependent
cases. Preserve the failed outcome; later operator recovery cannot turn it into
a pass. Restore device permissions, remove the temporary helper, and independently
check RTC reads, normal Chrony synchronization and unchanged process/configuration.

Finally shut down, disconnect external power, open both fault shunts and return
to `nominal`. Reconnect and require a new boot, ordinary RTC bootstrap, and the
safe time component suite on the same verified source stage. Qualification stays
open until physical restoration and software checks pass. Keep source-bound
findings here and dated detailed runs in the logbook; discard temporary output
only after analyzing failures and verifying restoration.
