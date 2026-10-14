# DS3231 I2C controller faults

This family checks production-adapter completion, helper termination and recovery
when the RTC bus is held low. It needs the physically prepared Pi/DS3231 carrier.
The recovery notes below explain the controller limitations and adopted operation bound.

## Fault wiring

Keep the [production schematic](../../../../HARDWARE.md) assembled. Add only
these removable **parallel branches**, never series breaks in the I2C bus:

```text
RTC_SCL (Pi pin 5 / GPIO3) -- JP_RTC_SCL_FAULT -- Pi pin 11 / GPIO17
RTC_SDA (Pi pin 3 / GPIO2) -- JP_RTC_SDA_FAULT -- Pi pin 13 / GPIO27
```

GPIO17/GPIO27 have no other connections. Fit both shunts while externally
unpowered and retain the coin cell. Open drain LOW asserts a fault; release
removes the drive. Never drive a connected fault pin push-pull high or add a
5 V pull-up. The test requests `Bias.AS_IS`; `Bias.UNKNOWN` is an observation,
not permission to change pulls.

| State | Shunts | GPIO17 | GPIO27 |
| --- | --- | --- | --- |
| `nominal` | Both open | Outside fixture ownership | Outside fixture ownership |
| `rtc_fault_ready` | Both fitted | Input/released | Input/released |
| `rtc_scl_low` | Both fitted | Open-drain LOW | Input/released |
| `rtc_sda_low` | Both fitted | Input/released | Open-drain LOW |

Before assertion, both spare GPIOs must be unused inputs and both bus pads high.
Between cases restore `rtc_fault_ready`; after the suite remove both shunts
unpowered and verify nominal operation. GPIO release and bus recovery are
separate checks, as described under [restoration](#restore-and-assess).

## Prepare the fixture

Start with valid, advancing RTC time and normal Chrony operation. Use the [boot-cycle procedure](../boot_cycle/README.md) when changes to the
hardware or OS/kernel require rechecking validity and retention. Establish the ordinary
[time configuration](../../../../deploy/CHRONY.md) before this test.
Normal Chrony remains running throughout controller injection; the test checks
its process identity and configuration and does not step Linux time.

Stage/hash the current receiver and protocol source in an isolated directory
with its test environment outside `/tmp`. Use a fresh root-owned marked test root
as described in [the shared test guide](../../../../tests/README.md#hardware-and-destructive-runs).

Follow the [fault wiring](#fault-wiring) above.
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

## Recovery lessons

On the Pi 3 bcm2835 controller with the stock `rtc-ds1307` driver, releasing
held-low SCL can leave subsequent reads failing. Timeout cleanup can leave stale
completion/error state; repeated reads can recover without a write or power cycle,
but no fixed retry count is guaranteed. GPIO release and bus recovery are separate
facts. Low pads after release do not forbid bounded read recovery; require high
pads and a valid read before a restoration write, then verify both again.

Runtime recovery uses repeated single reads within its three-second window plus
the separate final in-flight-operation bound. Only a successful read's own bracket
contributes UTC uncertainty, and a late result is unusable. Stop ordinary recovery
on `INVALID`; `EINVAL` alone does not identify OSF. Network-qualified operator
recovery is explicit. Never blindly repeat an uncertain write or treat a kill
request as a reaped helper. Controller timeout/interrupt recovery remains a driver
limitation; longer waits or a successful boot-cycle test do not resolve it.

## Adopted operation bound

`RTC_KERNEL_BOUND_US=3000000` is the adopted complete-operation bound for Raspberry
Pi 3 Model B Rev 1.2, the Adafruit DS3231/CR1220 production wiring, and stock
`rtc-ds1307`/`i2c-bcm2835` on Debian 13.7 arm64 with kernel
`6.18.50+rpt-rpi-v8`, package `1:6.18.50-1+rpt1`. The helper/adapter build and
fault setup must also match the applicable measurements in the
[logbook record](https://github.com/S3tuit/cura-agrorum-logbook/tree/main/experiments/2026-10-ds3231-operation-bound).

The bound includes helper termination; it is not just a caller timeout or an
average operation time. Reassess after controller, RTC, wiring, kernel/driver,
or helper/adapter changes. It is not a universal kernel worst-case guarantee,
RTC drift measurement, or installed-service/RF qualification. Dated measurements
and source/build identities stay in the logbook; the procedure and recovery
rationale stay here.
