# DS3231 boot, retention and OSF checks

These four boots check RTC reads, short battery retention, oscillator-stop flag
(OSF) rejection and recovery. Accurate UTC is unnecessary for this check; Chrony
is not used. This checks functional validity and short retention, not drift or
the separate [I2C operation bound](../i2c_faults/README.md#adopted-operation-bound).

Prepare the Pi using the [deployment guide](../../../../deploy/README.md).
Keep the receiver and time synchronization services stopped throughout the run;
no automatic RTC writer may repair the deliberately invalid state. Use the
carrier's [nominal wiring](../../../../HARDWARE.md).
Run commands on the Pi, reconnecting with SSH after each boot.

## Boot 1: initialize and read

Initial OSF state can vary. Copy Linux time to the verified RTC and read it:

```bash
sudo hwclock --systohc --utc --noadjfile --rtc=/dev/rtc0
sudo env TZ=UTC hwclock --show --verbose --utc --noadjfile --rtc=/dev/rtc0
```

Require a successful write and read. This establishes a valid starting state,
without asserting that the copied time is accurate.

## Boot 2: retain time with the battery installed

```bash
sudo poweroff
```

After shutdown completes, unplug every external Pi/module power source. Keep
the coin cell installed, wait 30 seconds, then reconnect power. After boot:

```bash
sudo journalctl -k -b -o short-monotonic --no-pager | grep -E 'ds1307'
sudo env TZ=UTC hwclock --show --verbose --utc --noadjfile --rtc=/dev/rtc0
```

Require an advancing RTC calendar, a successful read, no `SET TIME!` warning,
and `setting system clock to ...` in this boot's kernel log. Do not write the
RTC between boots 1 and 2.

## Boot 3: reject oscillator-stopped time and recover

Shut down with `sudo poweroff`. Disconnect all external power, remove the coin
cell while unpowered, wait 30 seconds, reinstall it, then reconnect power.
Run boot 2's journal and read commands **before any RTC write**.

Require `SET TIME!`, `hctosys: unable to read the hardware clock`, and a failed
time read with `Invalid argument` (`EINVAL`). A missing device, permission
error or I2C transport error does not establish OSF rejection.

Now repeat boot 1's write and read commands. Require successful read-back:
setting time must clear the invalid state.

## Boot 4: recovery survives the next cold boot

Repeat boot 2's shutdown, 30-second external-power removal with the battery
retained, and journal/read commands. Require the same successful outcomes.

`RTC_UIE_ON: Invalid argument` followed by a successful polled read is allowed.
util-linux 2.41.5 prints `RTC_RD_NAME` for the failed `RTC_RD_TIME` call.
Its reported zero adjustment-file drift is not a measured oscillator drift.

## Optional production-adapter check

[test_ds3231_osf.py](test_ds3231_osf.py) verifies that the production adapter
reports `INVALID`/`EINVAL` without UTC after boot 3's physical fault. The manual
four boots verify the kernel behavior; they do not execute this adapter test.
Its explicit hardware/destructive opt-ins and source-bound inputs are retained.

On an isolated current-source stage, use a fresh root-owned marked test directory
as described in [the shared test guide](../../../../tests/README.md#hardware-and-destructive-runs). Before boot 3,
create the actual adapter baseline with [prepare_baseline.py](prepare_baseline.py):

```bash
sudo env PYTHONPATH=receiver:protocol/protocol-v2-lora/python \
  .venv/bin/python receiver/hardware/ds3231/tests/boot_cycle/prepare_baseline.py \
  /absolute/path/to/marked-test-root
```

After boot 3's fault, create root-owned `osf-operator-confirmation.json` in that
directory. It must contain `baseline_sha256` (SHA-256 of `osf-baseline.json`),
the new boot ID as `after_boot_id`, and these booleans, true only after the
operator has confirmed each action:

```json
{
  "all_external_power_disconnected": true,
  "coin_cell_removed_while_unpowered": true,
  "coin_cell_reinstalled_before_power": true,
  "no_rtc_write_since_fault": true
}
```

The JSON above lists the action fields; add the digest and boot-ID fields.
Before recovering the RTC, run only the adapter test from the repository root:

```bash
sudo env PYTHONPATH=receiver:protocol/protocol-v2-lora/python \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest \
  -c receiver/pytest.ini receiver/hardware/ds3231/tests/boot_cycle/test_ds3231_osf.py \
  --receiver-hardware --confirm-receiver-destructive \
  --receiver-test-root=/absolute/path/to/marked-test-root
```

The test retains its adapter result and current-boot kernel evidence in the
marked root. Complete boot 3 recovery and boot 4 afterward. Discard temporary
run files after checking outcomes and restoration; keep useful findings here.
Initialize accurate RTC time through the [deployment time procedure](../../../../deploy/CHRONY.md#initialize-rtc-time-before-deployment)
before production startup. Detailed dated runs belong in the logbook.

## Driver behavior

The DS3231 driver must reject oscillator-stopped reads and clear OSF when setting
time. Older Pi kernels lacked these behaviors; the
[upstream correction](https://github.com/torvalds/linux/commit/a091e1ba3b68cabc9caedafc6f81d9fe9b3b2200)
explains the requirement. The supported stable Pi package `1:6.18.50-1+rpt1`
includes it; a beta kernel is not needed. Recheck the four boots after relevant
hardware or OS/kernel changes, before synchronization or another writer can
repair the deliberately invalid initial state. Do not infer controller-fault
recovery or a complete-operation bound from OSF success.
