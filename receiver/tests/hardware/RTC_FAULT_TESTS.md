# DS3231 component fault procedure

This procedure qualifies the production adapter on the installed Pi 3,
`rtc-ds1307` DS3231 driver and bcm2835 I2C controller. Its owner is the runtime
time workload. It follows [the hardware test rules](../../TESTING.md) and
[the existing physical RTC procedure](../../hardware/ds3231/OPERATOR_TESTS.md).
Physical actions belong to the operator; the test supervisor owns commands,
source staging, captures and restoration. Keep the two runs and their source
manifests separate. A prepared fixture is not passing target evidence.
The [receiver test carrier](../../hardware/TEST_CARRIER.md) owns the schematic,
pin allocation and shunt states. Use its `nominal` state for the OSF sequence.

[DS3231 LIMITATION.md](../../hardware/ds3231/LIMITATION.md) records the
controller findings, approved pilot recovery and qualification outcome.

## Oscillator-stop capture and recovery

1. Stage and hash the current receiver/protocol tree in an isolated directory
   under `/home/cura`, with a working test environment that survives reboot.
   Create a dedicated root-owned marked evidence directory under `/var/lib`.
   Record the source manifest, current boot ID, a production-adapter `OK` RTC
   baseline, normal Chrony configuration and synchronization. Preserve the
   original JSON as `osf-baseline.json`.
2. Request shutdown. The operator follows RTC-04: disconnect all external
   Pi/module power, remove the coin cell while unpowered for at least 30 seconds,
   reinstall it correctly, and reconnect power. Record the exact confirmation
   and any deviation. Do not write or recover the RTC before the fault capture.
3. After reconnecting, verify the staged hashes and changed boot ID. Write the
   root-owned `osf-operator-confirmation.json` required by
   `test_ds3231_osf.py`, bound to the baseline SHA256 and new boot ID. Run only
   `test_oscillator_stop_rejected_by_production_adapter` with hardware and
   destructive opt-ins and the marked root. Preserve its pytest output,
   `osf-adapter.json` and current-boot `osf-kernel.txt` before any recovery write.
4. Follow RTC-05 recovery: establish qualified network synchronization, perform
   one recorded RTC write, and verify both the production adapter and an
   independent read. Preserve the write's time bracket and outcome. Run the
   safe time component suite. Never repeat an uncertain recovery write blindly.
5. Complete RTC-05's reference to RTC-03 steps 1–3: shut down, disconnect all
   external power **with the coin cell retained**, and leave it disconnected
   for at least **30 minutes**. The operator may time this interval. Record the
   timing owner and confirmation; message timestamps are not exact physical
   disconnection times. Reconnect power and confirm completion/deviations.
6. Before another RTC write, capture the new boot's kernel RTC bootstrap and
   a production-adapter read. Require a third boot ID, successful RTC bootstrap,
   no OSF rejection, `OK` RTC input, normal Chrony synchronization and unchanged
   configuration/device access. Repeat the safe time suite, collect originals
   with SHA256, and archive the result with its source manifest and physical
   confirmations. Mark completion only after restoration passes.

## Controller faults and in-flight helper termination

The carrier's removable fault shunts join two reserved spare GPIOs to the
existing bus. The fixture uses **open-drain** outputs: it pulls a bus line low
or releases it; it never drives a bus line high. No GPIO production port, kernel
replacement, driver unbinding or controller reset is involved.

### Prepare and connect

1. Complete and preserve the oscillator-stop recovery first. With the Pi
   powered normally, verify the installed controller/RTC identities, system
   Python/libgpiod 2.x, GPIO17/GPIO27 as unused inputs, and read-only `pinctrl get`
   output for GPIO2/GPIO3. Confirm the carrier revision and available removable
   shunts. These GPIO checks belong only to the connected fault fixture.
2. Stage/hash the current tree separately, retain the test environment outside
   `/tmp`, and create a fresh root-owned marked evidence directory. Record the
   normal Chrony/device metadata and current GPIO state before any GPIO drive.
3. Request shutdown. After shutdown, disconnect every external Pi/module power
   source and **leave the coin cell installed**. Assemble the approved carrier
   if necessary, confirm the two spare GPIOs have no other connections, and
   fit both `JP_RTC_SCL_FAULT` and `JP_RTC_SDA_FAULT` to select
   `rtc_fault_ready`. Use the carrier's physical pin table and schematic.
   If the operator has already completed this preparation, retain their
   confirmation and the current boot ID rather than repeat the power cycle.
4. Reconnect Pi power. Record the exact operator confirmation and any deviation.
   After checking the new boot, the supervisor creates root-owned, non-writable
   by group/other `i2c-fault-wiring.json` in the marked root. It contains the
   exact `operator_response`, current `boot_id`, `carrier_revision` equal to
   `receiver-time-r1`, `fixture_state` equal to `rtc_fault_ready`, and these booleans:
   `operator_confirmed`, `spare_gpios_previously_unconnected`,
   `wired_while_unpowered`, `gpio17_to_scl`, `gpio27_to_sda`.
   Missing confirmation fails before service or GPIO changes.

### Execute and restore

Run only `test_time_mutations.py::test_ds3231_controller_fault`, serially with
`-x`, `--receiver-hardware`, `--confirm-receiver-destructive` and the dedicated
`--receiver-test-root`. Verify the source manifest immediately before execution.
There are four cases: SCL-low and SDA-low, each for read and write.

Each case acquires the existing time fixture lock and uses a dedicated RTC-only
fixture, capability-free receiver child and pinned native write helper. Normal
Chrony keeps running: record its process identity and configuration before and
after; do not stop, restart or step it. The GPIO holder
checks that its spare line is an unused input before requesting open-drain
control. The component first reads a valid RTC, then reports readiness. The
supervisor holds the selected line low, verifies the actual GPIO2/3 pad level,
and releases the component to call the production adapter.

The acceptance allowance is **3 seconds** for the complete reported operation.
Reads receive a 5-second deadline. Writes receive a 500-millisecond deadline
to exercise termination while a held-SCL transaction is in progress. Preserve
all actual statuses and durations. The held-SCL cases require observation of
the real AArch64 RTC ioctl through `/proc`; the write additionally requires
pending SIGKILL while the native helper remains in `RTC_SET_TIME`, followed by
its reaping before SCL is released. Failure to observe this is a failed
qualification, not an assumed success.

SDA-low may yield invalid data or false acknowledgements. Its read must not
return trusted UTC; its write outcome is retained without treating a successful
write syscall as verified RTC content. These cases measure the actual bound;
the existing refresh tests own the separate mandatory read-back/provenance rule.

The GPIO holder explicitly releases the line and restores input direction,
preserving the pull configuration with `Bias.AS_IS` on every request. It never
uses the current Pi's reported `Bias.UNKNOWN` as a requested setting.
The supervisor waits for normal holder exit and reaping, then checks both spare
GPIOs are unused inputs before RTC cleanup. Record the bus pad levels; a low
bus after injection release does not prevent read recovery. An open-drain
release removes the injection drive and does not guarantee a high bus level.
Use the production communicator read-recovery
function with its three-second retry window and separate five-second read-call
budget. Record every attempted read and the final recovery result. This phase
is outside the original fault-operation bracket: it cannot repair a failed
operation-bound or termination assertion. A late valid read is deadline failure,
and INVALID stops for explicit network-qualified operator recovery. Require
both bus pads high after recovery and before any restoration write. Check both
pads again after final read-back. Failed recovery or persistently low pads
stop cleanup before a write and stop dependent cases.
Read cases verify the original advancing RTC using only reads. Write cases
require a successful recovery read before explicitly restoring that value once
and verifying read-back through the same read-recovery function;
checkpoint GO authorization before sending it so uncertain submission requires
restoration too. A setup failure before GO does not authorize a cleanup write.
Always restore device permissions and remove the temporary helper. Retain every
`controller-fault.json`, component output and `restoration.json`,
including failures during analysis. The retired controller investigation is
consolidated in LIMITATION.md; its original debug logs and failed-run archives
were removed by explicit user decision. If a bound or cleanup fails, release
the fault, preserve
evidence and stop dependent cases; do not relax the acceptance threshold or
blindly retry a failed restoration. Any subsequent qualified operator recovery
is a separate result and cannot change the failed case into a pass.

After execution, independently check RTC reads, normal synchronization,
unchanged configuration/device metadata, released GPIO inputs and absence of
temporary helper/socket processes. Then request shutdown and have the operator
open **both fault shunts while externally unpowered**, returning the carrier
to `nominal` with its fixed wiring and coin cell retained. Reconnect power and
record confirmation.
Verify the changed boot, ordinary RTC bootstrap and the safe time component
suite on the same verified source stage. Archive source-bound outcomes and
physical restoration; qualification remains open until these checks pass.
