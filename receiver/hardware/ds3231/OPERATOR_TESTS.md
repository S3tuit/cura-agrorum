# DS3231 operator acceptance procedure

This procedure checks physical retention, oscillator-stop rejection, and
recovery on the Pi/Adafruit fixture in the [installation guide](README.md).
It is a manual hardware procedure, not a pytest test target. Commands run on
the Pi unless explicitly marked as laptop commands.

RTC-02, RTC-03 and RTC-06 use bracketed reads against network-synchronized
Linux UTC as best-effort functional checks. They can estimate a change in
RTC offset with stated uncertainty; they do not certify the chip's specified
drift. The datasheet accuracy remains a design assumption.

An operator removes power and the coin cell, records those actions against
an independent clock, and judges the evidence. A pytest process on this Pi
cannot continue while its own power is absent or establish that a human
removed a battery. Future automation can collect/check evidence around these
actions, but must retain explicit operator confirmation and recovery checks.
The stage-9 production adapter, privilege, timeout, provenance and policy
tests still belong in the [automated receiver suite](../../TESTING.md#time-policy-and-timestamp-analysis).

## Scope, outcomes and prerequisites

Use a bench Pi without a running receiver workload or another application
depending on uninterrupted UTC. RTC-02 and RTC-05 step Linux time and overwrite
RTC time; RTC-03 steps Linux only after reboot, and RTC-06 repeats the
initialization/retention sequence. RTC-04 deliberately destroys retained time.
Physical power changes must follow `sudo poweroff` and completion of shutdown.
Disconnect all supplies, including any USB/GPIO connection capable of
back-powering the Pi or module.

Before starting, verify the installed driver/package and time-writer
configuration using the installation guide. Leave the normal Chrony
configuration in place with no automatic clock steps or RTC writes. Do not
add `rtcsync`, an automatic `hwclock` job, or a startup RTC-setting command to
make the tests pass. Do not change SDA/SCL or battery wiring while powered.

Use these outcomes for each test:

- **PASS:** every stated acceptance criterion has supporting evidence.
- **FAIL:** an observed result contradicts a criterion.
- **INCONCLUSIVE:** an action, independent reference, or required capture is
  missing or ambiguous; state exactly what is missing.
- **NOT RUN:** no attempt was made.

Existing September results predate this procedure. Their qualitative pass
does not retroactively satisfy new measurement or evidence requirements.
See the [results index](results/README.md) for archived runs and their limits.

## Evidence setup

For the default procedure below, connect from the laptop to the Pi:

**Inside that SSH session, on the Pi**, create the capture directory:

```bash
export RTC_RUN="$HOME/ds3231-runs/2026-09-12-run-01"
mkdir -p "$RTC_RUN"
```

Use your existing run's directory when resuming. `RTC_RUN` must be set in
the same Pi shell that calls the capture functions; setting it on the laptop
or in an earlier SSH session does not set it here. The capture file is opened
by your Pi shell before `sudo` runs, so the directory must be writable by your
SSH user. An error mentioning `/rtc02-baseline.txt` means the directory prefix
is missing: repeat this setup, then retry the capture without another RTC write.

Create a new run ID if these files already exist. In that same Pi shell,
define the following Bash function. Repeat the `export` and function definition
after every reconnect. A function defined on the laptop is not transferred
to the Pi by SSH. In this default arrangement, both the commands and the
capture files are on the Pi. The function records the command's exit status,
including an expected failure, and refuses overwrites.

```bash
rtc_capture() {
    if [ -z "${RTC_RUN:-}" ]; then
        printf 'RTC_RUN is unset or empty. Export your run directory in this Pi shell first.\n' >&2
        return 2
    fi
    if [ ! -d "$RTC_RUN" ] || [ ! -w "$RTC_RUN" ] || [ ! -x "$RTC_RUN" ]; then
        printf 'RTC_RUN must be an existing directory writable/searchable by your SSH user: %s\n' "$RTC_RUN" >&2
        return 2
    fi
    local rtc_label="$1"
    shift
    local rtc_file="$RTC_RUN/$rtc_label.txt"
    if [ -e "$rtc_file" ]; then
        printf 'Refusing to overwrite %s\n' "$rtc_file" >&2
        return 1
    fi
    local rtc_rc=0
    "$@" > "$rtc_file" 2>&1 || rtc_rc=$?
    printf '\ncommand_exit=%s\n' "$rtc_rc" >> "$rtc_file"
    cat "$rtc_file"
    return "$rtc_rc"
}
```

For an alternative that saves captures directly on the laptop, see
[collecting Pi output from the laptop](#optional-collect-pi-output-from-the-laptop).
The unwrapped `rtc_capture ... sudo ...` examples below assume the Pi shell.

### Shared bracketed capture

Also define this function **on the Pi**, in the same shell as `rtc_capture`.
Repeat both definitions after reconnecting. `rtc_snapshot LABEL` creates
`$RTC_RUN/LABEL.txt`, preserving both tracking reports, the UTC bracket, the
verbose RTC read and exit statuses. It only reads the clocks.

```bash
rtc_snapshot() {
    rtc_capture "$1" sudo -n env LC_ALL=C TZ=UTC bash -c '
printf "tracking_before\n"
chronyc -h /run/chrony/chronyd.sock tracking || exit "$?"
date -u "+before_epoch=%s.%N before_utc=%Y-%m-%dT%H:%M:%S.%N%z" || exit "$?"
hwclock --show --verbose --utc --noadjfile --rtc=/dev/rtc0
rtc_status=$?
date -u "+after_epoch=%s.%N after_utc=%Y-%m-%dT%H:%M:%S.%N%z"
date_status=$?
printf "hwclock_exit=%s\nafter_date_exit=%s\ntracking_after\n" "$rtc_status" "$date_status"
chronyc -h /run/chrony/chronyd.sock tracking
tracking_status=$?
printf "tracking_after_exit=%s\n" "$tracking_status"
if [ "$rtc_status" -ne 0 ]; then exit "$rtc_status"; fi
if [ "$date_status" -ne 0 ]; then exit "$date_status"; fi
exit "$tracking_status"
'
}
```

Run `sudo -v` immediately before invoking it, in the same Pi SSH session.
This refreshes sudo authentication without running an unrelated command such
as `sudo ls ~`. The capture uses one `sudo -n` invocation: an unavailable
credential fails immediately instead of prompting between timestamps.

The timestamps come from `date -u` after Chrony has corrected Linux.
Chrony's `Ref time (UTC)` identifies its last reference measurement, not the
current time. Keep tracking outside the two `date` calls so those calls
bracket only the RTC command and shell overhead. This is sequential, not an
atomic measurement; `hwclock` may wait for a tick. See the
[Chrony tracking documentation](https://chrony-project.org/doc/4.6.1/chronyc.html#tracking).

### Run identity

Do not paste the entire procedure as an unattended script: inspect the
results at each acceptance point. Capture initial identity:

```bash
rtc_capture identity bash -c '
uname -a
cat /etc/os-release
cat /proc/device-tree/model
printf "\n"
dpkg-query -W linux-image-6.18.50+rpt-rpi-v8 chrony util-linux util-linux-extra
cat /sys/class/rtc/rtc0/name
readlink -f /sys/class/rtc/rtc0/device
readlink -f /sys/class/rtc/rtc0/device/driver
'
rtc_capture chrony-config sudo cat /etc/chrony/chrony.conf
rtc_capture chrony-options sudo cat /etc/default/chrony
rtc_capture services systemctl --no-pager --full status \
  chrony.service systemd-timesyncd.service
```

If testing another package, replace the versioned package name and record that
choice.

## RTC-01: baseline device and initial state

Before setting either clock, run:

```bash
rtc_capture rtc01-boot-id cat /proc/sys/kernel/random/boot_id
rtc_capture rtc01-kernel sudo journalctl -k -b -o short-monotonic --no-pager | grep -E 'ds1307'
rtc_capture rtc01-read sudo env TZ=UTC hwclock \
  --show --verbose --utc --noadjfile --rtc=/dev/rtc0
```

**Acceptance:** the expected DS3231 is bound as the verified RTC path and its
initial state is captured. Record either a successful calendar read or the
OSF warning plus failed RTC read as the starting state. An uninitialized RTC
with OSF is not a wiring failure. A missing/wrong device or I2C transfer error
fails this precondition; resolve it before time writes. `EINVAL` alone does
not identify OSF uniquely: use the boot warning and controlled test sequence.

## RTC-02: qualified write and read-back

1. Ensure RTC-01's initial evidence is saved. Capture network status:

   ```bash
   rtc_capture rtc02-sources sudo chronyc -h /run/chrony/chronyd.sock sources -v
   rtc_capture rtc02-tracking-before sudo chronyc -h /run/chrony/chronyd.sock tracking
   ```

2. Require a selected network source (`^*`), recent reference time consistent
   with independent UTC, and normal synchronized status. If these are absent,
   do not write the RTC. A large `System time` correction must be resolved even
   if `Leap status` is `Normal`.

3. With no time-sensitive workload active, run this conditional sequence.
   This is the **one RTC write** before the retention comparison:

   ```bash
   sudo -v &&
   rtc_capture rtc02-step sudo -n chronyc -h /run/chrony/chronyd.sock makestep &&
   rtc_capture rtc02-wait sudo -n chronyc -h /run/chrony/chronyd.sock waitsync 30 0.1 0 2 &&
   rtc_capture rtc02-write sudo -n hwclock --systohc --utc --noadjfile --rtc=/dev/rtc0
   ```

4. Require each command to succeed, then capture the bracketed read-back
   into the baseline file:

   ```bash
   sudo -v && rtc_snapshot rtc02-baseline
   ```

**Acceptance:** network qualification, step, bounded wait, one RTC write, and
bracketed read-back all succeed. Both tracking reports must show a recent
network reference, `Leap status: Normal`, and less than 0.1 seconds absolute
`System time` correction. `waitsync` bounds the remaining Linux correction,
not the entire UTC reference error; this is a bench criterion, not a receiver
runtime trust policy. See [Chrony waitsync](https://chrony-project.org/doc/4.6.1/chronyc.html#waitsync).

Calculate the baseline offset interval using
[the comparison method below](#interpreting-the-bracketed-captures). It must
be consistent with zero offset within the stated uncertainty. A `RTC_UIE_ON`
warning followed by a successful polled read is allowed; a failed RTC read
is not. The successful fixed-driver read shows OSF is clear at that read.
Preserve failed/uncertain writes without retrying blindly; re-establish
qualification before another attempt under a new label. Keep the accepted
baseline unchanged and do not write the RTC again before the after-poweroff
capture, including during shutdown and startup.

## RTC-03: battery retention and cold boot

1. Start after RTC-02 passes, with the battery installed. Use
   `rtc02-baseline.txt` as the before-poweroff measurement. No RTC writes may
   occur between this measurement and step 4 below. Save the current boot ID,
   then shut down:

   ```bash
   rtc_capture rtc03-before-boot-id cat /proc/sys/kernel/random/boot_id
   sudo poweroff
   ```

2. After shutdown completes, unplug Pi power. Leave the DS3231 and coin cell
   untouched for 30 minutes. This duration is a bench-test choice, not a chip
   spec.

3. Restore power. Reconnect SSH, restore `RTC_RUN`, `rtc_capture` and
   `rtc_snapshot`, and save the boot evidence and first RTC read before
   manually correcting Linux. Do not write the RTC:

   ```bash
   rtc_capture rtc03-after-boot-id cat /proc/sys/kernel/random/boot_id
   rtc_capture rtc03-after-kernel sudo journalctl -k -b -o short-monotonic --no-pager | grep -E 'ds1307'
   rtc_capture rtc03-after-chrony sudo journalctl -u chrony.service -b -o short-monotonic --no-pager
   rtc_capture rtc03-after-read sudo env TZ=UTC hwclock \
     --show --verbose --utc --noadjfile --rtc=/dev/rtc0
   ```

4. Re-establish a network reference. Linux just bootstrapped from the RTC,
   so agreement between them before network correction is circular evidence.
   Capture and inspect:

   ```bash
   rtc_capture rtc03-sources sudo chronyc -h /run/chrony/chronyd.sock sources -v
   rtc_capture rtc03-tracking-before sudo chronyc -h /run/chrony/chronyd.sock tracking
   ```

   Require the selected network source, recent reference and normal status
   from RTC-02 step 2. Then correct **Linux only**, wait for synchronization
   and run the same capture into a different file:

   ```bash
   sudo -v &&
   rtc_capture rtc03-step sudo -n chronyc -h /run/chrony/chronyd.sock makestep &&
   rtc_capture rtc03-wait sudo -n chronyc -h /run/chrony/chronyd.sock waitsync 30 0.1 0 2 &&
   rtc_snapshot rtc03-after-poweroff
   ```

   There is deliberately no `hwclock --systohc` here. If any manual or
   automatic RTC write occurred after the baseline, this retention comparison
   is invalid; record it as inconclusive and start a new run.

5. Compare `rtc02-baseline.txt` with `rtc03-after-poweroff.txt` using
   [the method below](#interpreting-the-bracketed-captures). Preserve the boot
   read separately: it documents initial validity, while the bracketed
   network-qualified read provides the after-poweroff offset estimate.

**Acceptance:** a new boot ID, recorded physical outage with battery retained,
successful RTC read, no OSF warning in that fresh boot, and successful
RTC-to-Linux bootstrap before network synchronization. RTC time must advance
without a reset or frozen clock. Both bracketed captures must meet RTC-02's
reference/read criteria; record the offset change and its uncertainty. A
resolved small offset change is a measurement to report, not automatically
a failure against an undeclared ppm limit. Investigate discrepancies that
suggest lost time or an intervening writer before accepting retention.

If reference quality, timing uncertainty or continuity cannot be established,
mark the comparison inconclusive even if the qualitative boot check passes.
The interval between samples includes powered time before shutdown and after
startup; it is not the exact physical outage duration or a measurement solely
of the battery-powered oscillator rate.

## RTC-04: total power loss and OSF rejection

1. Start from a passing RTC read with OSF clear, then:

   ```bash
   rtc_capture rtc04-before-read sudo env TZ=UTC hwclock \
     --show --verbose --utc --noadjfile --rtc=/dev/rtc0
   sudo poweroff
   ```

2. After shutdown, unplug every external source of Pi/module power, then
   remove the CR1220. Record both actions. Allow supplies to discharge (use
   30 seconds as a practical pause; this is not a specified discharge test).
   Reinsert the coin cell correctly, then restore Pi power. Reinsertion does
   not clear OSF. If power isolation is uncertain, this fault injection is
   inconclusive; do not substitute a hot battery removal with VCC present.

3. Reconnect and capture before any RTC write:

   ```bash
   rtc_capture rtc04-boot-id cat /proc/sys/kernel/random/boot_id
   rtc_capture rtc04-kernel sudo journalctl -k -b -o short-monotonic --no-pager | grep -E 'ds1307'
   rtc_capture rtc04-read sudo env TZ=UTC hwclock \
     --show --verbose --utc --noadjfile --rtc=/dev/rtc0
   ```

   A nonzero read exit is expected here. Inspect the error; do not treat any
   arbitrary command failure as a passing OSF test.

**Acceptance:** the expected device registers successfully, the fresh boot
logs `SET TIME!` and `hctosys: unable to read the hardware clock`, and the RTC
time read fails with `EINVAL` (`Invalid argument`). A permission error,
missing device or transport failure does not pass. util-linux 2.41.5 prints
`RTC_RD_NAME` for this failing `RTC_RD_TIME` call. Success after controlled
total power loss is a failure unless an intervening RTC writer is identified;
such a writer also invalidates this test's evidence.

An old log entry alone is insufficient: correlate the fault, new boot ID and
current read. This validates hardware/driver rejection, not receiver
`RtcHealth.INVALID` mapping or durable holdover policy.

## RTC-05: recovery and next cold boot

Ensure the battery is installed before recovery; if it is still absent,
shut down and disconnect power before fitting it. Preserve RTC-04 evidence.
Repeat RTC-02 with labels starting `rtc05-` so earlier captures remain intact.
Require a qualified write and successful read-back. Then shut down, remove
Pi power with the battery retained, restore power, and capture new boot ID,
kernel log and RTC read under `rtc05-reboot-*` labels as step 1-3 in RTC-03.

**Acceptance:** recovery clears invalid-read behavior; the next cold boot
reads and bootstraps the RTC without a new OSF warning. Check Chrony has a
selected source and small remaining Linux correction. Restore the normal
bench/service state and record it. Until this passes, the run is not closed
as recovered even if the deliberate fault was detected correctly.

## RTC-06: longer retention and best-effort offset comparison

1. Choose and record a longer outage, for example 24 hours. Create a **new run
   directory** using the evidence setup so the previous short run remains intact.
   Record RTC-06 as the purpose of this run.
2. Execute RTC-01 and RTC-02 in that directory: preserve initial state,
   synchronize Linux, write the RTC once, and create `rtc02-baseline.txt`.
3. Execute RTC-03 with the longer planned outage. After reboot, synchronize
   Linux without copying to the RTC and create `rtc03-after-poweroff.txt`.
   Keep these filenames: the run directory distinguishes the long test.
4. Record the calculation below, the two filenames, physical outage notes
   and acceptance outcome in the run summary. Do not remove the coin cell
   or perform RTC-04/RTC-05 between the two measurements.

**Acceptance:** the RTC-03 retention and capture criteria pass over the
recorded longer outage, and the offset comparison includes its uncertainty.
This is a functional smoke test, not a numerical datasheet-accuracy pass.
A drift estimate is optional; no particular ppm result is required. If the
measurement does not resolve a change, report **"no drift resolved by this
measurement"**, not zero drift. A reset, frozen clock or invalid read fails
the functional check; missing or invalid reference evidence is inconclusive.

## Interpreting the bracketed captures

Use the same sign throughout: **offset = RTC minus reference**, so a positive
offset means the RTC is ahead. For each successful snapshot, copy these values
into the run summary without editing the raw file:

| Value | Evidence |
|---|---|
| `B`, `A` | `before_epoch`, `after_epoch`, in seconds since the Unix epoch |
| `R` | Integer epoch seconds from the verbose `Hw clock time : ... = ... seconds since 1969` line |
| Reference quality | Both tracking reports: reference ID/time, `System time`, `Root delay`, `Root dispersion`, leap status |
| Command status | `hwclock_exit`, `after_date_exit`, `tracking_after_exit`, final `command_exit` (all zero) |

`R` is the raw whole-second reading. Do not substitute `hwclock`'s final
fractional timestamp: util-linux derives that display using software timing.
Its zero adjustment-file drift with `--noadjfile` is not measured oscillator
drift. Keep the verbose tick-wait output as evidence. See the
[util-linux 2.41.5 implementation](https://github.com/util-linux/util-linux/blob/v2.41.5/sys-utils/hwclock.c).

Estimate a reference error allowance `E` in seconds. For each tracking report,
Chrony gives this bound, conditional on a correct upstream reference:

```text
reference_error = abs(System time) + Root dispersion + Root delay / 2
```

Use at least the larger result from the two reports and record any added
margin. This is a best-effort allowance assuming stable synchronization
throughout the short bracket, not an independently calibrated guarantee.
If a reference changes, a clock step occurs during capture, leap status is
not normal, `A < B`, or the reports are stale/unsynchronized, preserve the
capture but mark that comparison inconclusive. Do not use a broad bracket or
poor reference to claim a precise result. The error expression and tracking
fields are documented in [Chrony's manual](https://chrony-project.org/doc/4.6.1/chronyc.html#tracking).

The RTC was read somewhere between `B` and `A`. Allowing a full second for
its integer representation gives a conservative offset interval under the
reference assumptions above:

```text
offset_low  = R     - A - E
offset_high = R + 1 - B + E
```

Compute this for the baseline (`O0`) and after-poweroff (`O1`) snapshots,
then subtract intervals to remove the baseline setting offset:

```text
change_low  = O1_low  - O0_high
change_high = O1_high - O0_low
```

If this interval includes zero, the change is unresolved. If entirely
positive, the RTC gained time relative to the reference; if entirely
negative, it lost time. Report the interval, not just its midpoint. For
example, `O0 = [-0.4, 0.8] s` and `O1 = [-0.2, 1.1] s` give a change of
`[-1.0, 1.5] s`: **no drift resolved by this measurement**. This is an
illustration, not evidence from a Pi run.

Optionally estimate an average rate over the entire interval between reads.
Bound that interval using the network-qualified timestamps, not RTC elapsed
time or human power-button timestamps:

```text
elapsed_low  = B1 - A0 - E0 - E1
elapsed_high = A1 - B0 + E0 + E1
```

Require `elapsed_low > 0`. A conservative ppm interval is the minimum and
maximum of the four endpoint ratios `change / elapsed * 1e6`, pairing each
change endpoint with each elapsed endpoint. State the assumptions and label
it a coarse estimate. One second of offset-change uncertainty corresponds to
about 11.6 ppm over 24 hours. Longer intervals improve resolution; these
captures do not by themselves certify the datasheet or receiver drift bound.

## Close and archive the run

Record the outcome and evidence filenames for every test, including not-run
items, and confirm RTC-05 recovery. Archive evidence alongside this procedure
under [results/](results/README.md) after recording the physical actions and
checking completeness. On the Pi:

```bash
cd "$RTC_RUN"
sha256sum ./*.txt > SHA256SUMS
```

From the laptop, copy that run into a new
`receiver/hardware/ds3231/results/<run-id>/data/raw/` directory in
`cura-agrorum`, and verify `sha256sum -c SHA256SUMS` in that copied directory.
Add a run summary, link it from the results index, and include the summary,
captures and checksums in Git. Do not overwrite earlier runs. Use
`ssh -F /dev/null` / `scp -F /dev/null` when the laptop's default SSH config
is unusable; do not put passwords into checked-in instructions or evidence.

The final record must distinguish physical actions reported by the operator,
raw software captures, derived measurements, missing evidence, and whether
the hardware was restored. Normal pytest collection does not execute this
procedure, and a manual PASS does not replace the stage-9 automated tests.

