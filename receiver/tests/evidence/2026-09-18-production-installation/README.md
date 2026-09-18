# Production installation qualification — 2026-09-18

The sections below retain successive attempts. The latest status is in
[Final qualification](#final-qualification);
earlier statements of NOT RUN or unchanged target describe their attempt only.

The approved shared receiver path configuration is implemented. Final receiver
host suite: **3421 PASS in 32.93 seconds**. [Source manifest](SOURCE_MANIFEST.json)
records the current inputs and routine log hash. Runtime/storage preflight share
one pure loader; incomplete or unsafe environment values fail before access.
Test roots reject lexical production overlap, including Linux `//` aliases.
The service omits path defaults so an incomplete environment-file override
cannot silently mix test and production paths. Filesystem/symlink isolation
and effective installed service settings still require target verification.

An earlier version of this package was staged at
`/var/tmp/cura-pilot-20260918-vuh7p781`; all 89 actual files matched the
[initial staging manifest](INITIAL_PI_STAGE_MANIFEST.json). The final unit-default
adjustment occurred afterward. **Restage current sources before further tests**;
this initial snapshot is retained only for the reproduction below, not as the
final installed package. No existing target checkout was used.

## Retained startup failure — T-015

The target vendor `chrony.service` sets `User=_chrony`, no ambient capabilities,
and `ExecStart=!/usr/sbin/chronyd $DAEMON_OPTS`. The executable has no file
capabilities. The repository's [drop-in](../../../hardware/ds3231/chrony-runtime.conf)
replaces the command without the `!` prefix, retaining the unprivileged user.

An isolated launch as `_chrony`, with `/dev/null` as an empty configuration,
exited 1 with `Fatal error : Not superuser`.
[Exact result](CHRONY_START_FAILURE.json) retains command, outcome and staging
identity. The reproduction did not install the drop-in, stop live Chrony,
configure sources, or perform clock writes. Postcheck showed Chrony still
active with MainPID 641, active since 2026-09-18 12:47:57 CEST, and the same
existing `/usr/sbin/chronyd -F 1` processes.

Paused before installation for contract agreement: preserve the vendor's
privileged-launch prefix and verify effective startup (smallest, recommended),
or maintain a complete dedicated unit with explicit privilege/drop-user policy
and its own security/upgrade maintenance. Neither fix has been applied.

## Other applicability inputs

The Pi kernel is `6.18.50+rpt-rpi-v8`, package
`1:6.18.50-1+rpt1~beta1`, RTC `rtc-ds1307 1-0068` and device `rtc0`.
The native helper and Linux DS3231 adapter hashes match the retained controller
qualification, including read/write fault and in-flight termination observations.
Those support assessing the historical three-second allowance under DEC-007;
installed privilege/configuration and changed runtime paths are not yet qualified.
No new fault fixture, drift or hard real-time guarantee is claimed.

T-009/T-010/T-011 and final evidence closure remain open. The production receiver
paths, service user, helper and stable RTC alias were absent at inventory; none
has yet been installed or changed. The C6 remains on its non-transmitting
sensor-carrier image. Production firmware is built and sealed but unflashed.

## Approved Chrony prefix correction; new startup blocker

The user approved the smallest Chrony fix. The repository drop-in now preserves
`!`; the target audit checks `ExecStartEx` for exact arguments and `no-setuid`.
[Relevant host checks](CHRONY_PREFIX_HOST.json): **39 PASS in 0.17 seconds**.
The live service remains unchanged. Target startup verification remains NOT RUN,
so T-015 is not yet closed.

Inspection before installation found that the receiver CLI passes the required
helper SHA256 as a string, while the adapter requires 32 bytes. The
[host reproduction](HELPER_DIGEST_TYPE_FAILURE.json) uses real CLI parsing and
the real validator type check with device/filesystem boundaries supplied: a
64-character digest reaches the validator as `str` and is rejected with
`trusted helper SHA256 required`. This is not a Pi runtime test.

T-016 awaits agreement on strict digest decoding at the CLI boundary, before
hardware construction, retaining the adapter's exact bytes/ELF/capability
checks. A broader startup-options object could consolidate inputs but is not
recommended for this localized boundary error. No receiver/helper code in this
area has been changed. Further installation and target verification are paused.


## Installed package and remaining blocker

The approved CLI fix converts a canonical lowercase SHA256 to 32 bytes before
hardware access. The adapter validation remains unchanged. The final receiver
host suite passed **3434 tests in 32.50 seconds**; its
[receipt](HELPER_DIGEST_HOST.json) records the log identity.

A fresh stage, `/var/tmp/cura-pilot-20260918-vuh7p781-v2`, verified all 233 files
in the [current stage manifest](CURRENT_STAGE_MANIFEST.json). The isolated
installed package at `/opt/cura-pilot-vuh7p781` verified all 89 runtime files
against its [source manifest](INSTALLED_SOURCE_MANIFEST.json).
[Installed dependencies](PACKAGE_INSTALLATION.json) and
[actual service-user database checks](PACKAGE_VERIFICATION.json) establish
T-009: strict disposable group loading, database/schema/integrity, WAL/FULL,
foreign keys, busy250, real preflight, nonsymlink ownership/modes, persistent
ext4 data/temp filesystem and 1GiB preventive reserve. Capture retention
forecast, fault campaigns and backup/restore acceptance are separate obligations.
The production package contains no test modules; qualification scripts run
separately against installed imports.

[Actual Chrony startup](CHRONY_INSTALLED_STARTUP.json) closes T-015: exact argv,
privileged `no-setuid` launch, successful validator and active MainPID confirmed.
The boot-scoped RTC service returned **COPIED**, not ALREADY_SYNCHRONIZED:
it performed a provisional RTC-to-system copy and established no receiver trust.
The helper is root:receiver0750 with cap_sys_time=ep; its real pin and RTC read
passed in the service-user check before that check failed on Chrony access.
This partial observation does **not** close T-010.

The [failed service-user check](TIME_USER_CHECK.log) found the real Chrony
socket mode0755, which does not grant the receiver group write access.
Persistent socket access provisioning remains incomplete. An independent
[deliberately absent-socket reproduction](CHRONY_UNAVAILABLE_FAILURE.json)
establishes a separate adapter defect with pinned chronyc4.6.1: exit1, empty
stdout, `Could not open connection to daemon` on stderr is classified as
INVALID_RESPONSE, although the interface requires UNAVAILABLE. The retained
[reproducer](reproduce-chrony-unavailable.py) invokes only tracking, never
makestep. T-017 awaits contract agreement; no adapter correction was applied.

At pause, Chrony is active and the isolated receiver is **inactive/disabled**.
No receiver RF operation occurred; the C6 remains on its nontransmitting
sensor-carrier image. The target now contains the isolated package/database,
dedicated UID, helper, RTC alias, polkit rule and disabled service units.
[Created host paths](CREATED_HOST_PATHS.json) and the root-owned stage directory
`host-before-installation/` record restoration inputs. Keep those artifacts
until qualification finishes; the previous Chrony configuration/unit state
is preserved there. No Pi reboot or full installed-service lifecycle result
is claimed. T-010/T-011/T-012 remain open.


## Approved connection-error fix and reply-socket deployment gap

The narrow T-017 fix is implemented. The complete receiver host suite passed
**3443 tests in 33.45 seconds** ([receipt](CHRONY_UNAVAILABLE_HOST.json)).
A fresh `/var/tmp/cura-pilot-20260918-vuh7p781-v3` runtime stage verified 89 files,
then updated the inactive isolated package against its
[source manifest](V3_INSTALLED_SOURCE_MANIFEST.json). The
[real missing-socket regression](V3_CHRONY_UNAVAILABLE_FIXED.json) now returns
UNAVAILABLE. Overflow, altered response envelopes and deadline precedence have
host regressions; clock-step uncertainty was not changed.

The installed `/etc/systemd/system/chrony.service.d/40-cura-pilot-socket.conf`
adds privileged post-start chgrp cura-receiver and chmod0660 for chronyd.sock.
Actual restart preserved that ownership/mode and Chrony remained active.
This is an additional created host file beyond CREATED_HOST_PATHS.json.
The runtime directory remains0750; no new writable-directory access was granted.

The [service-user check](V3_TIME_USER_CHECK.log) still cannot query Chrony.
[Actual strace](V3_CHRONYC_REPLY_SOCKET_FAILURE.json) establishes why:
chronyc binds its own `/run/chrony/chronyc.<pid>.sock`, receiving EACCES.
The receiver unit also uses ProtectSystem=strict and only allows writes under
its data directory. Socket access provisioning therefore needs an explicit
reply-socket directory and sandbox contract; changing only daemon-socket mode
cannot complete it. T-018 awaits agreement on constrained shared runtime access
or a narrow local coordinator. No permission expansion or coordinator change
has been applied. Receiver inactive/disabled, Chrony active, C6 nontransmitting;
T-010/T-011/T-012 remain open. No full-service or RF acceptance is claimed.


## Final qualification

The selected production qualification batch is complete. The final
[89-file installed runtime manifest](FINAL_INSTALLED_SOURCE_MANIFEST.json) was
verified before target execution and again after both Pi reboots. The
[qualification record](FINAL_QUALIFICATION.json) retains installed settings,
public lifecycle rows, decisive journals, failed attempts and restoration.
[Qualification scripts](qualification_scripts/) retain the exact checks and
fixtures for review. They contain fixed disposable paths and privileged target
actions; they are evidence for this run, not a generic test runner. The retained
missing-device checker intentionally remains the original failed version;
MISSING_DEVICE_RECONCILED records its corrected interpretation without replacing
the original result.

| Installed assertion | Result |
|---|---|
| Package, strict group binding, schema, SQLite policy, storage/UID checks | PASS |
| Helper pin/capability boundary, RTC/kernel/Chrony access under service sandbox | PASS |
| Chrony reply cleanup, sticky daemon-entry protection, unrelated-write rejection | PASS after single-command correction; repeat daemon restart and reboot PASS |
| Clean stop/restart, exact marker and sleep-inhibitor release | PASS; stops approximately0.34s |
| SIGKILL restart, delayed supervisor recovery, no false clean marker | PASS |
| Denied SPI device, five-start limit, nominal restoration | PASS; original Result-label checker failure retained |
| Offline boot with missing bootstrap RTC | PASS; MISSING in1.59s, receiver UNTRUSTED |
| Bootstrap once per boot and receiver restart | PASS; explicit repeat ALREADY_ATTEMPTED |
| Reboot after clean and killed instances, new boot/process identities | PASS |

Final host validation: **3443 PASS** before the single-command deployment
adjustment; **40 affected tests PASS** afterward, including the regression that
prevents splitting the permission changes across post-start commands. Final
unit behavior was exercised on the Pi. The failed multi-command attempt is
retained because systemd reapplies RuntimeDirectory ownership/mode before each
command. The final one-command form preserves the approved permissions.

Final state: nominal Chrony configuration restored and active; RTC alias/device
permissions nominal; temporary denial/offline fixtures removed; isolated receiver
cleanly stopped, disabled and holding no inhibitor; database integrity `ok`.
Installed package/helper/units and the disposable database remain for later
qualification. The C6 is still on its nontransmitting carrier test image; the
production firmware remains built but unflashed. No authenticated RF operation
was run in this batch.

These are software device-denial and orderly reboot results, not physical
power-loss or RTC-fault proof. Network-time sources were removed for the offline
fixture while administrative SSH remained available. No time-accuracy,
complete runtime time/storage campaign, RF-019/RF-020, RF-027/RF-028 or aggregate
deployment acceptance is inferred. The cleaner Chrony permission boundary is
explicitly deferred as [PERM-001](../../../deploy/README.md#deferred-post-pilot-permissions-review).


A final [live Chrony restart check](LIVE_CHRONY_RESTART.json) also passed inside
the already-running receiver's mount namespace, with unchanged receiver PID.
The distro unit's `RuntimeDirectoryPreserve=restart` retains the directory inode;
the approved post-start command restores socket permissions. This verifies
restart access without claiming network trust immediately after restart.
The final receiver instance stopped cleanly and the service remains disabled.
