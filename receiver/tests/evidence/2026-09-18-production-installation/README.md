# Installed Pi boot qualification — 18 September 2026

## Final qualification

**PASS within the isolated pre-radio boot scope.** Retained because this sequence
changes time/bootstrap availability and reboots the dedicated Pi twice.
The [result](results.json) contains the verified 89-file runtime identity,
installed unit configuration, boot/instance rows and restoration observations.

| Test | Recorded result |
|---|---|
| Boot without network-time sources or accessible bootstrap RTC | Bootstrap MISSING in 1.59 s; receiver UNTRUSTED |
| Bootstrap once per boot, including receiver restart | Explicit repeat returned ALREADY_ATTEMPTED |
| Reboot after clean and killed receiver instances | New boot/process identities; no false clean-stop marker for the killed instance |
| Restore nominal configuration and stop | Chrony active, database integrity `ok`, receiver stopped/disabled, no inhibitor, all 89 installed source hashes verified |

Target: Pi kernel `6.18.50+rpt-rpi-v8`, dedicated `cura-receiver` service account,
isolated `/opt/cura-pilot-vuh7p781` package. The source baseline alone does not
identify the then-uncommitted runtime; the recorded file hashes do.

Network-time sources were removed while administrative SSH remained available.
These are orderly reboot/software-denial results, not physical power-loss,
time-accuracy, RF or aggregate deployment acceptance. The later initial-Chrony-
poll correction changed the runtime; review applicability before reusing this
result. Rerun after relevant boot, unit, privilege or time-policy changes.

Routine installation/host checks, failed attempts and one-off scripts were
discarded under the [retention policy](../../../../EVIDENCE.md).
Chrony permission lessons and the separate PERM-001 deferral live in the
[deployment procedure](../../../deploy/README.md#pilot-chrony-socket-permissions).
