# Oscillator stop and cold boot — 14 September 2026

Passed: the production adapter returned `INVALID`/`EINVAL` after the operator
removed the coin cell while externally unpowered. One network-qualified
recovery write restored valid time. A later battery-retained cold boot
initialized Linux from the RTC and the adapter returned `OK`.

The [baseline](baseline.json), [fault capture](adapter.json),
[recovery](recovery.json) and [cold-boot capture](coldboot.json) show the sequence.
The fault capture includes the operator's confirmation and the baseline hash.
The [outage confirmation](operator-confirmation.json) records the 30-minute
wait: the operator timed it; exact physical switching times were not measured.
Three different boot IDs are recorded across baseline, fault and final boot.

These are the relevant lines from the discarded fault-boot kernel log:

```text
[   13.533499] cura-receiver kernel: rtc-ds1307 1-0068: SET TIME!
[   13.534218] cura-receiver kernel: rtc-ds1307 1-0068: registered as rtc0
[   13.534872] cura-receiver kernel: rtc-ds1307 1-0068: hctosys: unable to read the hardware clock
```

The final capture retains the successful RTC bootstrap and restoration checks.
The adapter fault test passed once; both following safe Pi suites passed eight
cases. Their repetitive pytest output and duplicate captures have been removed.

[Tested source](source-manifest.json): 287 files, with the manifest hash in
`baseline.json`. This establishes invalid-time rejection and functional battery
retention, not calibrated drift, trusted holdover or full receiver-service boot
integration. Use the maintained [operator procedure](../../../../../hardware/ds3231/OPERATOR_TESTS.md)
for another physical run.
