# Runtime time — 14–15 September 2026

Pi 3 / Adafruit DS3231 carrier, kernel 6.18.50+rpt-rpi-v8, Chrony 4.6.1.
These separate runs used the source manifests linked below.

| Costly test | Recorded outcome | Essential captures |
|---|---|---|
| Maximum positive/negative slew | Both directions passed against an independent laptop reference | [Positive](slew/positive.json), [negative](slew/negative.json), [restoration](slew/postflight.json), [source](slew/source-manifest.json) |
| Oscillator stop and battery cold boot | Adapter rejected invalid time; recovery and cold-boot bootstrap passed | [Baseline](osf/baseline.json), [fault](osf/adapter.json), [recovery](osf/recovery.json), [cold boot](osf/coldboot.json), [operator](osf/operator-confirmation.json), [source](osf/source-manifest.json) |
| SCL/SDA controller faults | Four read/write cases passed bounded recovery and restoration | [Controller record](controller-run.json), [shared source](controller-source-manifest.json) |
| Normal carrier restoration | RTC refresh/replacement and both clock-step directions passed | [Nominal record](nominal-run.json), same controller source |

Slew measured +3456.57…+3653.24 ppm over 1245.920480 monotonic seconds
(63 samples), and -3605.17…-3496.04 ppm over 1206.055509 seconds (61 samples).
All 124 samples/122 intervals remain. Acceptance was within ±3700 ppm,
requested-direction magnitude >3300 ppm and interval width <200 ppm.
From the repository root, recompute uncertainty, intervals, stopping decisions
and restoration with:

```sh
python3 receiver/tests/hardware/evidence/runtime_time/slew/verify_slew_evidence.py receiver/tests/hardware/evidence/runtime_time/slew
```

The first slew attempt had insufficient resolution: +3218.31…+3538.58 ppm,
320.27 ppm wide. It did not establish an excessive rate; negative was NOT RUN.
Its samples were not saved. The fix saves before assertions and extends the
20-minute measurement only for insufficient resolution, at most to 40 minutes;
the first resolved interval decides. Do not extend/retry an actual rate failure.

For oscillator stop, the operator removed the coin cell while externally
unpowered; the adapter returned `INVALID`/`EINVAL`. One network-qualified write
restored valid time. A later battery-retained cold boot initialized Linux from
the RTC and returned `OK`. Three boot IDs corroborate the sequence. The operator
timed the 30-minute outage; physical switching times were not measured. The
fault boot reported `hctosys: unable to read the hardware clock`.

Releasing a fault GPIO does not ensure a high bus. The controller abort/NACK
mechanism remains unresolved; bounded stock-driver read recovery is the qualified
workaround. The [controller investigation](../../../../hardware/ds3231/LIMITATION.md)
records that lesson. Early setup fixes were root-owned persistence paths,
Chrony-owned socket directories and numeric SSH addresses when DNS was unreliable.
Their logs are unnecessary.

Later writer-audit, offline-startup, scheduling, step-deadline and uncertain-commit
fixes have host validation only. These older Pi results do not qualify those
changes or `ExecStartPre` deployment. Oscillator/cold-boot results establish
functional retention, not calibrated drift, trusted holdover or full service boot.
Rerun the relevant [time procedures](../../../../TESTING.md#time-policy-and-timestamp-analysis)
when clock policy/adapters, Chrony/kernel configuration or the RTC fixture change.
Selected JSON and manifests retain original bytes; duplicate reports/logs are gone.

The [19 September current-source qualification](../../../evidence/2026-09-19-pilot-runtime/README.md)
subsequently exercised the changed time/offline/step paths and installed helper
access under the actual service UID. It documents which unchanged assertions
reuse this older evidence; it does not turn these component/physical records
into full-service or RF acceptance.
