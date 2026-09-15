# Runtime time: results worth keeping

These runs used the Pi 3 / Adafruit DS3231 carrier, kernel
`6.18.50+rpt-rpi-v8` and Chrony 4.6.1, on 14–15 September 2026.

| Run | What we learned | Details |
|---|---|---|
| Maximum slew | Both directions passed after fixing insufficient measurement resolution | [Slew report and samples](slew/README.md) |
| Oscillator stop and cold boot | The adapter rejected invalid RTC time; recovery and a battery-retained cold boot passed | [OSF report and captures](osf/README.md) |
| Controller faults | Four SCL/SDA read/write cases passed with bounded recovery and restoration | [Findings and limits](../../../../hardware/ds3231/LIMITATION.md), [original compact record](controller-run.json) |
| Return to the normal carrier | RTC refresh/replacement and both clock-step directions passed; restoration checked | [Original compact record](nominal-run.json) |

Controller and normal-carrier results share this
[tested-source manifest](controller-source-manifest.json). The other reports
link their own manifests; these were separate runs on different source trees.

## Things worth remembering

Early setup failures were fixture issues: persistence test data needed a
root-owned directory, Chrony's socket directory needed the daemon's ownership
(the test child gets supplementary group access), and unreliable hostname
resolution needed the numeric Pi address with the existing SSH host-key alias.
Those logs are gone; the fixes and these reminders are enough.

The controller investigation is already summarized in
[LIMITATION.md](../../../../hardware/ds3231/LIMITATION.md). In particular,
releasing a fault GPIO does not guarantee that the bus is high. The original
controller failure mechanism remains unresolved; stock-driver read recovery
is the qualified workaround.

Review later found that the old writer audit checked the wrong configuration
assumption and that the offline fixture sampled before its receiver instance
existed. Refresh scheduling, step deadlines and uncertain state-commit retries
also needed corrections. Those fixes have host validation only; these older
Pi passes do not qualify them or the new `ExecStartPre` deployment procedure.

## Keeping this small

Keep manual/slow results, useful failure lessons and the inputs needed to check
them. Routine host/safe-Pi logs, duplicated metadata and resolved setup logs
have been deleted. Put future raw runs in ignored `raw/` or outside the repo;
use the [test procedures](../../../../TESTING.md#time-policy-and-timestamp-analysis)
for new runs. Separate DS3231 operator archives are unchanged.

Retained JSON captures and source manifests are original bytes. From this
folder, `sha256sum -c SHA256SUMS` checks the curated files; the checksum list
was regenerated during curation, not captured during the hardware runs.
