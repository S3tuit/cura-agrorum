# Physical and intrusive time tests — 14–15 September 2026

Pi 3 / Adafruit DS3231, kernel `6.18.50+rpt-rpi-v8`, Chrony 4.6.1.
The [result](results.json) consolidates tested input hashes, decisive observations
and restoration for these separate runs.

| Test | Recorded result |
|---|---|
| Positive slew, 1245.920480 monotonic seconds | PASS: +3456.57…+3653.24 ppm against independent laptop reference |
| Negative slew, 1206.055509 seconds | PASS: −3605.17…−3496.04 ppm |
| Manual oscillator stop | Coin cell removed while externally unpowered; adapter returned INVALID/EINVAL; one network-qualified write restored valid time |
| Battery-retained cold boot | Operator-timed 30-minute outage; Linux initialized from RTC, adapter OK |
| SCL/SDA controller read/write faults | 4 PASS with bounded recovery and successful restoration |
| Intrusive nominal restoration | RTC refresh/replacement and both clock-step directions PASS |

Slew acceptance required ±3700 ppm, requested-direction magnitude >3300 ppm and
interval width <200 ppm. Before consolidation the independent exact-arithmetic
verifier checked all 124 samples/122 intervals, uncertainty and stopping rules.
Only the final endpoint brackets and rate intervals remain; the complete
stopping sequence cannot be replayed. Chrony, device permissions, helpers and
fault GPIOs were restored.

The outage is operator-timed; physical switching instants were not measured.
These historical component/physical tests do not establish calibrated RTC drift,
trusted holdover, full-service boot or later changed implementations. See the
[controller limitation](../../../../hardware/ds3231/LIMITATION.md) for the unresolved
controller behavior and the [time procedure](../../../../TESTING.md#time-policy-and-timestamp-analysis)
for assertions and rerun criteria. The
[19 September record](../../../evidence/2026-09-19-pilot-runtime/README.md) separately
qualifies destructive component paths under the actual service UID.
