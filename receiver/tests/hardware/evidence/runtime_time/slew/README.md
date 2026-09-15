# Maximum slew — 14 September 2026

Both directions passed with an independent laptop time reference:

| Direction | Pi monotonic seconds | Rate interval, ppm | Samples |
|---|---:|---:|---:|
| Positive | 1245.920480 | +3456.57 to +3653.24 | 63 |
| Negative | 1206.055509 | -3605.17 to -3496.04 | 61 |

The first attempt failed on resolution: its positive interval was
+3218.31 to +3538.58 ppm, 320.27 ppm wide against a <200 ppm requirement.
It did not establish an excessive clock rate. Fail-fast left the negative case
not run. That fixture saved samples only after acceptance, so the failed run's
individual samples are unavailable and cannot be reconstructed.

The fix saves samples before assertions and extends a 20-minute measurement
only while resolution is insufficient, up to 40 minutes. The first resolved
interval decides the result; no extra attempts to turn a rate failure into a
pass. The rerun passed both directions and restored the Pi. Earlier RTC
refresh and forward/backward step cases had passed separately.

[Positive](positive.json) and [negative](negative.json) retain all 124 original
reference samples and 122 intervals. [Postflight](postflight.json) includes both
restoration records and the Pi addresses used to check reference independence.
[Tested source](source-manifest.json): 286 files, manifest SHA256
`e68f271a9bcb51b9e7c4e8e930386b7a6009248588d3cefc11666018e1c2e6bf`.

From this folder, run `python3 verify_slew_evidence.py .` to recompute reference
uncertainty, every rate interval and the stopping decisions using exact
rational arithmetic. The acceptance limits remain ±3700 ppm, requested-direction
magnitude >3300 ppm, and interval width <200 ppm.

The duplicate laptop bridge replies matched every retained sample when checked
before curation. That log, duplicate restoration files, generated calculations
and routine pytest output have been deleted. The verifier now reads one copy
of each measurement and the restoration records already in postflight.
