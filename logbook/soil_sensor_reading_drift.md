# Capacitive Soil Sensor v1.2 — Reading Drift After Power-On / Immersion

**Date:** 2026-06-12
**Setup:** Sensor blade submerged ~2.5 cm in water, stable ambient conditions. 1 reading/s, medians computed over growing windows.

## Observations

**Run 1 — cold power-on + fresh immersion:**
Initial reading 1012, drifting down asymptotically: median 1008 (first 100), 1002.5 (200), 993 (300), 990 (400), 988 (500), 987 (600), 986 (700), 985 (800–1000). Plateau ≈ 985, reached in ~13 min.

**Run 2 — fast power cycle (~1 s off), sensor untouched:**
Read 987 immediately, stable. Confirms drift is not a readout-chain artifact (ADC, software state); discriminates nothing else since electronics stayed warm and blade stayed wet.

**Run 3 — 10 min powered off, sensor untouched (cold restart, wet state preserved):**
Started at 962, drifting **up**: median 966 (20), 972 (100), 977 (200), 980 (300), 982 (400), 983 (500). Converging back to ~985.

### Follow-up in soil at field capacity (2026-06-18)

Both sensors were inserted into soil at field capacity and sampled with the deep-sleep calibration firmware, which powers the sensor only for each reading and sleeps for approximately 2 s between readings.

**Sensor A:**
Started at 995 and reached 958 after 1000 readings. It was then left unplugged for 15 min without moving it. On restart it read 948 initially, with a median of 952 over the first 100 readings.

**Sensor B:**
Started at 736. After 100 readings, the median was 712, the current reading was 704, and the mean was 714.84. The current reading being below both the median and mean indicates that the downward drift was still in progress.

## Interpretation

Run 3 isolates the thermal effect: only the electronics temperature changed between power-off and restart. **Electronics warm-up raises the reading by ~+23 counts** (962 → 985). Consistent with the diode peak detector dominating the tempco (Vf ≈ −2 mV/°C → higher output when warm).

The soil runs reject the hypothesis that insertion drift is negligible when sensor heating is minimized. Both sensors drifted downward by a similar relative amount: Sensor A by approximately 3.7% and Sensor B by at least 4.3%. The deep-sleep duty cycle makes sensor self-heating an unlikely explanation for most of this drop.

The downward component should therefore be described as an **insertion/contact equilibration effect**, not specifically as solder-mask absorption or capillary creep. Possible mechanisms include blade wetting, trapped air escaping, soil relaxing around the blade, and local water redistribution. The current data do not distinguish them.

Run 1 was likely two opposing processes superimposed: a downward wetting/contact effect and an upward thermal warm-up effect. Sensor A's restart from 948 toward a first-100 median of 952 is also consistent with a small positive warm-up contribution, but it does not isolate temperature because insertion/contact equilibration continued during the 15 min power-off period.

Estimated tempco: roughly **1.5–2.5 counts/°C** (assuming ~10–15 °C self-heating).

## Implications for field deployment

- **Insertion/contact equilibration must be allowed for.** The blade sits permanently in soil, so the drift should eventually become part of the installed baseline, but calibration or validation readings taken immediately after insertion can be biased high.
- **Thermal regime must be consistent.** With one reading every 15–45 min and brief power-ups, every read is a cold read — fine, but **calibration points must be taken the same way** (brief cold power-up, fixed settle delay, immediate read). Mixing warmed bench calibration with cold field reads bakes in a ~20-count systematic offset.
- **Diurnal temperature swing** (15–25 °C on above-ground electronics) implies ~30–50 counts of thermal artifact, correlated with time of day — can masquerade as a moisture signal. Significance depends on the dry-to-wet span in actual soil (unknown yet).

#OPEN
## Next experiment: isolate the thermal contribution

For each sensor, use one settled insertion in soil at field capacity and do not move the sensor between runs:

1. Start from ambient temperature and run the deep-sleep firmware for 30–40 min.
2. Power the sensor off for 15–30 min, leaving it unmoved so the electronics can cool while the insertion state is preserved.
3. Start the always-on firmware from ambient temperature and run it for 20–30 min.
4. Compare the slopes after the first few readings, rather than comparing only the endpoints.

If the always-on run drifts upward substantially more than the deep-sleep run on the same settled insertion, that difference is strong evidence of sensor self-heating. A continued downward trend in both runs would indicate that insertion/contact equilibration is still active and must be allowed to settle before estimating the thermal effect.

## Other open items

#OPEN
- [ ] Dry-and-resubmerge test to split capillary creep vs coating absorption (low priority — both lead to the same mitigation).
