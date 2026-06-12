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

## Interpretation

Run 3 isolates the thermal effect: only the electronics temperature changed between power-off and restart. **Electronics warm-up raises the reading by ~+23 counts** (962 → 985). Consistent with the diode peak detector dominating the tempco (Vf ≈ −2 mV/°C → higher output when warm).

Run 1 was therefore two opposing processes superimposed:

- **Wetting effects** (capillary creep above the waterline + water absorption into the solder mask): ~**−50 counts**, slow and asymptotic.
- **Warm-up**: ~**+23 counts**.
- Net observed: −27 counts. The raw Run 1 data understates both mechanisms.

Estimated tempco: roughly **1.5–2.5 counts/°C** (assuming ~10–15 °C self-heating).

## Implications for field deployment

- **Wetting effects: ignorable.** Blade sits permanently in soil; equilibration happens once at install and becomes part of the baseline. Residual action item: seal the electronics end before deployment.
- **Thermal regime must be consistent.** With one reading every 15–45 min and brief power-ups, every read is a cold read — fine, but **calibration points must be taken the same way** (brief cold power-up, fixed settle delay, immediate read). Mixing warmed bench calibration with cold field reads bakes in a ~20-count systematic offset.
- **Diurnal temperature swing** (15–25 °C on above-ground electronics) implies ~30–50 counts of thermal artifact, correlated with time of day — can masquerade as a moisture signal. Significance depends on the dry-to-wet span in actual soil (unknown yet).
- **No dedicated bench tempco study.** Instead: log raw counts + DS18B20 + BME280 from day one; estimate a lumped counts-vs-temperature correction from field periods of known-constant moisture (dry stretches, nighttime). Compensate offline only if the correlation is visible against the moisture signal.

## Open items

- [ ] Repeatability of first cold-start reading (10× single cold reads, minutes apart) → decide single-shot read vs fixed settle delay.
- [ ] Optional free tempco estimate: overnight bench log (fixed water level + temperature sensor next to board).
- [ ] Dry-and-resubmerge test to split capillary creep vs coating absorption (low priority — both lead to the same mitigation).
- [ ] Measure actual counts span dry soil → field capacity at install; revisit whether thermal compensation is needed.
