# Soil Sensor Curve-Fitting Experiment

Date prepared: 2026-06-17  
Project: Capacitive soil sensor calibration and sensor-to-sensor transfer test

---

## 1. Purpose of the experiment

The purpose of this experiment is to test whether multiple capacitive soil moisture sensors of the same type share a similar **normalized response curve** for the same soil.

The practical question is:

> If I calibrate the soil curve using some sensors, can I later take a new sensor, measure only its dry/wet endpoints, and use the existing curve to estimate gravimetric water content?

The claim to test is not that all sensors output the same raw mV value. They clearly do not. The claim is:

> For a given soil, the shape of the response curve is similar enough across sensors that sensor-specific endpoint normalization can map different sensors onto one shared calibration curve.

This matters because future sensors may not need a full 13-point calibration if the normalized curve transfers well enough.

---

## 2. Background from previous measurements

Previous measurements were done using:

- 180 g of oven-dried soil in a plastic cup.
- A known added mass of water.
- Soil mixed/whisked after adding water.
- Soil compacted and left for about 12 hours before measurement.
- Four capacitive soil sensors.

The previous measurement points were:

| Gravimetric water content | Sensor 1 mV | Sensor 2 mV | Sensor 3 mV | Sensor 4 mV |
|---:|---:|---:|---:|---:|
| 0% | 2161 | 2148 | 2203 | 2155 |
| 5% | 1915 | 1918 | 1989 | 1903 |
| 10% | 1483 | 1459 | 1593 | 1409 |
| 15% | 1190 | 1145 | 1239 | 1124, 1102, 1108 |
| 20% | 1085 | 1006 | 1118 | 993.5 |
| 25% | 913, 1024 | 1042, 997 | 1215, 1161 | 933, 1039 |
| 30% | 1003, 982 | 928 | 969 | 778.5, 976, 962 |
| 35% | 769 | 662 | 875 | 676, 691 |
| 40% | 664 | 627.5 | 808 | 608 |
| 45% | 656 | 612 | 809 | 608 |

### Main observations from the previous test

1. **Dry soil was easier to measure.**  
   Up to around 10% gravimetric water content, the readings were relatively stable. Inserting the sensor and gently tapping the bottom of the cup was enough to redistribute the soil and reduce air gaps.

2. **Very wet soil was also easier to measure.**  
   Above about 35%, the soil behaved more like a paste/slurry and naturally compacted around the sensing blade.

3. **The mid-moisture range was the problem area.**  
   Between about 15% and 30%, the soil was moist enough to hold its shape but not wet enough to flow around the sensor. The blade could create a slot that stayed open, leaving air gaps near the sensing surface.

4. **The first sensor inserted had an advantage.**  
   The first sensor created the hole and often had better soil contact. Later measurements were affected by the disturbed soil structure unless the soil was broken up and compacted again.

5. **Waiting after insertion did not solve the issue.**  
   For this soil, waiting about 15 minutes after inserting the sensor did not meaningfully improve contact. The soil kept its shape for a long time.

6. **Squeezing or compacting improved contact, but may change density.**  
   Squeezing the cup improved soil contact and made the curve more stable, but it also changed local bulk density around the blade. This is useful as a diagnostic trick, but it should not be used in the final protocol unless the force/compaction method is standardized.

7. **Sensor 3 may behave differently.**  
   Sensor 3 was consistently higher in the wet range. At 40–45%, Sensor 3 stayed around 808–809 mV, while the other sensors were around 608–664 mV. This suggests that raw mV values are not directly interchangeable between sensors.

8. **The curve appears to plateau in wet soil.**  
   From 40% to 45%, most sensors changed very little. This suggests the sensors have poor resolution in the high-moisture range for this soil.

---

## 3. Hypothesis

For this specific soil, the raw mV curves of different sensors will differ because of:

- manufacturing tolerances,
- sensing blade geometry differences,
- coating differences,
- oscillator/output circuit tolerances,
- parasitic capacitance,
- ADC and supply-voltage effects,
- and sensor-to-soil contact differences.

However, after endpoint normalization, the curve shape may be similar enough to use one shared calibration curve.

The main hypothesis is:

> A shared normalized curve can estimate gravimetric water content for new sensors with acceptable error, using only each sensor's dry/wet endpoint readings.

A secondary hypothesis is:

> Dry-soil/wet-soil endpoint normalization will perform better than air/water endpoint normalization, because dry/wet soil endpoints include the actual soil contact, density, salinity, and field geometry effects better than air/water endpoints.

---

## 4. Sensors needed

Ideal setup:

- 10 total sensors.
- 5 sensors used for calibration.
- 5 different sensors used only for validation.

Groups:

| Group | Purpose |
|---|---|
| Calibration sensors | Build the shared normalized curve |
| Validation sensors | Test whether the shared curve transfers to unseen sensors |

If fewer than 10 sensors are available, use leave-one-sensor-out validation:

1. Build the curve using all sensors except one.
2. Test on the excluded sensor.
3. Repeat until every sensor has been used as the validation sensor once.

---

## 5. Soil preparation

For the main calibration experiment, use soil sieved to remove coarse fragments larger than 2 mm.

Reason:

- stones hold less water than fine soil,
- stones create local air gaps,
- stones make compaction less repeatable,
- stones make small cup samples less homogeneous,
- one stone near the sensing blade can strongly affect the reading.

The goal of the main experiment is to test the sensor curve, not the randomness caused by stones.

Later, do a separate real-world validation using unsieved soil to estimate practical field error.

---

## 6. Measurement points

Use 13 gravimetric water-content points.

The added water mass is calculated as:

```text
added water mass = dry soil mass × target gravimetric water content
```

For 180 g of oven-dry soil:

| Target gravimetric water content | Dry soil mass | Added water mass |
|---:|---:|---:|
| 0% | 180 g | 0.0 g |
| 5% | 180 g | 9.0 g |
| 10% | 180 g | 18.0 g |
| 15% | 180 g | 27.0 g |
| 18% | 180 g | 32.4 g |
| 20% | 180 g | 36.0 g |
| 22% | 180 g | 39.6 g |
| 25% | 180 g | 45.0 g |
| 28% | 180 g | 50.4 g |
| 30% | 180 g | 54.0 g |
| 35% | 180 g | 63.0 g |
| 40% | 180 g | 72.0 g |
| 45% | 180 g | 81.0 g |

Extra points are included in the 15–30% range because previous measurements showed this was the most contact-sensitive region.

---

## 7. Endpoint measurements for each sensor

For every sensor, record the following endpoints:

| Endpoint | Purpose |
|---|---|
| Air mV | Easy reference, useful for air/water normalization |
| Water mV | Easy reference, useful for air/water normalization |
| Oven-dry soil mV | Better dry endpoint for soil-specific normalization |
| Very wet or saturated soil mV | Better wet endpoint for soil-specific normalization |

Two normalization methods will be compared.

### Method A — air/water normalization

```text
N_air_water = (mV - water_mV) / (air_mV - water_mV)
```

### Method B — dry-soil/wet-soil normalization

```text
N_soil = (mV - wet_soil_mV) / (dry_soil_mV - wet_soil_mV)
```

In both cases:

```text
N ≈ 1 means dry
N ≈ 0 means wet
```

Expected result:

Dry-soil/wet-soil normalization should probably perform better than air/water normalization.

---

## 8. Standardized measurement protocol

For each moisture level:

1. Weigh 180 g of oven-dry, sieved soil.
2. Add the correct mass of water.
3. Mix/whisk thoroughly.
4. Cover or seal the cup.
5. Let the soil equilibrate for about 12 hours.
6. Before each sensor measurement, break up the soil structure completely.
7. Remix the soil.
8. Refill the cup and compact using the standardized method.
9. Insert one sensor to the same depth/mark every time.
10. Wait a fixed time before reading, for example 30 seconds.
11. Record mV.
12. Remove the sensor.
13. Break up and remix the soil again before measuring with the next sensor.

Important: do not simply insert the next sensor into the existing hole. Each sensor must get a freshly prepared soil structure.

---

## 9. Compaction protocol

The compaction method must be repeatable.

Avoid subjective hand-compaction where possible.

Suggested simple protocol:

1. Use the same cup type for all measurements.
2. Use the same dry soil mass.
3. Fill to the same final height.
4. Tap the cup the same number of times.
5. Press the soil surface with a flat disk.
6. Use the same weight or force each time.
7. Press for the same duration.

Example protocol:

```text
Tap the cup gently 10 times.
Press the soil surface with a flat disk using a 1 kg weight for 10 seconds.
Insert sensor to the marked line.
Wait 30 seconds.
Record mV.
```

The exact numbers can be changed, but they must stay constant during the experiment.

---

## 10. Repeated measurements

At each moisture level, for each sensor, take 3 repeated measurements.

Each repeat should include the full reset procedure:

```text
break soil → remix → refill → compact → insert sensor → measure
```

Do not simply leave the soil as-is and reinsert the sensor.

The median of the 3 readings should be used for curve fitting.

The spread between the 3 readings should also be recorded because it estimates the noise caused by the soil-preparation method.

---

## 11. Sensor order

Randomize sensor order at each moisture level.

Reason:

The previous experiment suggested that the first sensor inserted may get better contact because it creates the hole. Randomizing order prevents one sensor from always receiving the best or worst soil condition.

Example:

| Moisture point | Sensor order |
|---:|---|
| 0% | S3, S1, S5, S2, S4 |
| 5% | S2, S5, S1, S4, S3 |
| 10% | S4, S2, S3, S1, S5 |

---

## 12. Curve fitting method

For the calibration sensors:

1. Convert raw mV readings into normalized values.
2. Build one shared curve:

```text
gravimetric water content → normalized sensor value
```

or the inverse:

```text
normalized sensor value → estimated gravimetric water content
```

For practical use, prefer one of these methods:

### Option 1 — piecewise linear interpolation

This is likely the safest option.

Advantages:

- simple,
- does not force a fake equation,
- respects the measured points,
- easy to implement in code.

### Option 2 — monotonic smooth curve

Useful if the measured points are noisy but still follow a clear trend.

Requirement:

The fitted curve should be monotonic: as water content increases, mV should decrease or normalized dryness should decrease.

Avoid using a simple straight line because the response is clearly nonlinear.

---

## 13. Validation method

For each validation sensor:

1. Do not use its full calibration data to build the curve.
2. Use only its endpoint readings for normalization.
3. Normalize its raw mV values.
4. Use the shared curve to predict gravimetric water content.
5. Compare predicted water content against the known true water content.

Main error metric:

```text
moisture error = predicted gravimetric water content - true gravimetric water content
```

Example:

```text
true water content = 25%
predicted water content = 28%
error = +3 percentage points
```

Use moisture error as the main metric, not only mV error.

Reason:

The same mV error does not mean the same moisture error everywhere. In steep parts of the curve, a 50 mV error may be small. In flat plateau regions, a 50 mV error may correspond to a much larger uncertainty.

---

## 14. Metrics to report

Report the following metrics for each normalization method:

- MAE: mean absolute error in percentage points of gravimetric water content.
- RMSE: root mean squared error.
- Maximum absolute error.
- Percentage of measurements within ±3 percentage points.
- Percentage of measurements within ±5 percentage points.
- Percentage of measurements within ±7 percentage points.

Also report error by moisture range:

| Moisture range | Reason |
|---|---|
| 0–10% | Dry region, previously stable |
| 15–30% | Mid-moisture region, previously unstable/contact-sensitive |
| 35–45% | Wet region, likely sensor plateau |

---

## 15. Acceptance criteria

Suggested practical thresholds:

| Result | Interpretation |
|---|---|
| MAE ≤ 3 percentage points | Excellent |
| MAE ≤ 5 percentage points | Good/practically useful |
| MAE ≤ 7 percentage points | Maybe usable |
| MAE > 7 percentage points | Poor transfer between sensors |

Suggested pass/fail rule:

```text
The shared normalized curve is acceptable if:

MAE ≤ 5 percentage points
and
90% of validation measurements are within ±7 percentage points.
```

Also inspect the 15–30% range separately. If the total MAE is acceptable but the 15–30% region fails badly, the method may still be unreliable for irrigation decisions in that range.

---

## 16. Expected results

Expected outcome based on previous measurements:

1. Raw mV curves will differ between sensors.
2. Normalized curves should be closer than raw curves.
3. Air/water normalization may help but may not be enough.
4. Dry-soil/wet-soil normalization should probably work better.
5. The largest errors will probably occur in the 15–30% range.
6. Above 35–40%, the sensor may have poor resolution because the curve approaches a wet plateau.
7. If the normalized validation curves overlap well enough, future sensors may only need endpoint measurements plus one or two intermediate checks.

---

## 17. Optional extra validation

After the main experiment with sieved soil, repeat a smaller validation using unsieved field soil.

Suggested points:

```text
10%, 20%, 30%, 40%
```

Purpose:

- estimate the error caused by stones, roots, organic debris, and real soil structure,
- understand how much worse field conditions are compared with clean lab conditions,
- decide whether the lab calibration is still useful outdoors.

---

## 18. Practical conclusion to remember

The calibration curve is not simply:

```text
mV → water content
```

It is more accurately:

```text
mV → water content under a specific soil type, density, compaction, salinity, and sensor-contact condition
```

The purpose of this experiment is to separate three sources of error:

1. real differences between sensors,
2. errors caused by poor endpoint normalization,
3. errors caused by soil preparation/contact noise.

If the validation sensors predict moisture accurately after endpoint normalization, then the shared-curve approach is useful. If not, each sensor needs a fuller calibration, or at least additional intermediate calibration points.
