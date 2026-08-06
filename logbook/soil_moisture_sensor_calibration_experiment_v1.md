# Soil Moisture Sensor Calibration and Manufacturer-Comparison Experiment

**Protocol version:** 1.0  
**Status:** Frozen experimental procedure  
**Prepared:** 15 July 2026  
**Project:** Affordable soil-moisture sensing for Italian agriculture

---

## 1. Purpose

This experiment will evaluate whether inexpensive capacitive soil-moisture sensors can provide sufficiently repeatable and useful measurements for an affordable agricultural monitoring product.

The experiment is designed to answer three main questions:

1. Do multiple capacitive soil-moisture sensors from the same manufacturer share a similar **normalized response curve** in the same soil?
2. Does a normalized curve transfer between sensors sold by different manufacturers when the sensors appear to use the same circuit and sensing geometry?
3. How do inexpensive clone sensors compare with:
   - original DFRobot capacitive sensors; and
   - one commercial DFRobot RS485 soil-temperature and moisture probe used as an exploratory benchmark?

The experiment will first focus on controlled data collection. Detailed statistical analysis and agronomic irrigation thresholds will be defined after the dataset has been collected.

---

## 2. Sensor groups

### 2.1 Main analog capacitive sensors

| Group | Quantity | Source | Purpose | Order |
|---|---:|---|---|---|
| Manufacturer A clones | 5 | [AliExpress listing A](https://it.aliexpress.com/item/1005009040921501.html) | Within-manufacturer consistency and transfer | Ordered on Jul 15 10pm for 7.70 eur |
| Manufacturer B clones | 5 | [AliExpress listing B](https://it.aliexpress.com/item/1005009445826695.html) | Cross-manufacturer comparison | Ordered on Jul 15 10pm for 8.97 eur |
| DFRobot capacitive sensors | 5 | Comparable DFRobot capacitive model (SKU: SEN0193) | Original-versus-clone comparison | Ordered on Jul 15 10pm for 24.30 eur |

Total analog capacitive sensors:

\[
5 + 5 + 5 = 15
\]

### 2.2 Exploratory commercial sensor

| Sensor | Quantity | Role |
|---|---:|---|
| DFRobot RS485 MODBUS-RTU IP68 Soil Temperature & Moisture Sensor, SEN0600 | 1 | Exploratory commercial benchmark |

Only one SEN0600 unit will initially be tested. Its results will describe the performance of that particular unit and must not be generalized to all SEN0600 sensors.

If the first unit shows useful potential, a separate experiment using at least two or more SEN0600 units will be designed.

### 2.3 Same-design verification

The two clone listings appear to show the same schematic and sensor design. This classification is provisional until the received sensors are inspected.

For every received analog sensor, record:

- unique sensor ID;
- seller/manufacturer group;
- purchase date and batch;
- photographs of both PCB sides;
- PCB length, width, and thickness;
- sensing-trace geometry;
- regulator marking;
- oscillator IC marking;
- resistor and capacitor markings where readable;
- coating or solder-mask differences;
- connector and pin order;
- air output;
- water output;
- supply current;

Two sensors should not be considered electrically equivalent solely because the schematic appears the same. PCB geometry, coating thickness, component tolerances, and oscillator frequency may still differ.

---

## 3. Soil material

Use one homogeneous stock of the same soil throughout the experiment.

The soil must be:

1. passed through the approximately 2 mm sieve;
2. dried using the established drying procedure;
3. stored in sealed containers after drying; and
4. mixed as a large dry stock before portions are taken for individual moisture batches.

The same dry-soil stock should be used for:

- the preliminary packing experiment;
- startup-characterization tests;
- Stage 1; and
- Stage 2.

However, soil portions used in preliminary trials must not be reused as Stage 1 or Stage 2 calibration batches.

---

## 4. Measurement cups and geometry

Use tapered plastic cups with approximately:

| Dimension | Value |
|---|---:|
| Bottom internal diameter | 6.0 cm |
| Top internal diameter | 8.3 cm |
| Total height | 14.5 cm |
| Chosen soil fill height | 8.5 cm |
| Soil-surface diameter at 8.5 cm | approximately 7.35 cm |
| Soil volume at 8.5 cm | approximately 298.4 cm³ |

The cup is treated as a truncated cone. The volume below the 85 mm fill line is approximately:

\[
V \approx 298.4\ \text{cm}^3
\]

Mark the **85 mm fill line** clearly on every cup.

Use a flat packing disk approximately:

- 70 mm in diameter;
- rigid enough not to bend;
- flat on the soil-contact surface; and
- equipped with a central handle or rod.

### Sensor position

The analog sensor has an approximately 72 mm sensing blade.

Standard insertion position:

- sensor centred laterally in the cup;
- sensor tip approximately 5 mm above the cup bottom;
- upper end of the sensing blade approximately 8 mm below the soil surface;
- sensor blade not touching the cup wall or bottom;
- insertion depth marked physically on every sensor or holder.

---

## 5. Definitions

### 5.1 Gravimetric water content

\[
\theta_g = \frac{m_w}{m_d}
\]

where:

- \(m_w\) is the mass of added water;
- \(m_d\) is the oven-dry soil mass.

Example:

\[
320\text{ g dry soil at }20\% =
320 \times 0.20 = 64\text{ g water}
\]

Gravimetric water content is the directly controlled experimental reference.

### 5.2 Volumetric water content

Volumetric water content will be derived later from the known packed volume:

\[
\theta_v = \frac{V_w}{V_{soil}}
\]

Assuming water density is approximately 1 g/cm³:

\[
\theta_v \approx \frac{m_w}{V_{soil}}
\]

or equivalently:

\[
\theta_v = \theta_g \rho_b
\]

where \(\rho_b\) is dry bulk density.

### 5.3 Technical reading

A **technical reading** is one electronic reading or one set of ADC samples collected while the sensor remains stationary.

Multiple technical readings within one insertion measure electronic and short-term signal variation.

### 5.4 Placement replicate

A **placement replicate** requires the complete procedure:

```text
remove sensor
→ break up soil
→ repack to 85 mm
→ reinsert sensor
→ collect readings
```

Placement replicates measure installation, packing, and soil-contact variability.

### 5.5 Batch replicate

A **batch replicate** is a newly prepared mixture made from a fresh dry-soil portion and a newly weighed water quantity at the same target gravimetric water content.

Batch replicates measure preparation, mixing, and day-to-day variability.

---

## 6. Preliminary experiment: selecting the dry-soil mass

The goal is to choose the greatest practical dry-soil mass that can consistently be packed to the 85 mm line across the tested moisture range without requiring excessive force.

### 6.1 Initial candidate

Start with:

| Variable | Initial value |
|---|---:|
| Dry soil | 320 g |
| Target GWC | 20% |
| Water | 64 g |
| Target final height | 85 mm |
| Maximum permitted scale reading | 5 kg |
| Force duration | 15 seconds |

Twenty percent GWC is used because previous measurements indicate that this clay-rich soil is particularly cohesive and difficult to compact around that moisture level.

### 6.2 Candidate adjustment

Prepare and mix the 20% sample using the direct-in-cup procedure in Section 7.

After overnight equilibration:

1. break up the soil completely;
2. refill and level the cup;
3. place the 70 mm disk on the soil;
4. press for 15 seconds;
5. record the scale reading in kilograms;
6. remove the disk;
7. wait 10 seconds; and
8. record the final soil height.

Target after pressure release:

\[
85 \pm 1\text{ mm}
\]

Adjustment rules:

- If the final surface is below 84 mm with ordinary packing, add:
  - 10 g dry soil;
  - 2 g water.
- Remix, reseal if necessary, and retest.
- If reaching the line requires more than 5 kg for 15 seconds, reduce the mixture by approximately:
  - 10 g dry-soil equivalent;
  - 2 g water;
  - 12 g total well-mixed 20% wet soil.
- Do not select a mass that only barely succeeds at the 5 kg limit.

The preferred candidate should normally reach the line with a comfortable margin below the 5 kg hard limit.

### 6.3 Confirmation samples

Once a candidate dry mass has been selected:

1. retain the successful 20% candidate;
2. prepare two additional fresh 20% cups using the same dry mass;
3. confirm that all three independently prepared 20% cups reach the fill line consistently;
4. prepare a separate fresh 0% cup using the selected dry mass;
5. verify packing at 0%;
6. add enough water to one of the 20% cups to increase it to 40%;
7. remix, seal, equilibrate, and verify packing at 40%.

For a dry mass \(M\):

\[
\text{additional water for }20\%\rightarrow40\% = 0.20M
\]

Record for each test:

- dry-soil mass;
- total water;
- target GWC;
- scale reading needed for compaction;
- pressure duration;
- height immediately after release;
- height after 10 seconds;
- visible cracks;
- visible voids;
- large persistent clods;

These preliminary samples must not be used as Stage 1 or Stage 2 calibration batches.

### 6.4 Packing-force limit for later experiments

The formal experiment will standardize the **final mass and volume**, not the exact force.

During later measurements:

- compact only until the soil reaches the 85 mm line;
- never exceed the maximum accepted force established during the preliminary test;
- never exceed the absolute limit of 5 kg for 15 seconds;
- if the line cannot be reached within the accepted procedure, flag the preparation rather than applying arbitrary extra force.

Compaction force does not need to be recorded during every formal placement measurement.

---

## 7. Direct-in-cup mixing procedure

Direct mixing in the measurement cup is used to avoid transfer losses of water and clay-rich wet soil from a separate container.

### 7.1 Preparation

For every moisture batch:

1. label the cup with:
   - batch ID;
   - target GWC;
   - dry-soil mass;
   - water mass;
   - preparation date and time;
2. weigh and record the empty dry cup;
3. weigh the complete dry-soil portion;
4. weigh the complete water portion separately;
5. divide both the soil and water conceptually into four approximately equal portions.

Exact quartering is not required if total masses are correct, but additions should remain reasonably balanced.

### 7.2 Layered addition and mixing

Repeat four times:

1. add approximately one quarter of the dry soil;
2. distribute approximately one quarter of the water over several areas rather than pouring it into one point;
3. whisk and break clumps for approximately 30 seconds;
4. scrape soil from the walls and bottom back into the mixture.

After all four additions:

1. mix the complete batch for approximately 2 minutes;
2. scrape the sides, bottom, and mixing tool;
3. visually check for dry pockets, wet pockets, and large unmixed clods;
4. weigh the cup and mixture;
5. seal the cup.

### 7.3 Equilibration

Leave every sealed batch to equilibrate overnight for a fixed interval.

Target interval:

\[
24 \pm 2\text{ hours}
\]

Before measurement:

1. weigh the sealed cup again;
2. compare its mass with the post-mixing mass;
3. record any difference;
4. do not silently replace missing water.

A meaningful unexplained mass change must be flagged.

---

## 8. Preliminary startup and thermal-drift characterization

This test must be completed after the packing protocol is selected and before Stage 1 begins.

### 8.1 Purpose

Determine:

- whether sensors are stable immediately after power-up;
- whether the first readings show electrical or thermal drift;
- how long each sensor family requires to stabilize;
- which fixed reading window should be used in Stage 1 and Stage 2.

### 8.2 Test design

Use approximately:

- 3 sensors from manufacturer A;
- 3 sensors from manufacturer B;
- 3 DFRobot capacitive sensors.

Test at:

- 0% GWC;
- 20% GWC;
- 40% GWC.

For each test:

1. use a fresh prepared soil batch;
2. pack to the 85 mm line;
3. insert the sensor at the standard position;
4. apply the intended field power cycle;
5. record at least one output value per second for 120 seconds.

### 8.3 Power-cycle requirement

ESP32 deep sleep does not necessarily switch off a sensor powered directly from a permanent 3.3 V or 5 V rail.

The final firmware procedure must document whether the sensor is:

- continuously powered; or
- physically power-switched off between measurement cycles.

If cold-start behaviour is being tested, sensor power must actually be disconnected using suitable switched power hardware.

The laboratory cycle should match the intended field cycle.

### 8.4 Final reading window

The final reported value for one placement will be the median of a fixed group of consecutive readings.

The window will be selected from the startup test.

Examples:

```text
median of seconds 1–10
median of seconds 5–14
median of seconds 10–19
```

The first ten readings may be used only if the startup experiment shows that they are stable.

Once selected, the same rule must be used for every analog sensor in Stage 1 and Stage 2.

---

## 9. Standard packing and measurement procedure

### 9.1 Order of moisture levels

Measurements will be performed from dry to wet to reduce contamination of drier batches by water or wet clay remaining on a sensor.

### 9.2 Random sensor order

Before beginning each experimental day:

1. generate the complete random order for all 15 analog sensors;
2. generate a separate random order for every placement-replicate block;
3. save or print the orders before measurement begins.

Do not consistently measure all A sensors, then all B sensors, then all DFRobot sensors.

### 9.3 Preparation before every placement measurement

For each sensor placement:

1. remove the previous sensor;
2. recover visible soil adhering to the blade where practical;
3. break up the complete soil structure;
4. redistribute the soil throughout the cup;
5. place the disk on the surface;
6. compact to the 85 mm line using the accepted packing procedure;
7. remove the disk;
8. wait 10 seconds;
9. verify final height is \(85 \pm 1\) mm;
10. insert the assigned sensor at the marked position;
11. verify that it does not touch the cup wall or bottom;
12. execute the standardized power and logging cycle;
13. calculate the placement result using the fixed median window;
14. save the raw time series as well as the median.

### 9.4 Sensor cleaning

Use the same cleaning procedure for every sensor:

- remove loose soil gently;
- use the same type of clean, non-abrasive wipe or tool;
- do not wet the sensor before inserting it into a drier sample;
- ensure the blade is visibly clean before moving to the next moisture level;
- avoid scraping or damaging the sensor coating.

### 9.5 Soil-mass checks during a sensor block

One cup is used for each moisture level.

For each 15-sensor placement block:

1. weigh the cup and soil before the block;
2. measure all 15 sensors once in the predetermined random order;
3. weigh the cup and soil after the block;
4. record the mass difference.

Initial interpretation limits:

| Mass change | Action |
|---:|---|
| ≤ 0.5 g | Accept and record |
| > 0.5 g to 1.0 g | Accept provisionally and flag |
| > 1.0 g | Investigate; repeat the batch or block if a procedural loss occurred |

Do not add unmeasured replacement water.

Keep the cup sealed whenever it is not actively being mixed, packed, or measured.

---

## 10. Endpoint and baseline measurements

For every analog sensor, collect:

- air output;
- water output, with only the permitted sensing section submerged;
- 0% oven-dry soil output;
- high-moisture soil output.

The 0% and high-moisture soil points will support soil-specific endpoint normalization.

Air/water normalization may also be tested later, but water immersion must not reach unsealed electronics.

All endpoint measurements must use the same:

- supply voltage;
- ADC;
- firmware;
- power cycle;
- reading window;
- sensor orientation.

---

## 11. Stage 1: screening experiment

### 11.1 Moisture levels

Prepare independent cups at:

```text
0%, 5%, 10%, 15%, 20%, 25%, 30%, 40% GWC
```

Total screening levels:

\[
8
\]

### 11.2 Replication

For every analog sensor at every moisture level:

- perform 2 placement replicates;
- each replicate requires full soil break-up, repacking, reinsertion, and measurement.

Total analog placement measurements:

\[
15\text{ sensors}
\times 8\text{ levels}
\times 2\text{ placements}
= 240
\]

Each moisture level uses one prepared batch during Stage 1.

### 11.3 Stage 1 goals

Stage 1 should reveal:

- dead or abnormal sensors;
- sensors with non-monotonic behaviour;
- unusually noisy sensors;
- within-manufacturer raw-output variation;
- within-manufacturer normalized-curve similarity;
- cross-manufacturer curve similarity;
- whether DFRobot sensors have lower unit-to-unit variation;
- whether the controlled packing strategy reduces the deviations observed during earlier manual compaction;
- whether the 15–30% region remains dominated by contact variability;
- whether sensor response approaches a plateau at high moisture;
- whether the experimental procedure is practical enough to continue.

### 11.4 Data-review rule

During data collection:

- record data and procedural observations;
- do not exclude measurements merely because they appear unusual;
- avoid making product conclusions during the measurement session;
- perform the first full interpretation no earlier than the following day.

---

## 12. Stage 2: formal experiment

Stage 2 proceeds only if Stage 1 produces promising data and the protocol remains unchanged.

If the procedure, firmware, reading window, cup geometry, packing method, or soil preparation is materially changed, Stage 1 must remain pilot data and must not be merged directly with Stage 2.

### 12.1 Full moisture grid

The full experiment uses the original 13-point grid:

```text
0%, 5%, 10%, 15%, 18%, 20%, 22%,
25%, 28%, 30%, 35%, 40%, 45% GWC
```

After adding 5% to Stage 1, only five new levels remain for Stage 2:

```text
18%, 22%, 28%, 35%, 45%
```

### 12.2 Placement replication

Final target for every analog sensor at every moisture point:

\[
3\text{ placement replicates}
\]

For the eight Stage 1 levels:

- prepare a new independent Stage 2 batch;
- collect one additional placement replicate per sensor;
- combine it with the two Stage 1 placement replicates;
- final total: 3 placements across 2 independently prepared batches.

For the five new Stage 2 levels:

- prepare one fresh batch per level;
- collect 3 placement replicates per sensor.

### 12.3 Important batch-replicate points

The points:

```text
5%, 20%, 30%, 40%
```

must contain at least two independently prepared moisture batches.

This requirement is automatically met if:

- the Stage 1 batch supplies the first two placement replicates; and
- a fresh Stage 2 batch supplies the third placement replicate.

Whenever practical, prepare the Stage 2 batches on a different day.

### 12.4 Stage 2 additional workload

Additional placements after Stage 1:

For the eight existing levels:

\[
15 \times 8 \times 1 = 120
\]

For the five new levels:

\[
15 \times 5 \times 3 = 225
\]

Total Stage 2 additions:

\[
120 + 225 = 345
\]

Combined Stage 1 and Stage 2 analog placements:

\[
240 + 345 = 585
\]

This is equivalent to:

\[
15\text{ sensors}
\times 13\text{ levels}
\times 3\text{ placements}
= 585
\]

The multiple technical readings collected during each placement are not counted as separate placement measurements.

---

## 13. Exploratory SEN0600 measurements

The SEN0600 will be tested separately from the 15 analog sensors because:

- it needs different firmware and communication;
- its geometry is different;
- it may disturb a larger soil volume;
- one unit cannot estimate manufacturing variability.

Procedure:

1. complete the analog-sensor measurements first;
2. prepare a separate fresh or minimally disturbed soil portion;
3. use a container large enough to prevent wall effects;
4. record the SEN0600-reported moisture percentage and temperature;
5. compare its output with known GWC and derived VWC;
6. label all conclusions as applying to the tested unit only.

The SEN0600 data are exploratory and will not be used as ground truth.

If promising, design a separate experiment with at least two or more units and container-size controls.

---

## 14. Repeat and exclusion rules

A placement may be repeated when a documented procedural failure occurs, including:

- data logger failure;
- loose electrical connection;
- incorrect sensor ID;
- incorrect insertion depth;
- sensor touching the cup wall or bottom;
- final soil height outside tolerance;
- failure to reach the fill line within permitted force;
- visible large air gap caused by insertion;
- significant accidental soil loss;
- incorrect water or soil mass;
- cup left unsealed long enough to cause meaningful evaporation.

A measurement must not be automatically repeated or excluded because:

- its value looks too high or too low;
- one unit differs from the other four;
- a clone performs badly;
- the curve is non-monotonic;
- the reading worsens expected results;
- the sensor appears to be an outlier.

Unexpected valid results are part of the product-reliability dataset.

Every repeat or exclusion must contain:

- original measurement ID;
- reason;
- operator note;
- replacement measurement ID, if repeated.

---

## 15. Minimum data to record

### 15.1 Sensor table

- sensor ID;
- manufacturer group;
- purchase source;
- purchase batch;
- component markings;
- dimensions;
- photographs;
- endpoint measurements.

---

## 16. Analysis goals reserved for later

The present protocol freezes data collection, not the final statistical analysis.

Later analysis will address:

1. similarity of normalized curves within each manufacturer;
2. transfer of normalization between manufacturers;
3. raw and normalized sensor-to-sensor variability;
4. prediction error for GWC;
5. prediction error for derived VWC;
6. repeatability error caused by repacking and insertion;
7. batch-preparation variability;
8. original DFRobot versus clone performance;
9. exploratory comparison with the tested SEN0600 unit.

A provisional decision rule may later be tested:

```text
Irrigate when VWC < 25%
Do not irrigate yet when VWC ≥ 25%
```

This is an analytical placeholder only, not an agronomic recommendation.

Field capacity, wilting point, root-zone depth, crop type, and allowable depletion are outside the present experiment.

---

## 17. Protocol freeze conditions

This protocol is considered frozen once the following are selected and recorded:

- final dry-soil mass per cup;
- accepted maximum packing force;
- final reading window from the startup test;
- actual sensor power-cycle method;
- exact DFRobot capacitive model;
- final cup and disk dimensions;
- sensor insertion jig or depth mark;
- randomization procedure;
- data-file format.

Any material change after Stage 1 begins must be documented and may prevent Stage 1 data from being combined with Stage 2.
