# Soil Moisture Field Deployment — Version 2

## 1. Goal of the project

Deploy one field sensor node to measure soil temperature and soil-moisture trends for one month without maintenance.

The node writes readings to non-volatile storage. At the end of the deployment, the data is transferred to a laptop for analysis.

This is an experimental base for a future low-cost agricultural sensor that should eventually survive outdoors for years with limited maintenance. Version 1 is not intended to prove multi-year durability. It is intended to identify weak points, validate the enclosure strategy and collect one month of field data.

---

## 2. Scope and current limitations

### Objectives for this deployment

- Verify that the enclosure survives outdoor exposure for one month.
- Verify that the soil-moisture probe survives direct exposure to soil, rain and irrigation splashes.
- Verify that the DS18B20 assembly survives prolonged soil and water exposure.
- Log soil-moisture trend data, soil temperature and internal enclosure humidity.
- Identify whether condensation forms inside the enclosure.
- Collect enough data to improve the next hardware revision.

### Known limitations

- Only one soil-moisture probe is deployed.
- The probe does not represent the entire root zone or the spatial variability of the field.
- The system cannot determine whether water is moving too deeply below the root zone.
- The capacitive soil sensor is a low-cost relative sensor. Its readings require local calibration.
- The inexpensive vent, coating, cable and probe assemblies are treated as experimental components unless verified by testing.
- This version is a field-instrumentation prototype, not yet a complete irrigation-scheduling controller.

---

## 3. Evidence levels used in this document

| Evidence level | Meaning |
|---|---|
| Confirmed design choice | Decision selected for this prototype |
| Vendor claim | Product-page statement not independently verified |
| Bench-tested | Verified on the actual component or assembly before deployment |
| Field-tested | Verified after the one-month deployment |
| Pending | Must be decided or measured later |

---

## 4. Components and products

| ID | Component | Selected product or description | Link | Notes |
|---|---|---|---|---|
| SOIL | Soil-moisture sensor | Capacitive Soil Moisture Sensor v1.2 |  | Low-cost analog capacitive sensor. Must be calibrated individually. |
| BME | Internal humidity and temperature sensor | BME280 |  | Used primarily to monitor enclosure humidity and temperature. Leave sensing opening uncoated. |
| ESP32 | Microcontroller | ESP32-WROOM-32D DevKit |  | Logs readings to internal LittleFS storage. |
| CASE | Outdoor enclosure | IP65 junction box, 150 × 110 × 70 mm, supplied with M20 cable-gland positions |  | Final gland compatibility and internal layout must be checked after delivery. |
| VENT | Breather vent | Clockjuan IP68 screw breather vent valve, M12 × 1.5 | https://www.amazon.it/-/en/gp/product/B0FY75CG4L | No datasheet available. Must remain installed during enclosure tests. |
| TEMP | Soil-temperature probe | AZDelivery stainless-steel DS18B20 probe | https://www.amazon.it/AZDelivery-Temperature-Stainless-Waterproof-Compatible/dp/B075FYYLLV | Stainless-steel probe assembly sold as waterproof. Approximately 6 mm probe diameter and 4.2 mm cable diameter. |
| BANK | Power supply | 20,000 mAh USB power bank, 22.5 W | https://www.amazon.it/-/en/gp/product/B0G2XWPXCV | | Auto-off behavior during deep sleep must be tested. |
| SEALANT | Conformal coating | Electronic Protection Paint Silicone Sealant | https://www.amazon.it/-/en/gp/product/B0FH28QD3R | | Intended as moisture-resistant electronics coating. Not treated as a certified immersion barrier. |
| CABLE | Soil-sensor cable | 3-core stranded extension cable, 3 × 0.3 mm², PVC jacket, approximately 4.6 mm outer diameter, 10 m | https://www.amazon.it/3%C3%970-3mm%C2%B2-Conductors-Extension-Stranded-Matugajp/dp/B0C14L51X7 | Cable remains above ground in this version. Full-length ADC stability test required. |
| GLAND | Cable glands | hb-digital IP68 M12 cable glands | https://www.amazon.it/-/en/hb-digital-Waterproof-Resistant-Industrial-Installation/dp/B0CLLQ3PYK | Intended for the soil-sensor cable and DS18B20 cable. Confirm supplied clamping range, gasket and locknut. |
| SHRINK-L | Large adhesive-lined heat-shrink tube | Dual-wall adhesive-lined heat-shrink tubing, 4:1 ratio, 32 mm initial diameter |  | Covers the upper PCB electronics area. Recovered diameter is approximately 8 mm. |
| SHRINK-S | Small adhesive-lined heat-shrink tube | Dual-wall adhesive-lined heat-shrink tubing, 4:1 ratio, 12 mm initial diameter |  | Overlaps the large tube and seals around the 4.6 mm cable transition. Recovered diameter is approximately 3 mm. |
| STRIPPER | Wire stripper | VCELINK automatic wire stripper | https://www.amazon.it/-/en/VCELINK-Automatic-Electric-Stripping-Precision/dp/B0DYRT82GN | Test on a spare cable section before final assembly. |

---

## 5. Overall structure

The node has three physical sections:

1. the above-ground enclosure;
2. the mechanical support between the enclosure and the soil;
3. the in-soil sensors.

```text
                above-ground enclosure
       ┌─────────────────────────────────┐
       │ ESP32 DevKit                    │
       │ power bank                      │
       │ perfboard                       │
       │ BME280                          │
       │ internal wiring                 │
       │ breather vent                   │
       └─────────────────────────────────┘
                  │               │
                  │               └── DS18B20 cable
                  │
                  └── soil-sensor 3-core cable

              support tube or stake
                        │
                        ▼
                 original soil level
       ─────────────────────────────────
                        │
             exposed sensing blade
                        │
                        ▼
``` 

---

## 6. Main enclosure

### 6.1 Installation strategy

The junction box is mounted vertically above the soil. The shorter side faces downward because the enclosure has cable-entry traces on that side.

The enclosure must not touch the soil and must not sit in standing water.

The enclosure color is white. No additional sun shield is required for the first deployment unless the logged internal temperature shows that it is necessary.

### 6.2 Functional requirements

- The enclosure remains above ground.
- Cable exits face downward where possible.
- The vent faces laterally or downward so that water cannot pool against it.
- Unused openings are sealed with blanking plugs.
- The power bank cannot move freely inside the case.
- The perfboard cannot move freely inside the case.
- Loose wires cannot pull on solder joints.
- Boards are mounted above the enclosure floor so that minor condensation does not collect around contacts.
- The BME280 sensing opening remains unobstructed and uncoated.
- The micro-USB power connection is secured against accidental disconnection.

### 6.3 Cable glands

Use two M12 IP68 glands:

| Gland | Cable | Approximate cable outer diameter | Requirement |
|---|---|---:|---|
| GLAND-1 | Soil-moisture sensor 3-core cable | 4.6 mm | Must clamp and seal correctly |
| GLAND-2 | DS18B20 cable | 4.2 mm | Must clamp and seal correctly |


### 6.4 Breather vent

The vent is a low-cost Clockjuan M12 × 1.5 breather valve sold as IP68.

The vendor claim is treated as a hypothesis to be tested. The vent is expected to help equalize internal and external pressure. It may reduce condensation risk, but it does not guarantee low internal humidity.

Installation requirements:

- lateral mounting is preferred if the enclosure geometry permits it;
- water must not pool directly against the vent membrane;
- the gasket must seal against a flat enclosure surface;
- the vent must remain installed during all enclosure tests.

### 6.5 Internal coating

Apply SEALANT as a conformal-coating layer to:

- the perfboard;
- solder joints;
- exposed component leads;
- relevant PCB edges;
- soldered wire terminals.

Do not coat:

- the BME280 humidity-sensing opening;
- USB connectors;
- removable contacts that must remain serviceable;
- buttons that must remain accessible.

The coating is a backup barrier against internal humidity and occasional condensation. It is not treated as full waterproofing.

### 6.6 Deliberate exclusion of desiccant

Version 1 intentionally excludes desiccant.

The purpose is to evaluate whether the enclosure, vent and coating strategy are sufficient without consumable humidity control. This increases the value of the BME280 humidity readings and provides evidence for future maintenance-free designs.

---

## 7. Mechanical support tube

The enclosure is held above the ground using an available iron or steel tube selected after the enclosure arrives.

### 7.1 Pending decisions

Record after selecting the tube:

| Property | Value |
|---|---|
| Material | likely PVC but can't be sure since the tube was found in the rubbish |
| External diameter | 59.50mm |
| Wall thickness | 2.65mm |
| Total length | 240mm |
| Buried depth | 50mm (enough so the cable glands don't touch soil) |
| Attachment method | Dig a hole, then insert the tube and the sensor, put soil in, compact each layer |

### 7.2 Requirements

- The tube is inserted manually into the soil.
- A visible depth mark indicates the required insertion depth.
- The enclosure remains stable when pushed laterally.
- Sharp edges cannot damage cables.
- Cable routing is protected from abrasion.
- Water must not collect in a way that damages cables or the enclosure.
- Rust is acceptable for the one-month prototype if it does not compromise stability.

---

## 8. Soil-moisture probe

### 8.1 Probe dimensions

| Measurement | Value |
|---|---:|
| Maximum width of upper electronics PCB section | 25 mm |
| Maximum thickness before connector removal | 10 mm |
| Distance from PCB top to white insertion line | 25 mm |
| Soil-sensor cable outer diameter | 4.6 mm |
| Cable conductor count | 3 |
| Individual conductor cross-section | 0.3 mm² |

The exact maximum thickness will be measured with the next batch of ordered soil sensors since the current ones have a known fault.

### 8.2 Insertion depth

The white insertion line printed on the sensor PCB is placed at the original soil level.

The active sensing blade below the line is inserted into the soil. The sealed upper electronics section remains above the initial soil line but may be exposed to:

- rain splashes;
- irrigation splashes;
- temporary pooling;
- soil movement;
- partial burial after irrigation or rain.

This is a deliberate stress condition for the first deployment.

### 8.3 Cable transition design

The original detachable white connector is removed. A continuous jacketed three-core cable is soldered directly to the PCB.

The three conductors are:

| Conductor function | Purpose |
|---|---|
| Supply voltage | Powers the probe |
| Ground | Common reference |
| Analog output | Soil-moisture sensor signal |

### 8.4 Protection stack

```text
active sensing blade
        │
        │ white insertion line at original soil level
        ▼
upper PCB electronics section
        │
        │ direct-soldered 3-core cable
        ▼
conformal coating on PCB faces, PCB edges and solder joints
        │
        ▼
large 32 mm 4:1 dual-wall adhesive-lined heat-shrink tube
covering the upper PCB electronics section and extending along the cable
        │
        ▼
small 12 mm 4:1 dual-wall adhesive-lined heat-shrink tube
overlapping the tail of the 32 mm tube and gripping the cable jacket
        │
        ▼
3-core cable to enclosure
```

### 8.5 Heat-shrink sizing rationale

| Tube | Initial diameter | Approximate recovered diameter | Purpose |
|---|---:|---:|---|
| SHRINK-L | 32 mm | 8 mm | Slides over the 25 mm PCB and covers the electronics area |
| SHRINK-S | 12 mm | 3 mm | Overlaps SHRINK-L and seals around the 4.6 mm cable transition |

The 32 mm tube is intentionally larger than the PCB. It is not expected to grip the cable tightly by itself.

The 12 mm tube provides the tighter cable-side seal and additional strain relief. It must overlap both the tail of the 32 mm sleeve and the external cable jacket.

### 8.6 Assembly sequence

1. Cut the required cable length.
2. Slide SHRINK-L and SHRINK-S onto the cable before soldering.
3. Strip only the minimum necessary length of outer jacket.
4. Strip the three inner conductors.
5. Remove the original white connector from the sensor PCB.
6. Solder the three conductors directly to the PCB.
7. Test the probe electrically.
8. Apply SEALANT to both faces of the upper PCB, PCB edges and solder joints.
9. Allow the coating to cure fully for at least 24 hours.
10. Place SHRINK-L over the upper PCB and cable transition.
11. Shrink SHRINK-L gradually with controlled hot air.
12. Place SHRINK-S over the tail of SHRINK-L and the cable jacket.
13. Shrink SHRINK-S gradually with controlled hot air.
14. Confirm that adhesive is visible around the cable-side seal.
15. Allow the assembly to cool.
16. Repeat electrical reference measurements.
17. Photograph the final assembly before testing and deployment.

### 8.7 Important construction rules

- Do not allow the heat-shrink tubing to extend below the white insertion line into the active sensing area.
- Do not use an open flame.
- Do not leave the cable under tension.
- Do not rely on SEALANT alone for the cable transition.

---

## 9. DS18B20 soil-temperature probe

### 9.1 Known dimensions and claims

| Property | Value |
|---|---|
| Sensor type | Stainless-steel DS18B20 probe assembly |
| Probe diameter | Approximately 6 mm |
| Cable diameter | Approximately 4.2 mm |
| Vendor temperature range | −55 °C to 125 °C |
| Vendor accuracy claim | ±0.5 °C |
| Vendor waterproof claim | Can be used under water |
| IP rating | Not specified |

### 9.2 Installation

- Route the cable through GLAND-2.
- Install the probe at a documented depth.
- Record the distance from the soil-moisture probe.
- Inspect the metal-to-cable transition carefully.
- Optional: reinforce the metal-to-cable transition with adhesive-lined heat-shrink tubing if a suitable size is available.

---

## 10. Power supply and current measurement

### 10.1 Power-bank strategy

The 20,000 mAh USB power bank powers the ESP32 DevKit through micro-USB.

Details at hardware/iniu_power_bank_20Ah.md.

---

## 11. Data storage

The ESP32 writes readings to internal LittleFS storage.

| Property | Value |
|---|---:|
| LittleFS partition | 2944 KiB |
| Sampling interval | 15 minutes |
| Approximate bytes per reading | 48 bytes |
| Readings per day | 96 |
| Planned duration | 30 days |
| Approximate total storage | 138,240 bytes |

The available LittleFS partition is sufficient for the planned deployment.

Firmware is already substantially tested. Before deployment, a 2 days test confirmed that:

- readings are written successfully;
- timestamps remain usable;
- files remain readable after simulated interruptions;
- the power bank does not switch off during deep sleep.

---

## 12. Pre-deployment test plan

### 12.1 Coating screening test

Purpose: verify whether SEALANT adheres adequately and remains stable after water exposure.

Procedure:

1. Apply SEALANT to a spare PCB fragment, spare sensor or soldered test wires.
2. Allow it to cure fully for at least 24 hours.
3. Immerse the sample in water for at least 72 hours.
4. Inspect for peeling, swelling, softening and visible water pathways.
5. Flex the coated wire transition gently.
6. Inspect again.

Acceptance criteria:

- no visible peeling;
- no large cracks;
- no obvious swelling or softening;
- no exposed metal caused by coating failure.

Test failed. The coating hasn't formed cloudy areas, bubbles or wrinkles but I can still easily peel it of using my nail.
Decided to still proceed with this bad quality coating since coat is a second protection only.

### 12.2 Soil-sensor full-length cable ADC test

Purpose: verify that the 10 m analog cable does not introduce excessive noise.

Procedure:

1. Record at least 100 readings with the original short cable in a stable environment.
2. Record at least 100 readings with the final 10 m cable in the same stable environment.
3. Compare mean, standard deviation and outliers.
4. Repeat with the cable routed close to the final power wiring and irrigation equipment if available.

Acceptance criteria:

- no large unexplained spikes;
- no unstable ADC behavior;
- any constant offset is documented and handled through calibration.

Test passes. With its main cable the soil sensor reads 928mV. With the full length (10m) cable, 933mV.

### 12.3 Finished soil-probe soak and flex test

Purpose: verify the completed probe assembly before burial.

Baseline:

- record ten stabilized readings in air;
- record ten stabilized readings in water;
- photograph the completed transition.

Procedure:

1. Immerse the active blade, sealed electronics area, cable transition and at least 10 cm of cable in tap water.
2. Keep the opposite cable end dry.
3. Power the probe periodically using the normal measurement cycle.
4. Log one reading every 15 minutes for at least 72 hours.
5. Inspect for fixed output, sudden jumps, drift, coating separation and adhesive detachment.
6. Bend the cable gently approximately 20 times in each direction near the sealed transition.
7. Apply a 0.5 kg pull load for 60 seconds.
8. Repeat the soak for another 24 hours.
9. Dry the external surface.
10. Repeat air and water reference readings.

Provisional acceptance criteria:

- no loss of electrical function;
- no intermittent readings;
- no visible water pathway into the cable transition;
- no detached heat-shrink tubing;
- post-test reference readings remain within approximately 5% of the original air-to-water voltage span.

Passes.

### 12.4 DS18B20 soak and flex test

Purpose: screen the third-party waterproof probe assembly.

Procedure:

1. Record the probe reading in a stable room-temperature water bath.
2. Immerse the stainless-steel probe, metal-to-cable transition and at least 10 cm of cable for 72 hours.
3. Log readings periodically.
4. Bend the cable gently approximately 20 times near the transition.
5. Apply a 0.5 kg pull load for 60 seconds.
6. Immerse it for another 24 hours.
7. Repeat the stable water-bath comparison.

Acceptance criteria:

- no missing readings;
- no intermittent connection;
- no visible separation at the metal-to-cable transition;
- no unexplained post-test shift greater than approximately 0.5 °C in the same stable water bath.

Passes.

### 12.5 Enclosure leak and vent test

Purpose: test the completed enclosure with the final vent and cables installed.

Final assembly must include:

- actual vent;
- actual cable glands;
- actual cables;
- blanking plugs;
- lid gasket;
- final orientation.

Procedure:

1. Remove sensitive electronics if necessary, but leave the final cables, glands, vent and plugs installed.
2. Place dry tissue paper or a moisture-indicator card inside.
3. Close the lid normally.
4. Expose the enclosure to irrigation-like spray from multiple directions.
5. Check for visible ingress.
6. Optionally perform a short immersion stress test for approximately 5 seconds as an additional informal check.
7. Leave the enclosure outdoors overnight to experience a temperature cycle.
8. Inspect again.

Acceptance criteria:

- no visible liquid water inside;
- tissue paper remains dry;
- no water pathway through glands, plugs, lid gasket or vent.

Passes.

### 12.6 Mechanical support test

Purpose: verify stability after selecting the tube.

Procedure:

1. Install the tube at the planned burial depth.
2. Attach the enclosure.
3. Apply a repeatable horizontal load at enclosure height.
4. Inspect movement and attachment points.
5. Verify that cables are not pinched or cut.

The exact load must be recorded after selecting the support tube.

#OPEN

### 12.7 Power-bank auto-off and runtime test

Purpose: verify that the power bank remains active during deep sleep.

Procedure:

1. Assemble the final electronics.
2. Use the real sampling interval of 15 minutes.
3. Run the node continuously for at least 72 hours.
4. Confirm that wake-up cycles continue without interruption.
5. Record power-bank depletion using an inline USB power meter if available.

Acceptance criteria:

- no automatic shutdown;
- no unexpected reset loop;
- projected runtime sufficient for one month with a safety margin.

Test passes. Details at hardware/iniu_power_bank_20Ah.md.

---

## 13. Soil calibration plan

Calibration and field-capacity estimation are separate experiments.

### 13.1 Individual sensor calibration

The exact deployed sensor is calibrated before soldering, coating and heat-shrink installation.

Tests must be run, in a controlled environment, that:

- using the 3-core cable don't meaningfully change the readings. Tested.
- coating don't meaningfully change the readings. #OPEN
- Adding heat-shrink tube don't meaningfully change the readings. #OPEN

### 13.2 Gravimetric calibration experiment

Purpose: map raw sensor millivolts to gravimetric soil-water content for the selected field soil.

Results at logbook/soil_sensor_curve_fitting_experiment.md.

### 13.3 Field-capacity experiment

Purpose: estimate the water content retained after rapid gravitational drainage has mostly stopped.

Procedure:

1. Pack the selected field soil consistently in a container with drainage holes.
2. Saturate the soil.
3. Allow excess water to drain freely.
4. Wait until rapid drainage has largely stopped.
5. Record retained soil mass and sensor reading.
6. Calculate retained water mass.
7. Record the approximate field-capacity value.

Field validation:

- after heavy irrigation or rainfall, compare field readings after approximately 12–24 hours;
- heavier soils may require a longer drainage period;
- document the actual soil type and drainage behavior.

#OPEN

### 13.4 Permanent wilting point

PWP is estimated from soil type and published reference values for this prototype.

This must be labeled as an estimate, not as a measured property of the field.
#OPEN

### 13.5 Available water-holding capacity

Calculate a preliminary available water-holding capacity using the estimated field capacity and estimated PWP.

The result is a first approximation and must be checked for plausibility against the soil type.
#OPEN

---

## 14. Bulk-density measurement

The goal is a meaningful conversion from gravimetric water content to volumetric water content.

Use a DIY undisturbed-soil core sampler made from a rigid metal cylinder of known internal diameter and length.

### 14.1 Core volume

```text
core volume = π × radius² × height
```

### 14.2 Procedure

1. Select the same depth range where the soil-moisture probe will be installed.
2. Press or gently drive the metal cylinder into undisturbed soil.
3. Excavate around the cylinder carefully.
4. Remove it without losing soil.
5. Trim soil flush with both ends.
6. Dry the soil using the oven-drying procedure.
7. Weigh the dry soil.
8. Calculate bulk density.

```text
bulk density = dry soil mass / undisturbed core volume
```

### 14.3 Replicates

Take at least three samples near the deployment area.

Record:

| Sample | Depth range, cm | Core diameter, cm | Core length, cm | Core volume, cm³ | Dry soil mass, g | Bulk density, g/cm³ | Notes |
|---|---:|---:|---:|---:|---:|---:|---|
| 1 |  |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |

Document stones, roots, cracks and damaged samples rather than silently ignoring them.

---

## 15. Oven-drying procedure

Use equipment that will not later be used for food preparation.

### Procedure

1. Label each heat-resistant container.
2. Weigh the empty container.
3. Add the soil sample.
4. Weigh the container plus moist soil.
5. Dry at approximately 105 °C for 24 hours.
6. Remove the container and allow it to cool inside a sealed dry container.
7. Weigh promptly.
8. Dry for another 2–4 hours.
9. Cool and weigh again.
10. Repeat until the mass change is negligible relative to the precision of the scale.

Example provisional stability criterion:

- if the scale resolves 0.1 g, treat a change smaller than approximately 0.1–0.2 g as stable.

Do not insert the capacitive sensor into hot soil. Wait until the soil reaches a stable room temperature.

---

## 16. Field installation record

Complete this section at installation time.

| Property | Value |
|---|---|
| Deployment ID |  |
| Installation date |  |
| Planned retrieval date |  |
| Field name or area |  |
| Crop |  |
| Crop growth stage |  |
| Soil type |  |
| Irrigation method |  |
| Soil-moisture probe depth | White insertion line at original soil level |
| Soil-temperature probe depth |  |
| Distance between moisture probe and irrigation emitter |  |
| Distance between DS18B20 and moisture probe |  |
| Moisture-probe orientation |  |
| Enclosure height above soil |  |
| Support-tube buried depth |  |
| Sun exposure |  |
| Can water pool around the sensor? |  |
| Can water pool around the support tube? |  |
| Cable routing notes |  |
| Photo filenames |  |

#OPEN

---

## 17. Field observations

| Date and time | Weather | Irrigation event | Rain event | Soil condition | Enclosure condition | Internal humidity observation | Notes |
|---|---|---|---|---|---|---|---|
|  |  |  |  |  |  |  |  |

---

## 18. Acceptance criteria for the one-month deployment

The deployment is considered successful if:

- the enclosure contains no visible liquid water after retrieval;
- the vent, glands and blanking plugs show no obvious ingress path;
- the power bank does not shut down unexpectedly;
- the support tube remains stable;
- the soil probe continues responding after retrieval;
- the DS18B20 continues responding after retrieval;
- the sensor cables show no major abrasion or jacket failure;
- the LittleFS log remains readable;
- missing readings remain below the chosen tolerance;
- internal humidity and temperature logs are usable;
- post-deployment soil-probe air and water readings can be compared with pre-deployment references.

The deployment is considered unsuccessful if:

- data logging stops unexpectedly;
- the power bank switches off;
- visible water enters the enclosure;
- the soil-probe seal detaches;
- corrosion appears on exposed PCB areas;
- the DS18B20 connection becomes intermittent;
- the enclosure overheats repeatedly;
- the storage file becomes unreadable.

---

## 19. Pre-deployment photographs

Take photographs of:

- the unmodified soil sensor;
- the soil-sensor PCB after removing the white connector;
- direct solder joints;
- cured conformal coating;
- SHRINK-L and SHRINK-S installation;
- final soil-sensor transition;
- DS18B20 cable-to-metal transition;
- cable glands;
- breather vent;
- enclosure interior;
- perfboard mounting;
- power-bank mounting;
- completed enclosure;
- field installation before and after inserting the probes.

---

## 20. Post-deployment inspection

After retrieval:

1. Photograph the system before cleaning.
2. Inspect the enclosure interior for water marks and condensation residue.
3. Inspect cable glands and blanking plugs.
4. Inspect the vent.
5. Inspect the soil-probe heat-shrink transition.
6. Inspect PCB coating for peeling or cracks.
7. Inspect cables for abrasion and UV damage.
8. Inspect the DS18B20 cable-to-metal transition.
9. Repeat soil-probe air and water reference readings.
10. Repeat the DS18B20 stable water-bath comparison.
11. Compare pre-deployment and post-deployment values.
12. Record failures and design changes for Version 3.

---

## 21. Pending decisions and measurements

### Resolve after the enclosure arrives

| Item | Question | Status |
|---|---|---|
| CASE | Are the cable-entry points threaded holes, drilled holes or removable knockouts? | Pending |
| CASE | Can M12 glands be fitted directly, or are reducers needed? | Pending |
| CASE | How are unused openings sealed? | Pending |
| VENT | Is lateral mounting possible? | Pending |
| VENT | Can water pool against the selected location? | Pending |
| SUPPORT | Which tube is used? | Pending |
| SUPPORT | How is the enclosure attached? | Pending |
| INTERNAL | How are perfboard and power bank secured? | Pending |

#OPEN

### Resolve during sensor assembly

| Item | Question | Status |
|---|---|---|
| SOIL | Maximum PCB thickness after removing white connector | Pending measurement |
| SOIL | Does SHRINK-L slide over the PCB without damaging components? | Pending test |
| SOIL | Does SHRINK-S seal tightly around SHRINK-L and the cable jacket? | Pending test |
| CABLE | Does the full 10 m cable produce stable ADC readings? | Pending test |
| SEALANT | Does the coating survive soak and flex screening? | Pending test |
| TEMP | Does the DS18B20 survive soak and flex screening? | Pending test |
#OPEN

### Resolve at field installation

| Item | Question | Status |
|---|---|---|
| SITE | Exact distance from irrigation emitter | Pending |
| SITE | DS18B20 depth | Pending |
| SITE | Soil-sensor orientation | Pending |
| SITE | Sun exposure | Pending |
| SITE | Water-pooling risk | Pending |


#OPEN
