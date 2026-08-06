# Field Deployment v2 — LoRa Pilot

## Purpose

Deploy one sensor node for at least one week to validate the next low-cost field-monitoring design before creating a custom PCB.

This deployment is a communications and measurement pilot, not a battery-life or solar-autonomy test.

## Goals

- Validate an EU868 LoRa link in the intended field, enclosure and antenna positions.
- Measure reception rate, retries, RSSI and SNR under realistic vegetation and weather.
- Measure soil moisture and temperature at two depths using two moisture probes and two DS18B20 probes.
- Verify that the powered receiver can timestamp readings, retain them while the Internet is unavailable and later forward them.
- Measure the duration and current of each sensor-node operating state and build a six-month energy budget.
- Identify the final power, radio, antenna and enclosure requirements for the custom PCB.

## Selected design

### Sensor node

| Part | Selection | Status and reason |
|---|---|---|
| MCU | ESP32-C6, initially on two ESP32-C6-DEVKITM-1-N4 boards | One board is for the field node and one for bench work or as a spare. |
| MCU alternative | STM32WLE5 | Not selected/ordered. Its integrated LoRa radio could reduce the final component count, but changing to STM32Cube would require a firmware port and its smaller internal flash could require external storage. The C6 retains the existing ESP-IDF work and 4 MB flash. |
| LoRa prototype | Two Waveshare Pico-LoRa-SX1262-868M, EU868, no-battery version | Each kit includes a 2 dBi SMA antenna and an IPEX-to-SMA cable. |
| Soil moisture | Two low-cost capacitive probes | Final model pending calibration results and experiment at `logbook/soil_sensor_curve_fitting_experiment.md` . |
| Soil temperature | Two waterproof DS18B20 probes | Exact depths pending. |
| Enclosure monitor | BME280 | Retained from the current deployment to measure internal temperature and humidity. |
| Temporary power | Existing 20 Ah USB power bank | Selected for this pilot only. The awake load must remain high enough to prevent automatic shutdown. |

### Receiver

| Part | Selection | Status and reason |
|---|---|---|
| Computer | Raspberry Pi 3 Model B |  |
| Clock | Adafruit DS3231 with CR1220 battery |  |
| Storage | Reputable 32 GB A1 microSD | A consumer A1 card is acceptable because limited recent-data loss and field replacement are tolerable. |
| Radio | One EU868 SX1262 interface and supplied antenna | Physical integration remains to be finalized. |
| Power | Continuous external supply | Exact field supply pending. |

## Work before the custom PCB

1. Complete moisture-probe comparison and calibration; select two probes and their installation depths.
2. Assemble one C6/SX1262 sensor node and one powered receiver using development hardware.
3. Run a 48–72 hour bench test, including power-bank auto-off, receiver-clock and Internet-outage tests.
4. Run the field pilot for at least one week using the intended enclosures and antenna placement.
5. Evaluate link reliability and repeat the test at weak or obstructed positions if link margin is insufficient.
6. Measure the C6 through its J5 current header and measure the LoRa board and sensors separately.
7. Combine measured state energy with field retry rates to estimate six-month consumption.
8. Select the final battery chemistry, regulator, solar panel and charging circuit.
9. Decide between SX1261 and SX1262 for the PCB and finalize the antenna and RF module or reference design.
10. Create the schematic and PCB, then verify its sleep current before a longer deployment.

## Open decisions

- Choose the two soil depths after reviewing crop root depth and installation constraints.
- Choose the receiver's continuous field power source.
- Select the final SX1261 or SX1262 implementation after the pilot and energy measurements.
