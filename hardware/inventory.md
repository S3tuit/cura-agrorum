# Soil Sensor Hardware Inventory

## Conventions

- **Qty**: current count in working stock
- **Unit ID**: physical label attached to the part
- **Notes**: short one-line note; deeper documentation belongs in the component-specific README

---

## Inventory

### Microcontrollers

| Part | Description | Qty | Notes |
|---|---:|---:|---|
| ESP32-WROOM-32D DevKit | ESP32 development board | 2 | |

### Environmental sensors

| Part | Description | Qty | Notes |
|---|---:|---:|---|
| GY-BME280 | Temperature, humidity, and pressure sensor | 3 | |
| DS18B20 | Waterproof stainless-steel temperature probe | 5 | |
| Soil Moisture Capacitive Module V1.2 | Capacitive soil moisture sensor | 5 | |

### Prototyping

| Part | Description | Qty | Notes |
|---|---:|---:|---|
| Breadboard | 830-point, 2.54 mm pitch, 165 × 55 mm | 2 | Split in the middle |

### Cables / connectors

| Part | Description | Qty | Notes |
|---|---:|---:|---|
| CABLEPELADO Micro USB Sync and Charge | USB-A to Micro USB-B cable, 1 m | 1 | USB 2.0; supports data transfer up to 480 Mbit/s |

---

## Unit calibration log

### BME280 units

| Unit ID | Chip ID (`0xD0`) | I²C addr | Temp offset | Humidity offset | Notes |
|---|---:|---:|---:|---:|---|
| BME-01 | ? | ? | ? | ? | First unit; verify on arrival |

### DS18B20 units

| Unit ID | ROM serial | Temp offset | Notes |
|---|---:|---:|---|
| DS-01 | ? | ? | |

### Capacitive soil moisture units

| Unit ID | Dry ADC (air) mV | Wet ADC (submerged) mV | Notes |
|---|---:|---:|---|
| 1 | 2180 | 660 | |
| 2 | ? | ? | |
| 3 | ? | ? | |
| 4 | ? | ? | |
| 5 | ? | ? | |

---

## Known-bad / retired units

Empty for now.

---

## Order history

### Order 1

| Item | Qty | Cost |
|---|---:|---:|
| Soil Moisture Capacitive Module V1.2 | 5 | 9.79 |
| CABLEPELADO Micro USB Sync and Charge | 1 | 5.99 |
| BME280 | 3 | 15.39 |
| DS18B20 | 5 | 12.74 |
| Breadboard 830 pins | 2 | 10.02 |
| ESP32-WROOM-32D | 2 | 17.62 |
| **Total** |  | **71.55** |
