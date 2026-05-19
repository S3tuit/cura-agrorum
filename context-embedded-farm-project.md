# Architecture: Toy → v1

## Status

These are working decisions, not final designs. Anything in this document
can change if a better approach emerges during the build. The point is to
have a shared reference and to make the trajectory from toy to v1 explicit.

If during construction a tech choice doesn't work, or a simpler approach
becomes obvious, we change it and update this doc.

## Overview

Two scoped projects, sharing as much architecture as possible:

- **Toy**: indoor houseplant auto-irrigation. Validates the closed-loop
  control pattern and the server/bot stack on a deliberately small problem.
  ~3 week build target.
- **v1**: field sensor network at the farm. Battery + solar nodes in real
  soil, gateway at the farmhouse, alerts to grandparents. The actual product
  target. Years of iteration expected.

The toy is designed so that the majority of its code (server-side stack,
sample loop, buffering, bot logic) ports forward to v1 with no rewrite.
The transport layer (WiFi vs LoRa) and the field-hardening (power, weather,
enclosure) are where the two diverge.

---

## Toy version

### Goal

Single ESP32 + capacitive soil sensor + small pump waters one houseplant
from a 2L water bottle. Alerts on low water level. Graphs on demand.

### Architecture

```
ESP32 (WiFi)
    ↓  via home WiFi
Laptop (Python + FastAPI + Postgres + Telegram bot)
    ↓  Telegram Bot API
My phone (Telegram)
```

### Hardware

- ESP32 DevKit (WROOM-32D, micro-USB)
- Capacitive soil moisture sensor v1.2 (hot-glued electronics region)
- Diaphragm pump, 6–12V, ~1.5 L/min
- IRLZ44N logic-level MOSFET (TO-220)
- 1N4007 flyback diode across the pump
- 10kΩ gate pull-down on the MOSFET (mandatory: prevents pump pulse on boot)
- 12V 2A wall adapter for the pump
- USB power for the ESP32 (no battery, no power constraint indoor)
- BME280: optional, useful for room conditions, not load-bearing for control

### Power

- **ESP32**: USB from a wall charger or laptop. No power budget, no battery.
- **Pump**: 12V wall adapter, switched via MOSFET. **Common ground with
  the ESP32 is mandatory.**

Two power domains, one shared ground.

### Firmware

- **Language**: C, via ESP-IDF (the right place to apply C strength)
- **Sample loop**: read soil sensor every N seconds, decide whether to
  actuate the pump, post measurement to the server
- **Buffering**: write to ESP32 NVS when server is unreachable; drain
  the backlog when the server comes back. **Built in from day one.**

### Server

- **Lives on the laptop** during development
- **Python 3 + FastAPI** for the HTTP ingest endpoint
- **PostgreSQL** for storage (consistent with v1, leverages existing skill)
- **python-telegram-bot** for the bot, async, long-polling
- **matplotlib** to PNG for graphs, sent as Telegram images
- One process or two (FastAPI + bot in same process via asyncio is fine)

### User interface

- Telegram bot. No native app, no web dashboard.
- Commands: `/status` (latest reading), `/graph` (24h PNG),
  `/water` (current bottle estimate)
- Push alerts: water-low threshold, ESP silent for > 1h

### Water-low detection

Server-side, not ESP-side:

1. ESP reports each pump event ("ran pump 5.2s at timestamp T")
2. Server keeps cumulative volume in the DB
3. Server computes remaining from initial bottle volume
4. Server sends Telegram alert when threshold is crossed
5. Open question; does the sensor decide when to pump the water or is it
   server's responsibility?

Threshold and bottle volume are tweakable on the server without reflashing.

---

## v1 version

### Goal

Real sensor network at the farm. 2–5 nodes initially, scaling slowly.
Battery + solar, surviving real conditions (heat, humidity, condensation,
insects). Alerts to the grandparents on threshold breaches. Years of
iteration over multiple growing seasons.

### Architecture

```
Field sensors (ESP32 + LoRa, battery + solar, in IP66 enclosures)
    ↓  LoRa @ 868MHz (EU ISM band)
Gateway at the farmhouse (Pi + LoRa receiver + cellular or home WiFi)
    ↓  HTTP POST over internet
VPS in a datacenter (Postgres + FastAPI + Telegram bot)
    ↓  Telegram Bot API
Grandparents' phones and mine (Telegram)
```

### Hardware (sensor node)

Same control logic as the toy, but with field-hardening:

- ESP32 (probably the bare module on a custom PCB by this point)
- LoRa module: SX1276 / SX1278 (868MHz for EU)
- Solar panel (~5W), LiPo battery, TP4056 or similar charge controller
- IP66 enclosure with cable glands
- Conformal coating on the perfboard/PCB except sensors that breathe (BME)
- Custom PCB once the design has stabilized (post-v0.5, not before)

### Hardware (gateway)

- Raspberry Pi (3 or 4, whatever's available cheap)
- LoRa receiver module (matches the node radios)
- Internet uplink — choose what the farm has:
  - **Existing home internet** at the farmhouse (Ethernet or WiFi)
  - **4G/LTE** via USB modem (~€30–50) or HAT (SIM7600, ~€60–80) + IoT SIM
    (€5–10/month). Default fallback — works on any Italian farm with cell
    signal, no dependency on the farmer's existing setup.
  - **Starlink** if genuinely remote. Overkill for one farm.
- AC power from the farmhouse mains (no battery needed for gateway)

### Power

- **Nodes**: solar + LiPo, designed for unattended multi-week operation.
  Power budget is the design driver (radio duty cycle, deep sleep, etc.)
- **Gateway**: AC mains at the farmhouse. No battery constraint.
- **Server**: VPS in a datacenter. No farm dependency at all.

### Communication

Three hops:

1. **Sensor → Gateway**: LoRa @ 868MHz. Custom protocol (simple framed
   packets) or LoRaWAN. Decision deferred until the 2nd or 3rd node, when
   the tradeoffs become real.
2. **Gateway → Server**: HTTP POST over whatever uplink. Gateway buffers
   locally when the uplink is down.
3. **Server → User**: Telegram bot API, same as the toy.

### Server

- **Moved from laptop to a VPS** (€3–5/month: Hetzner, OVH, Contabo).
  24/7 uptime, public IP.
- **Same stack as the toy**: Python + FastAPI + Postgres + python-telegram-bot
  + matplotlib. Code is git pull'd from the toy's repo.
- Why VPS, not Pi-as-server:
  - Bot stays reachable when farm internet is flaky
  - Updates deploy to one machine over SSH, no farm visit needed
  - Farm uplink only needs to drain the gateway's buffer occasionally,
    not be continuously up

### Buffering

Three layers of buffering against intermittent connectivity:

1. **Sensor → Gateway**: ESP NVS buffer (already built in toy)
2. **Gateway → VPS**: gateway local disk buffer (new code for v1)
3. **VPS → User**: trivial, the bot just retries Telegram

Each layer assumes the next is sometimes gone.

---

## What ports forward (toy → v1)

**Identical, no rewrite:**

- Server-side Python (FastAPI + Postgres + Telegram bot + matplotlib)
- Database schema (with extensions for multi-node IDs)
- ESP32 sample loop, JSON formatting, threshold logic, NVS buffering
- Telegram bot commands and alert logic
- General "server is sometimes unreachable, keep working" patterns

**Changes:**

- ESP32 transport: WiFi/HTTP → LoRa packets (new radio code, same message
  semantics)
- New gateway component: ~100–150 lines of Python (LoRa receive → HTTP
  forward, with local buffer)
- Power management on the node: meaningful for the first time
  (battery budget, sleep modes, radio duty cycle)
- Field hardening: IP rating, conformal coating, strain relief,
  weatherproofing
- Per-node identification, calibration, time-sync across multiple radios

So building the toy is building ~70% of v1. The WiFi assumption sheds
cleanly at the gateway boundary — sensors get a different radio, gateway
gets a different uplink, the server doesn't notice.

---

## Open questions / deferred decisions

- **LoRa raw vs LoRaWAN.** Raw is simpler and fully private; LoRaWAN gives
  access to public gateway networks (The Things Network). Defer until the
  2nd or 3rd node when the tradeoff is concrete.
- **MQTT vs HTTP between gateway and VPS.** HTTP is fine for 1–10 nodes;
  MQTT becomes interesting at higher scale or for bidirectional commands.
- **Per-node calibration storage.** Probably in the server-side DB keyed
  by node ID, for consistency and remote updatability.
- **OTA firmware updates** on the nodes. Nice-to-have, real work. v1+ concern.
- **Multi-user Telegram** with per-user thresholds and chat IDs. Currently
  single-user (me). Add when grandparents are actually on the system.
- **BME280 in v1.** Frost prediction via dew point is a strong reason to
  have it; finalize once we know which crops/conditions matter.

---

## Things we decided against (explicit, not by accident)

- **Native phone app.** Telegram is zero-install for the users we have.
- **Rover for soil scanning.** Economics, localization difficulty, wrong
  skill set for now. Possible later hobby project.
- **MQTT on day one.** HTTP is simpler for the toy; revisit at v1+ scale.
- **SQLite for the toy server.** Postgres is what v1 will use. No reason
  to detour through SQLite when the team (me) already knows Postgres.
- **WhatsApp.** Telegram bot API is free and trivial. WhatsApp Business
  API requires Meta approval and per-message costs. Revisit only if the
  grandparents genuinely refuse Telegram.
- **Custom PCB before v0.5.** Perfboard is the intermediate step; you can't
  lay out a good PCB before you know what fails in the field.
- **C on the server side.** Python's ecosystem for HTTP + Postgres + Telegram
  + plotting is dominant. C goes where it matters: on the ESP32.

---

## Reminder

Working architecture, revisable. If something doesn't work, change it
and update this file. Treating this doc as fixed is a worse failure
mode than changing it too often.
