# Zero-IRQ investigation

The Pi receiver uses its normal production entry point and recovery behavior,
with temporary instrumentation in one file:
[`radio_investigation.py`](../../cura_receiver/radio_investigation.py).
The working folder on the Pi is `/home/cura/radio-unexpected-irq`.
Credentials are in `/etc/cura-radio-investigation/receiver-group.json` because
the strict loader requires trusted ownership of every parent directory.

## Wiring and scope

| Channel | Signal | Pi BCM GPIO | Pi physical pin |
|---|---|---:|---:|
| CH1 | Scope marker | 25 | 22 |
| CH2 | DIO1 | 23 | 16 |
| CH3 | MISO | 9 | 21 |
| CH4 | BUSY | 24 | 18 |

Use common circuit ground and x10/DC probes. Configure Single, CH1 rising
edge at 1.65 V, 10 ms/div, -40 ms horizontal delay, Normal acquisition,
10 Mpts/channel, 100 MSa/s, and full bandwidth. This retains 90 ms before
the marker and 10 ms after it. Keep all four channels enabled.

## Start

On the Pi:

```sh
cd ~/radio-unexpected-irq
sudo nmcli connection up "Siglent scope direct LAN"
ip route get 10.11.13.220
sudo systemctl start cura-radio-investigation.service
sudo journalctl -u cura-radio-investigation.service -n 20 --no-pager
sudo cat evidence/status.json
```

The scope route must use `eth0` with source `10.11.13.222`; Wi-Fi supplies
SSH and Internet access. The journal must report successful receiver startup.
The dedicated Ethernet profile reconnects automatically after boot; the
receiver service still requires a manual start. The `nmcli` command above
also brings the link up explicitly. An inactive scope link prevents receiver
setup and reports `APPLICATION_SETUP_FAILED`; check the route before retrying.

Single may already be armed when the service starts. Press the scope's Single
button manually if needed, then check `status.json` shows
`"armed": true` and `"error": null`. Then start the production node with all
sensors attached. The installed group accepts node `fb25c40751b35256`.

The fresh database was initialized with the operator-authorized one-use
known-empty-airtime request. Receiver startup consumes it atomically with the
first durable state. Subsequent starts preserve the database and its actual
airtime usage; do not delete, reinitialize, or add another commissioning token.

## One incident

1. On a DIO1 edge followed by the zero-IRQ `UNEXPECTED_IRQ` branch, the
   receiver snapshots context in RAM and raises GPIO25 before recovery.
2. A separate thread saves the stopped scope's PNG, all four complete binary
   waveforms, descriptors/settings, incident context, and SHA256 hashes under
   `evidence/captures/`. Each full capture uses about 80 MB plus its PNG.
3. Lower GPIO25 and continue.

Step 3 is automatic after saving. The scope remains stopped, retaining the
waveform. Wait until `status.json` shows `"capture_active": false`, then
manually press Single when you want the next capture. Software never rearms
or force-triggers the scope. Events while stopped or exporting retain detail
records without another waveform. Do not press Single during an export.

Capture failures retain partial evidence, lower GPIO25, and disable further
captures for that process. Inspect `status.json` and the incident record before
stopping/restarting the service. There is no automatic retry or service restart.

For status and stopping:

```sh
sudo systemctl status cura-radio-investigation.service --no-pager
sudo cat ~/radio-unexpected-irq/evidence/status.json
sudo systemctl stop cura-radio-investigation.service
```

Clean shutdown drives GPIO25 LOW and releases it as an input. Finish an
active export before stopping when possible; an interrupted export remains
incomplete evidence. Stop the node separately when ending the observation.

## Preserve and reproduce

The single investigation file owns GPIO25, bounded command/context retention,
manual-arming detection, Ethernet export, and hashes. Its docstring lists the
small temporary receiver calls needed to reproduce it on a compatible source.
Preserve that file plus the source identity and useful captures. The Pi's
`source/SOURCE_MANIFEST.json` seals every deployed runtime file, and
`deployment.json` records the module/configuration/helper digests and RTC bound.
The service checks source hashes before every start. Unsetting
`CURA_RADIO_INVESTIGATION_DIR` disables the instrumentation.

Raw `.raw.i16le` files contain signed 16-bit little-endian SCPI WORD samples,
with no file header. Descriptors and `.parameters.json` preserve scaling:
`V = code * vdiv_before_probe * probe_attenuation / code_per_div
- offset_before_probe * probe_attenuation`.
For this setup, `t = horizontal_delay_s - 0.05 + index * sample_interval_s`.

`scope-transport-check/` contains a re-export of the earlier stopped marker
smoke record, not a new unexpected-IRQ incident. Host tests and that transport
check do not establish the cause of an actual anomaly. MISO without SCK/NSS
does not independently establish SPI framing; no supply channel is measured.
