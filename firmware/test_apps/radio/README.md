# C6 raw radio component app

Build with the configured ESP-IDF environment:

```sh
CCACHE_DISABLE=1 idf.py -C firmware/test_apps/radio build
```

The app uses production radio/backend/platform code and Unity assertions. UART0
at 115200 is the control/evidence connection. Fresh boots print `RF_BOOT` and
`RF_READY`; no boot automatically transmits. Only a supported run/case command
bound to the reported random boot nonce executes. After one command, even a
Unity failure, the app attempts radio cleanup and enters real two-second timer
deep sleep. A subsequent boot waits again. RF-010 continuation additionally
checks retained run/case identity and DEEPSLEEP reset; it is never autonomous.

The app-local line accumulator tolerates fragmented nonblocking UART reads.
Commands end with LF (CRLF is accepted), contain at most 159 bytes before LF,
and must complete within two seconds after their first byte. Idle readiness is
unlimited. Invalid input emits structured `RF_REJECT` and latches until reset,
without touching the radio. The host fails without resending. `RF_COMMAND`
retains each accepted line's byte count and receipt duration for timing review.

The app buffers backend observations and radio results in RAM until after the
timed operations. These forwarding wrappers add small observation overhead;
they do not replace radio results, clocks or production policy. The checks do
not prove electrical-edge accuracy or authenticated receiver acceptance.

Flashing replaces the existing factory app. `partitions.csv` matches the
production physical layout: NVS `[0x9000,0xf000)`, PHY `[0xf000,0x10000)`,
factory app `[0x10000,0x110000)`, LittleFS `[0x110000,0x3f0000)`. Normal build
flash files contain only bootloader, partition table and app. This app does not
initialize/write NVS, PHY storage or LittleFS. Default builds have no credentials;
optional RF-023 builds contain private disposable credentials as described below. The joint
runner must verify these actual flash offsets/file lengths, the matching table
and source/build seal before requesting pytest-embedded's DUT fixture. Full
erase and arbitrary flash images are rejected. Never restore erased counters
and reuse an old identity/key; destructive identity changes remain a separate
provisioning operation.

Use the [joint procedures](../../../../tests/rf/README.md) for fixture selection,
operator admission, peer staging and result verification. Do not select a fault
case on nominal wiring. RF-012/013 require operator changes with power removed
and a separately retained nominal restoration result.

## Optional RF-023 packet cases

The [RF-023 preparation procedure](../../../../tests/rf/RF023_PREPARATION.md)
creates a fresh run-bound private input directory. Configure `RF023_INPUT_DIR`
to its absolute path to enable the twelve named cases. Ordinary builds reject
these cases before any radio operation. Enabled builds also require the exact
prepared run ID, phase zero and a fresh boot nonce in the existing RUN command.
The manifest maps each vector to its short command alias.

Each command builds one fixed frame using production protocol crypto, transmits
once and observes the exact expected ACK or silence until 500 ms after TX_DONE.
No command accepts arbitrary packet bytes. Credentials and counters are compiled
into this disposable image; NVS and LittleFS remain untouched. A source/header
hash guard rejects changed packet construction with an old input bundle. Prepare
fresh inputs after such changes. Enabled binaries and headers are private.

Local host comparisons and builds do not qualify RF-023. Installed-service
orchestration, revocation restart and physical ACK/silence evidence remain deferred.
