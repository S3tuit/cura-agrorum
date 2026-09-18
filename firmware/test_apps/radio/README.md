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
initialize/write NVS, PHY storage or LittleFS and has no credentials. The joint
runner must verify these actual flash offsets/file lengths, the matching table
and source/build seal before requesting pytest-embedded's DUT fixture. Full
erase and arbitrary flash images are rejected. Never restore erased counters
and reuse an old identity/key; destructive identity changes remain a separate
provisioning operation.

Use the [joint procedures](../../../../tests/rf/README.md) for fixture selection,
operator admission, peer staging and result verification. Do not select a fault
case on nominal wiring. RF-012/013 require operator changes with power removed
and a separately retained nominal restoration result.
