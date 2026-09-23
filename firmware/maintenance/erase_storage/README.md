# erase_storage

Destructive maintenance firmware for clearing a node's pending-reading storage
without invalidating its identity-lifetime counters.

When flashed and run, it:

- Logs an `ESP_LOGE` warning.
- Waits 5 seconds.
- Formats and mounts the `storage` LittleFS partition.
- Preserves the default `nvs` partition, including `next_sample_id` and
  `next_message_id`.
- Logs success or failure.
- Enters deep sleep with no wake source configured.

Do not erase or restore NVS while retaining the provisioned node identity and
key. The pilot cannot distinguish fresh provisioning from counter loss or
rollback, so doing so can reuse a CCM nonce. If NVS has been erased, corrupted,
or restored from an older image, rotate both the node identity and key before
the node transmits again. A protocol recovery handshake is deferred; see the
protocol and architecture documentation.

This application is not the erase step for identity rotation because it
deliberately preserves NVS. For an operator-controlled rotation, stop production
firmware, generate the new identity, erase the complete flash and cold-power-cycle
the node to clear retained RTC state. Then flash and run this nontransmitting
maintenance application to initialize an empty LittleFS filesystem. Require its
successful format/mount result before flashing production firmware containing
the new identity. That final flash must preserve the initialized `storage`
partition: do not erase all flash again or include a storage image in the
production flash operation. For example, the initial destructive flash step is:

```bash
source ~/esp/esp-idf/export.sh
idf.py -p PORT erase-flash
```

Do not let the old production firmware run after that erase, and do not let the
new identity transmit before the complete node-local erase has finished. The
rotation deliberately discards the node's pending, quarantine, diagnostic and
delivery logs; it does not erase receiver-side historical records.

Formatting creates the empty filesystem directly in flash; no separate image
generator is required. Verify that this helper and the production build use
the same actual storage partition offset/size and compatible LittleFS settings.
The production backend intentionally does not format on mount failure, so
flashing only production code after a full erase is insufficient. For RF tests,
retain the successful formatter result and verify the initialized storage with
the read-only capture procedure in `../../TESTING.md`. Keep the production
image stopped until the operator's fixture and airtime readiness are confirmed.

Build and flash from this directory:

```bash
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p PORT flash monitor
```

This app reuses `../../partitions.csv`, so it targets the same storage layout as
the production firmware.
