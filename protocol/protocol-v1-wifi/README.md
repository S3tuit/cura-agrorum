# Cura Agrorum Protocol Schemas

These schemas describe the historical Wi-Fi v1 protocol, not the current LoRa v2 pilot.
The legacy server has been removed. The JSON files in `schemas/` retain the v1
wire payload definitions. The firmware node UUID is generated separately into
`firmware/main/node_identity.h`.

Schema fields with type `enum` are signed 32-bit integers on the wire. Their
values are generated as fixed-width C constants and a Python `IntEnum`. Decoded
Python payloads retain raw integers so future unknown enum values remain
readable.

Regenerate legacy firmware schema files with:

```bash
python3 protocol/protocol-v1-wifi/tools/generate.py
```

The generator also creates ignored firmware-local identity files:

```text
firmware/main/node_uuid.txt
firmware/main/node_identity.h
```

`node_uuid.txt` is the stable UUID for this physical node. Keep it with that
node and do not commit it.

Check whether generated files are stale with:

```bash
python3 protocol/protocol-v1-wifi/tools/generate.py --check
```

Python decoder output is optional: pass `--python-output-dir /tmp/cura-v1-decoders`
when needed for historical data. The default command does not recreate `server/`.
These legacy identity files are unrelated to LoRa v2 provisioning.
