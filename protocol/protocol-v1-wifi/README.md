# Cura Agrorum Protocol Schemas

The JSON files in `schemas/` are the sources of truth for the current wire
payloads. The firmware node UUID is generated separately into
`firmware/main/node_identity.h` and accepted nodes are registered in the server
database.

Schema fields with type `enum` are signed 32-bit integers on the wire. Their
values are generated as fixed-width C constants and a Python `IntEnum`. Decoded
Python payloads retain raw integers so future unknown enum values remain
readable.

Regenerate the firmware and server schema files with:

```bash
python3 protocol/wifi-protocol-v1/tools/generate.py
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
python3 protocol/wifi-protocol-v1/tools/generate.py --check
```
