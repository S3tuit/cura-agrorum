# Cura Agrorum Protocol Schemas

`schemas/reading_v1.json` and `schemas/ack_v1.json` are the sources of truth for
the current wire payloads. The firmware node UUID is generated separately into
`firmware/main/node_identity.h` and accepted nodes are registered in the server
database.

Regenerate the firmware and server schema files with:

```bash
python3 protocol/tools/generate.py
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
python3 protocol/tools/generate.py --check
```
