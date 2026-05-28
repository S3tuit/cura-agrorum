# Cura Agrorum Protocol Schemas

`schemas/reading_v1.json` is the source of truth for the current ESP32 reading
wire payload. `schemas/node_config_v1.json` is the source of truth for the node
configuration payload sent before readings on a TCP connection.
`schemas/config_ack_v1.json` is the source of truth for the server ACK sent
after accepting a node configuration batch.

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
