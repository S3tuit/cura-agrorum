# Cura Agrorum Server

Small TCP gateway for ESP32 sensor nodes.

## Run locally

From the repository root, start Postgres, apply the manual schema migration,
install the server package, then run the TCP server:

```bash
cd server
docker compose up -d postgres
psql "postgresql://cura:cura_dev_password@localhost:55432/cura_agrorum" \
  -f db/migrations/001_init_schema.sql
python -m pip install -e .
python -m cura_server --host 0.0.0.0 --port 18032
```

Use `--database-url` or `DATABASE_URL` to override the local development
connection string.

At startup, the server loads accepted node UUIDs from active rows in
`node_configuration` where `valid_until IS NULL`. Restart the server after adding
or changing an accepted node. Readings from UUIDs outside this startup-loaded set
are rejected with a nonzero ACK.

The server listens for frames with this format:

```text
u32 body_len
u16 envelope_version
u16 event_count

repeated event_count times:
  u8  record_type
  u8  schema_version
  u16 payload_len
  u8  payload[payload_len]
```

Frame, envelope, and event header fields use network byte order. Payload byte
order is defined by the payload schema in
`../protocol/wifi-protocol-v1/schemas/`. The full wire format is documented in
`../protocol/wifi-protocol-v1/wire/v1.md`.

After durably persisting every event in a frame, the server returns an `ack_t`
event with status `0`. A nonzero status rejects the complete frame. Nodes keep
one frame outstanding at a time and may reuse the TCP connection for subsequent
frames after a successful ACK.

## Field Hotspot

For field collection from the Fedora laptop, start the NetworkManager hotspot:

```bash
sudo deploy/field-hotspot.sh up
```

The current firmware connects to SSID `cura-field` and sends TCP frames to
`10.42.0.1:18032`. No service discovery is used in this prototype.

## Regenerate protocol files

The firmware C headers and Python decoder constants are generated from the JSON
schemas in `../protocol/wifi-protocol-v1/schemas/`.

```bash
cd ..
python3 protocol/wifi-protocol-v1/tools/generate.py
```
