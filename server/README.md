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
order is defined by the payload schema in `../protocol/schemas/`. The full wire
format is documented in `../protocol/wire/v1.md`.

## Advertise with Avahi

For development:

```bash
avahi-publish -s "Cura Agrorum Gateway" _cura-agrorum._tcp 18032 proto=tcp-frame-v1 record_types=1,2
```

## Regenerate protocol files

The firmware C headers and Python decoder constants are generated from
`../protocol/schemas/reading_v1.json` and
`../protocol/schemas/node_config_v1.json`, and
`../protocol/schemas/handshake_ack_v1.json`.

```bash
cd ..
python3 protocol/tools/generate.py
```

For a Pi deployment, install:

```text
deploy/avahi/cura-agrorum.service -> /etc/avahi/services/cura-agrorum.service
deploy/systemd/cura-agrorum-server.service -> /etc/systemd/system/cura-agrorum-server.service
```

Then reload system services:

```bash
sudo systemctl restart avahi-daemon
sudo systemctl daemon-reload
sudo systemctl enable --now cura-agrorum-server
```
