# Cura Agrorum Database

For v0.5 the database uses manual SQL migrations. Each file in
`migrations/` is versioned and should be applied once, in filename order.

Start local Postgres from `server/`:

```bash
docker compose up -d postgres
```

Apply the initial schema:

```bash
psql "postgresql://cura:cura_dev_password@localhost:55432/cura_agrorum" \
  -f db/migrations/001_init_schema.sql
```

The compose file has development defaults, so copying `.env.example` to
`.env` is optional unless you want to override credentials or the host port.
