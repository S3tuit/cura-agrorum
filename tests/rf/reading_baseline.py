"""Minimal read-only selected-node baseline for RF-020; no airtime mutation."""
import hashlib
import json
import sqlite3


def row_digest(row):
    value = {name: ({'bytes': v.hex()} if isinstance(v, bytes) else v)
             for name, v in dict(row).items()}
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def capture_readings(database, node_id):
    if len(node_id) != 8:
        raise ValueError('invalid node identity')
    with sqlite3.connect(database.resolve().as_uri() + '?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        db.execute('BEGIN')
        rows = db.execute('SELECT * FROM reading_messages WHERE node_id=? ORDER BY message_id', (node_id,))
        return dict(schema=1, node_id=node_id.hex(),
                    rows={str(r['message_id']): row_digest(r) for r in rows})


def new_readings(rows, baseline, node_id):
    if baseline.get('schema') != 1 or baseline.get('node_id') != node_id.hex():
        raise ValueError('reading baseline identity mismatch')
    before = baseline['rows']
    after = {str(r['message_id']): r for r in rows}
    if any(key not in after or row_digest(after[key]) != digest for key, digest in before.items()):
        raise ValueError('pre-existing reading changed or disappeared')
    return [r for key, r in after.items() if key not in before]
