"""One-off DEP-021 check on isolated, already-qualified synthetic Pi data."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import sys

from cura_receiver.application_settings import ApplicationSettings
from cura_receiver.sqlite_database import open_receiver_database
from tests.support.builders.persistence import GROUP
from service_probe import snapshot


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def records(path):
    with sqlite3.connect(path.as_uri() + '?mode=ro', uri=True) as db:
        assert db.execute('PRAGMA integrity_check').fetchall() == [('ok',)]
        assert db.execute('PRAGMA foreign_key_check').fetchall() == []
        tables = {}
        for (name,) in db.execute("SELECT name FROM sqlite_master WHERE type='table' ORDER BY name"):
            rows = db.execute('SELECT * FROM "' + name.replace('"', '""') + '"').fetchall()
            encoded = sorted(json.dumps(row, default=lambda b: {'hex': b.hex()}, separators=(',', ':')) for row in rows)
            tables[name] = dict(rows=len(rows), sha256=hashlib.sha256('\n'.join(encoded).encode()).hexdigest())
        return tables


root = Path(sys.argv[1]).resolve(strict=True)
assert os.geteuid() != 0
workloads = sorted(p for p in (root / 'storage-safe').glob('*/workload.json')
                   if not p.parent.is_symlink())
assert len(workloads) == 3
output = root / 'local-retention'
output.mkdir(mode=0o700)
results = []
for index, workload in enumerate(workloads):
    path = workload.parent / 'receiver.db'
    before = digest(path)
    original = records(path)
    assert original['reading_messages']['rows'] == 24
    assert original['message_profiles']['rows'] == 48
    for name in ('clock_observations', 'diagnostics', 'receiver_health'):
        assert original[name]['rows'] == 24
    destination = output / f'snapshot-{index}.sqlite3'
    captured = snapshot(path, destination)
    assert records(destination) == original
    restored = output / f'restored-{index}.sqlite3'
    shutil.copyfile(destination, restored)
    opened = open_receiver_database(restored, GROUP, minimum_free_bytes=0)
    assert opened.database is not None, opened
    opened.database.close()
    assert records(restored) == original
    assert digest(path) == before
    assert digest(destination) == captured['sha256']
    results.append(dict(batch_size=json.loads(workload.read_text())['batch_size'],
                        tables=original, snapshot=captured, restored_validation='PASS',
                        original_unchanged=True))
space = os.statvfs(root)
threshold = ApplicationSettings().minimum_free_bytes
available = space.f_bavail * space.f_frsize
assert available > threshold
print(json.dumps(dict(uid=os.getuid(), gid=os.getgid(), results=results,
                      available_bytes=available, minimum_free_bytes=threshold,
                      sqlite=sqlite3.sqlite_version, snapshot_directory=str(output))))
