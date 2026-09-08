import json
import subprocess
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "tools/capture_database_evidence.py"


def _capture(database, destination):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(database),
            str(destination),
            "--note",
            "CORRUPT; test revision",
        ],
        capture_output=True,
        text=True,
        timeout=10,
        check=True,
    )
    return json.loads(result.stdout)


# Raw damaged bytes are preserved without any SQLite interpretation or source mutation.
def test_capture_corrupt_file_set(tmp_path):
    database = tmp_path / "receiver.db"
    originals = {}
    for suffix in ("", "-wal", "-shm"):
        path = database.with_name(database.name + suffix)
        path.write_bytes(b"damaged evidence" + suffix.encode())
        originals[path] = (path.stat().st_ino, path.read_bytes())
    destination = tmp_path / "incident"
    report = _capture(database, destination)
    assert report["errors"] == []
    assert report["operator_note"] == "CORRUPT; test revision"
    assert [item["status"] for item in report["files"]] == ["copied"] * 3
    assert json.loads((destination / "capture.json").read_text()) == report
    for path, (inode, data) in originals.items():
        assert (path.stat().st_ino, path.read_bytes()) == (inode, data)
        assert (destination / path.name).read_bytes() == data
        assert (destination / path.name).stat().st_mode & 0o777 == 0o600


# An unreadable main source does not prevent trying sidecars; existing evidence is never overwritten.
def test_capture_errors_are_best_effort(tmp_path):
    database = tmp_path / "receiver.db"
    database.mkdir()
    wal = tmp_path / "receiver.db-wal"
    wal.write_bytes(b"WAL evidence")
    destination = tmp_path / "incident"
    report = _capture(database, destination)
    assert [item["status"] for item in report["files"]] == [
        "failed",
        "copied",
        "missing",
    ]
    assert (destination / wal.name).read_bytes() == wal.read_bytes()
    before = (destination / "capture.json").read_bytes()
    assert _capture(database, destination)["errors"]
    assert (destination / "capture.json").read_bytes() == before
    assert _capture(database, tmp_path / "receiver.db-shm")["errors"]
    assert not (tmp_path / "receiver.db-shm").exists()
