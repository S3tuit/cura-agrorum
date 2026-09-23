"""Operator-only, best-effort raw capture after stopping all database users."""

from __future__ import annotations

import argparse
import json
import os
import shutil
from datetime import datetime, timezone
from pathlib import Path


def capture(database: Path, destination: Path, *, note: str) -> dict:
    """Copy evidence without SQLite access or changing the source file set.

    A failed copy may leave partial evidence. Report each failure and continue;
    the operator decides whether to restore, regardless of capture success.
    """
    report = {
        "database": str(database.absolute()),
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "operator_note": note,
        "files": [],
        "errors": [],
    }
    try:
        database = database.resolve()
        sources = {
            database.with_name(database.name + suffix)
            for suffix in ("", "-wal", "-shm")
        }
        if destination.resolve() in sources:
            raise ValueError(
                "evidence destination must be outside the original file set"
            )
        destination.mkdir(mode=0o700)
    except Exception as error:
        report["errors"].append(f"create evidence directory: {error}")
        return report
    for suffix in ("", "-wal", "-shm"):
        source = database.with_name(database.name + suffix)
        item = {"name": source.name}
        report["files"].append(item)
        try:
            with source.open("rb") as incoming:
                target = destination / source.name
                descriptor = os.open(
                    target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600
                )
                with os.fdopen(descriptor, "wb") as outgoing:
                    shutil.copyfileobj(incoming, outgoing)
                    outgoing.flush()
                    os.fsync(outgoing.fileno())
                    item["bytes"] = outgoing.tell()
            item["status"] = "copied"
        except FileNotFoundError as error:
            item["status"] = "missing"
            item["error"] = str(error)
        except Exception as error:
            item["status"] = "failed"
            item["error"] = str(error)
    try:
        with (destination / "capture.json").open("x", encoding="utf-8") as output:
            json.dump(report, output, indent=2)
            output.write("\n")
    except Exception as error:
        report["errors"].append(f"write capture report: {error}")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database", type=Path)
    parser.add_argument("destination", type=Path, help="new incident directory")
    parser.add_argument(
        "--note", required=True, help="original failure and source revision"
    )
    arguments = parser.parse_args()
    report = capture(arguments.database, arguments.destination, note=arguments.note)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
