"""Shared local capture mechanics; test-layer policy stays with each caller."""
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import time

REPO = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def admit_episode(run, case, root, manual_record, ready_run):
    """Record readiness for this exact episode; no accounting or RF scheduling."""
    if ready_run != run:
        with open("/dev/tty", "r+") as terminal:
            terminal.write(f"Reserve this episode in both records, then type READY {case}: ")
            terminal.flush()
            if terminal.readline().strip() != f"READY {case}":
                raise RuntimeError("operator not ready")
    write_json(Path(root) / "operator-ready.json", dict(
        run=run, case=case, at_unix_ns=time.time_ns(), record_sha256=digest(manual_record)))


@contextmanager
def episode_capture(run, case, root, run_root):
    """Run every registered cleanup, retaining primary and cleanup failures."""
    cleanups = []
    primary = None
    try:
        yield cleanups
    except BaseException as error:
        primary = error
        run["status"] = "FAIL"
        failure = dict(case=case, error=type(error).__name__ + ": " + str(error))
        run["failures"].append(failure)
        write_json(Path(root) / "failure.json", failure)
        raise
    finally:
        errors = []
        first = None
        for name, action in reversed(cleanups):
            try:
                action()
            except BaseException as error:
                if first is None:
                    first = error
                failure = dict(case=case, cleanup=name,
                               error=type(error).__name__ + ": " + str(error))
                errors.append(failure)
                run["failures"].append(failure)
        if errors:
            run["status"] = "FAIL"
            write_json(Path(root) / "cleanup-failures.json", errors)
        write_json(Path(run_root) / "run.json", run)
        if primary is None and first is not None:
            raise first
