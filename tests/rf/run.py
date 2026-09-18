"""Explicit fixture-specific launcher; no implicit cases or hardware fallback."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

from inputs import APP, REPO
from spec import select_cases, validate_fixture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture-state", choices=("nominal", "dio1_disconnected", "radio_absent"), required=True)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--cases", required=True)
    parser.add_argument("--run", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manual-record", type=Path, required=True)
    parser.add_argument("--host", default="cura-receiver")
    parser.add_argument("--host-key-alias")
    parser.add_argument("--peer-python", default="python3")
    parser.add_argument("--build", type=Path, default=APP / "build")
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--ready-run")
    parser.add_argument("--confirm-flash", action="store_true")
    parser.add_argument("--collect-only", action="store_true")
    args = parser.parse_args()
    fixture = validate_fixture(json.loads(args.fixture.read_text()))
    if fixture["c6_fixture"] != args.fixture_state:
        parser.error("entry point and fixture do not match")
    select_cases(args.cases, args.fixture_state)
    mac = ":".join(fixture["c6_dut"][i:i+2] for i in range(0, 12, 2))
    argv = [sys.executable, "-m", "pytest", "-c", str(REPO / "tests/rf/pytest.ini"),
            "-p", "pytest_embedded.plugin", str(REPO / "tests/rf/pytest_radio.py"), "-s", "-q",
            "--embedded-services=esp,idf", "--target=esp32c6", "--app-path=" + str(APP),
            "--build-dir=" + str(args.build.resolve()), "--port=" + fixture["c6_uart"], "--port-mac=" + mac,
            "--rf-cases=" + args.cases, "--rf-fixture=" + str(args.fixture.resolve()), "--rf-run=" + args.run,
            "--rf-output=" + str(args.output.resolve()), "--rf-manual-record=" + str(args.manual_record.resolve()),
            "--rf-host=" + args.host, "--root-logdir=" + str(args.output.resolve() / "uart"),
            "--rf-peer-python=" + args.peer_python,
            "--junitxml=" + str(args.output.resolve() / "junit.xml")]
    if args.confirm_flash:
        argv.append("--rf-confirm-flash")
    if args.ready_run:
        argv.append("--rf-ready-run=" + args.ready_run)
    argv.append("--rf-session=" + str(args.session.resolve()))
    if args.host_key_alias:
        argv.append("--rf-host-key-alias=" + args.host_key_alias)
    if args.collect_only:
        argv.append("--collect-only")
    return subprocess.run(argv, cwd=REPO, env=os.environ | {"PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"}).returncode


if __name__ == "__main__":
    raise SystemExit(main())
