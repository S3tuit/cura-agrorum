"""Destructive-controller interlocks are checked before credentials or SSH access."""

import os
from pathlib import Path
import subprocess
import sys

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "tools/test_airtime_reboot.py"


# Missing explicit confirmation and non-dedicated/traversing roots fail before reading credentials or contacting a target.
@pytest.mark.parametrize(
    "root,confirm",
    [
        ("/var/tmp/cura-airtime-test", False),
        ("/", True),
        ("relative", True),
        ("/var/tmp/cura-airtime-x/../../etc", True),
        ("/var/tmp/cura-airtime-..", True),
    ],
)
def test_reboot_controller_rejects_unsafe_invocation(tmp_path, root, confirm):
    output = tmp_path / "evidence"
    argv = [
        sys.executable,
        str(SCRIPT),
        "--host",
        "unused.invalid",
        "--remote-source",
        "/unused",
        "--remote-test-root",
        root,
        "--python",
        "/unused/python",
        "--output",
        str(output),
    ]
    if confirm:
        argv.append("--confirm-receiver-destructive")
    environment = {k: v for k, v in os.environ.items() if k != "CURA_PI_PASSWORD"}
    result = subprocess.run(
        argv, capture_output=True, text=True, timeout=5, env=environment
    )
    assert result.returncode == 2
    assert not output.exists()
    assert "KeyError" not in result.stderr and "Traceback" not in result.stderr
