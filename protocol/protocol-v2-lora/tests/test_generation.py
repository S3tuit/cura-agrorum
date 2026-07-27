from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def test_generated_files_are_current(repo_root: Path) -> None:
    generator = (
        repo_root
        / "protocol"
        / "protocol-v2-lora"
        / "tools"
        / "generate.py"
    )
    result = subprocess.run(
        [sys.executable, str(generator), "--check"],
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
