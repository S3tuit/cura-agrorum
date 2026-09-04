from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest


def _run_validation(
    repo_root: Path, schema_path: Path
) -> subprocess.CompletedProcess[str]:
    generator = (
        repo_root
        / "protocol"
        / "protocol-v2-lora"
        / "tools"
        / "generate.py"
    )
    return subprocess.run(
        [
            sys.executable,
            str(generator),
            "--schema",
            str(schema_path),
            "--validate-only",
        ],
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=False,
    )


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


@pytest.mark.parametrize(
    ("mutation", "expected_error"),
    (
        ("short_nonce", "ccm.nonce_size_bytes must be 13, got 12"),
        ("control_nonce_suffix", "ccm.nonce fields"),
        ("short_tag", "ccm.tag_size_bytes must be 8, got 4"),
        (
            "short_reading_frame",
            "frames.reading.sx1262_payload_size_bytes must be 54, got 50",
        ),
        (
            "short_ack_frame",
            "frames.ack.sx1262_payload_size_bytes must be 23, got 19",
        ),
    ),
)
def test_schema_validation_rejects_non_v2_crypto_and_frame_contracts(
    repo_root: Path,
    tmp_path: Path,
    mutation: str,
    expected_error: str,
) -> None:
    schema_path = (
        repo_root
        / "protocol"
        / "protocol-v2-lora"
        / "schemas"
        / "protocol_v2_lora.json"
    )
    schema = json.loads(schema_path.read_text(encoding="utf-8"))

    if mutation == "short_nonce":
        schema["ccm"]["nonce_size_bytes"] = 12
        schema["ccm"]["nonce"]["size_bytes"] = 12
        schema["ccm"]["nonce"]["fields"].pop()
    elif mutation == "control_nonce_suffix":
        schema["ccm"]["nonce"]["fields"][-1]["name"] = "control"
    elif mutation == "short_tag":
        schema["ccm"]["tag_size_bytes"] = 4
        for frame in schema["frames"].values():
            frame["ccm_tag_size_bytes"] = 4
            frame["sx1262_payload_size_bytes"] -= 4
    elif mutation == "short_reading_frame":
        schema["frames"]["reading"]["sx1262_payload_size_bytes"] = 50
    elif mutation == "short_ack_frame":
        schema["frames"]["ack"]["sx1262_payload_size_bytes"] = 19
    else:
        pytest.fail(f"unknown schema mutation: {mutation}")

    mutated_path = tmp_path / "protocol_v2_lora.json"
    mutated_path.write_text(json.dumps(schema), encoding="utf-8")
    result = _run_validation(repo_root, mutated_path)

    assert result.returncode == 2
    assert expected_error in result.stderr
