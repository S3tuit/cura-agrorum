from __future__ import annotations

import importlib.util
import json
import stat
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest


def _load_provisioning(repo_root: Path) -> ModuleType:
    path = (
        repo_root
        / "protocol"
        / "protocol-v2-lora"
        / "tools"
        / "provisioning_common.py"
    )
    spec = importlib.util.spec_from_file_location(
        "protocol_v2_lora_provisioning_common", path
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def provisioning(repo_root: Path) -> ModuleType:
    return _load_provisioning(repo_root)


def _run_tool(
    repo_root: Path,
    name: str,
    *arguments: object,
) -> subprocess.CompletedProcess[str]:
    script = (
        repo_root
        / "protocol"
        / "protocol-v2-lora"
        / "tools"
        / name
    )
    return subprocess.run(
        [sys.executable, str(script), *(str(value) for value in arguments)],
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=False,
    )


def _initialize(
    repo_root: Path, recv_dir: Path
) -> subprocess.CompletedProcess[str]:
    recv_dir.mkdir()
    result = _run_tool(
        repo_root,
        "init_receiver_group.py",
        "--recv-dir",
        recv_dir,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return result


def _read_group(recv_dir: Path) -> dict[str, object]:
    return json.loads(
        (recv_dir / "receiver-group.json").read_text(encoding="utf-8")
    )


def _provision(
    repo_root: Path,
    recv_dir: Path,
    node_dir: Path,
    *arguments: str,
) -> subprocess.CompletedProcess[str]:
    return _run_tool(
        repo_root,
        "provision_node.py",
        "--recv-dir",
        recv_dir,
        "--node-dir",
        node_dir,
        *arguments,
    )


def test_hkdf_matches_public_protocol_vector(
    provisioning: ModuleType,
) -> None:
    master_key = bytes.fromhex(
        "000102030405060708090a0b0c0d0e0f"
        "101112131415161718191a1b1c1d1e1f"
    )
    node_id = bytes.fromhex("0102030405060708")

    assert provisioning.derive_node_key(master_key, node_id).hex() == (
        "c0f9a1a0f386692e01028082be92330e"
    )


def test_node_id_generation_retries_zero_active_and_retired_collisions(
    provisioning: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    active = bytes.fromhex("0102030405060708")
    retired = bytes.fromhex("1112131415161718")
    fresh = bytes.fromhex("2122232425262728")
    candidates = iter((bytes(8), active, retired, fresh))

    def deterministic_token_bytes(size: int) -> bytes:
        assert size == 8
        return next(candidates)

    monkeypatch.setattr(
        provisioning.secrets,
        "token_bytes",
        deterministic_token_bytes,
    )
    state = provisioning.ReceiverGroupState(
        group_id=bytes.fromhex("3132333435363738"),
        group_master_key=bytes(range(32)),
        active_node_ids=frozenset((active,)),
        retired_node_ids=frozenset((retired,)),
    )

    assert provisioning.generate_unique_node_id(state) == fresh


def test_initialize_is_exclusive_and_does_not_print_secret(
    repo_root: Path, tmp_path: Path
) -> None:
    recv_dir = tmp_path / "receiver"
    result = _initialize(repo_root, recv_dir)
    state_path = recv_dir / "receiver-group.json"
    state = _read_group(recv_dir)

    assert set(state) == {
        "format_version",
        "group_id",
        "group_master_key",
        "active_node_ids",
        "retired_node_ids",
    }
    assert state["format_version"] == 1
    assert len(bytes.fromhex(str(state["group_id"]))) == 8
    assert len(bytes.fromhex(str(state["group_master_key"]))) == 32
    assert state["active_node_ids"] == []
    assert state["retired_node_ids"] == []
    assert stat.S_IMODE(state_path.stat().st_mode) == 0o600
    assert str(state["group_master_key"]) not in result.stdout

    original = state_path.read_bytes()
    repeated = _run_tool(
        repo_root,
        "init_receiver_group.py",
        "--recv-dir",
        recv_dir,
    )
    assert repeated.returncode != 0
    assert state_path.read_bytes() == original


def test_master_rotation_requires_both_flags_and_retires_active_ids(
    repo_root: Path, tmp_path: Path
) -> None:
    recv_dir = tmp_path / "receiver"
    node_dir = tmp_path / "node"
    node_dir.mkdir()
    _initialize(repo_root, recv_dir)
    provisioned = _provision(repo_root, recv_dir, node_dir)
    assert provisioned.returncode == 0, provisioned.stdout + provisioned.stderr

    before = _read_group(recv_dir)
    state_path = recv_dir / "receiver-group.json"
    original = state_path.read_bytes()
    for incomplete_flags in (
        ("--rotate-master-key",),
        ("--acknowledge-all-nodes-require-reprovisioning",),
    ):
        result = _run_tool(
            repo_root,
            "init_receiver_group.py",
            "--recv-dir",
            recv_dir,
            *incomplete_flags,
        )
        assert result.returncode != 0
        assert state_path.read_bytes() == original

    rotated = _run_tool(
        repo_root,
        "init_receiver_group.py",
        "--recv-dir",
        recv_dir,
        "--rotate-master-key",
        "--acknowledge-all-nodes-require-reprovisioning",
    )
    assert rotated.returncode == 0, rotated.stdout + rotated.stderr
    after = _read_group(recv_dir)
    assert after["group_id"] != before["group_id"]
    assert after["group_master_key"] != before["group_master_key"]
    assert after["active_node_ids"] == []
    assert after["retired_node_ids"] == before["active_node_ids"]


def test_provisioning_creates_matching_private_header_and_active_id(
    repo_root: Path,
    tmp_path: Path,
    provisioning: ModuleType,
) -> None:
    recv_dir = tmp_path / "receiver"
    node_dir = tmp_path / "node"
    node_dir.mkdir()
    _initialize(repo_root, recv_dir)

    result = _provision(repo_root, recv_dir, node_dir)
    assert result.returncode == 0, result.stdout + result.stderr

    state = _read_group(recv_dir)
    assert len(state["active_node_ids"]) == 1
    node_id_hex = state["active_node_ids"][0]
    node_id = bytes.fromhex(node_id_hex)
    node_key = provisioning.derive_node_key(
        bytes.fromhex(state["group_master_key"]),
        node_id,
    )
    identity_path = node_dir / "protocol_v2_lora_identity.h"
    identity = identity_path.read_text(encoding="utf-8")

    assert f'CURA_LORA_V2_NODE_ID_HEX "{node_id_hex}"' in identity
    for byte in node_key:
        assert f"UINT8_C(0x{byte:02x})" in identity
    assert node_key.hex() not in result.stdout
    assert stat.S_IMODE(identity_path.stat().st_mode) == 0o600


def test_existing_identity_requires_explicit_operation(
    repo_root: Path, tmp_path: Path
) -> None:
    recv_dir = tmp_path / "receiver"
    node_dir = tmp_path / "node"
    node_dir.mkdir()
    _initialize(repo_root, recv_dir)
    first = _provision(repo_root, recv_dir, node_dir)
    assert first.returncode == 0, first.stdout + first.stderr

    identity_path = node_dir / "protocol_v2_lora_identity.h"
    first_identity = identity_path.read_bytes()
    first_state = _read_group(recv_dir)

    refused = _provision(repo_root, recv_dir, node_dir)
    assert refused.returncode != 0
    assert identity_path.read_bytes() == first_identity
    assert _read_group(recv_dir) == first_state


def test_replace_staged_identity_changes_header_without_retiring_old_id(
    repo_root: Path, tmp_path: Path
) -> None:
    recv_dir = tmp_path / "receiver"
    node_dir = tmp_path / "node"
    node_dir.mkdir()
    _initialize(repo_root, recv_dir)
    first = _provision(repo_root, recv_dir, node_dir)
    assert first.returncode == 0, first.stdout + first.stderr

    identity_path = node_dir / "protocol_v2_lora_identity.h"
    previous_header = identity_path.read_bytes()
    previous_state = _read_group(recv_dir)
    previous_id = previous_state["active_node_ids"][0]

    replaced = _provision(
        repo_root,
        recv_dir,
        node_dir,
        "--replace-staged-identity",
    )
    assert replaced.returncode == 0, replaced.stdout + replaced.stderr
    replaced_state = _read_group(recv_dir)
    replaced_header = identity_path.read_bytes()

    assert replaced_header != previous_header
    assert len(replaced_state["active_node_ids"]) == 2
    assert previous_id in replaced_state["active_node_ids"]
    assert previous_id not in replaced_state["retired_node_ids"]
    replacement_id = next(
        node_id
        for node_id in replaced_state["active_node_ids"]
        if node_id != previous_id
    )
    replacement_text = replaced_header.decode("utf-8")
    assert replacement_id in replacement_text
    assert previous_id not in replacement_text


def test_rotate_node_identity_changes_header_and_retires_old_id(
    repo_root: Path, tmp_path: Path
) -> None:
    recv_dir = tmp_path / "receiver"
    node_dir = tmp_path / "node"
    node_dir.mkdir()
    _initialize(repo_root, recv_dir)
    first = _provision(repo_root, recv_dir, node_dir)
    assert first.returncode == 0, first.stdout + first.stderr

    identity_path = node_dir / "protocol_v2_lora_identity.h"
    previous_header = identity_path.read_bytes()
    previous_state = _read_group(recv_dir)
    previous_id = previous_state["active_node_ids"][0]

    rotated = _provision(
        repo_root,
        recv_dir,
        node_dir,
        "--rotate-node-identity",
    )
    assert rotated.returncode == 0, rotated.stdout + rotated.stderr
    rotated_state = _read_group(recv_dir)
    rotated_header = identity_path.read_bytes()

    assert rotated_header != previous_header
    assert previous_id not in rotated_state["active_node_ids"]
    assert previous_id in rotated_state["retired_node_ids"]
    assert len(rotated_state["active_node_ids"]) == 1
    replacement_id = rotated_state["active_node_ids"][0]
    replacement_text = rotated_header.decode("utf-8")
    assert replacement_id in replacement_text
    assert previous_id not in replacement_text


def test_unsafe_receiver_group_permissions_are_rejected(
    repo_root: Path, tmp_path: Path
) -> None:
    recv_dir = tmp_path / "receiver"
    node_dir = tmp_path / "node"
    node_dir.mkdir()
    _initialize(repo_root, recv_dir)
    state_path = recv_dir / "receiver-group.json"
    original = state_path.read_bytes()
    state_path.chmod(0o640)

    result = _provision(repo_root, recv_dir, node_dir)

    assert result.returncode == 2
    assert "accessible by group or others" in result.stderr
    assert state_path.read_bytes() == original
    assert not (node_dir / "protocol_v2_lora_identity.h").exists()


def test_unsafe_existing_node_identity_permissions_are_rejected(
    repo_root: Path, tmp_path: Path
) -> None:
    recv_dir = tmp_path / "receiver"
    node_dir = tmp_path / "node"
    node_dir.mkdir()
    _initialize(repo_root, recv_dir)
    first = _provision(repo_root, recv_dir, node_dir)
    assert first.returncode == 0, first.stdout + first.stderr

    identity_path = node_dir / "protocol_v2_lora_identity.h"
    state_before = (recv_dir / "receiver-group.json").read_bytes()
    identity_before = identity_path.read_bytes()
    identity_path.chmod(0o644)

    result = _provision(
        repo_root,
        recv_dir,
        node_dir,
        "--replace-staged-identity",
    )

    assert result.returncode == 2
    assert "accessible by group or others" in result.stderr
    assert (recv_dir / "receiver-group.json").read_bytes() == state_before
    assert identity_path.read_bytes() == identity_before


@pytest.mark.parametrize(
    ("case", "expected_error"),
    (
        ("invalid-syntax", "cannot read receiver group state"),
        ("missing-field", "receiver group state fields differ"),
        ("unknown-field", "receiver group state fields differ"),
        (
            "invalid-master-key",
            "group_master_key must contain exactly 64 lowercase hex digits",
        ),
        (
            "invalid-node-id",
            "active_node_ids[0] must contain exactly 16 lowercase hex digits",
        ),
        (
            "overlapping-node-id",
            "active_node_ids and retired_node_ids overlap",
        ),
    ),
)
def test_malformed_receiver_group_is_rejected_without_changes(
    repo_root: Path,
    tmp_path: Path,
    case: str,
    expected_error: str,
) -> None:
    recv_dir = tmp_path / "receiver"
    node_dir = tmp_path / "node"
    node_dir.mkdir()
    _initialize(repo_root, recv_dir)
    state_path = recv_dir / "receiver-group.json"
    state = _read_group(recv_dir)

    if case == "invalid-syntax":
        malformed = "{not valid JSON\n"
    else:
        if case == "missing-field":
            del state["group_id"]
        elif case == "unknown-field":
            state["unexpected"] = True
        elif case == "invalid-master-key":
            state["group_master_key"] = "00"
        elif case == "invalid-node-id":
            state["active_node_ids"] = ["01"]
        elif case == "overlapping-node-id":
            node_id = "0102030405060708"
            state["active_node_ids"] = [node_id]
            state["retired_node_ids"] = [node_id]
        else:
            raise AssertionError(f"unhandled malformed-state case: {case}")
        malformed = json.dumps(state, indent=2) + "\n"
    state_path.write_text(malformed, encoding="utf-8")
    original = state_path.read_bytes()

    result = _provision(repo_root, recv_dir, node_dir)

    assert result.returncode == 2
    assert expected_error in result.stderr
    assert state_path.read_bytes() == original
    assert not (node_dir / "protocol_v2_lora_identity.h").exists()
