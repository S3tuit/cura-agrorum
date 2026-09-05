from __future__ import annotations

import errno
import json
import os
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

from cura_protocol_v2_lora import receiver_group
from cura_protocol_v2_lora.receiver_group import (
    ReceiverGroupRejectedError,
    ReceiverGroupRejection as Rejection,
    load_receiver_group,
)


def _document() -> dict[str, object]:
    # Public protocol vector; never generated from a live deployment.
    return {
        "format_version": 1,
        "group_id": "0102030405060708",
        "group_master_key": "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f",
        "active_node_ids": ["1122334455667788"],
        "retired_node_ids": ["2233445566778899"],
    }


def _write(path: Path, value: object) -> Path:
    path.write_text(json.dumps(value), encoding="utf-8")
    path.chmod(0o600)
    return path


# The shared loader returns an exact immutable value whose representation hides the key.
def test_load_private_group_is_immutable_and_secret_safe(tmp_path: Path) -> None:
    path = _write(tmp_path / "group.json", _document())
    original = path.read_bytes()
    state = load_receiver_group(path)
    assert state.group_id == bytes.fromhex("0102030405060708")
    assert state.group_master_key == bytes(range(32))
    assert state.active_node_ids == frozenset({bytes.fromhex("1122334455667788")})
    assert state.retired_node_ids == frozenset({bytes.fromhex("2233445566778899")})
    assert "group_master_key" not in repr(state)
    assert repr(bytes(range(32))) not in repr(state)
    with pytest.raises(FrozenInstanceError):
        state.group_id = bytes(8)
    assert path.read_bytes() == original


# Each content rejection has a stable reason without retaining the supplied value in text.
@pytest.mark.parametrize(
    ("field", "value", "reason"),
    (
        ("format_version", True, Rejection.UNSUPPORTED_VERSION),
        ("format_version", 2, Rejection.UNSUPPORTED_VERSION),
        ("group_id", "00" * 8, Rejection.INVALID_GROUP_ID),
        ("group_id", "AB" * 8, Rejection.INVALID_GROUP_ID),
        ("group_master_key", "sensitive-invalid-key", Rejection.INVALID_MASTER_KEY),
        ("active_node_ids", "1122334455667788", Rejection.INVALID_NODE_IDS),
        ("active_node_ids", ["00" * 8], Rejection.INVALID_NODE_IDS),
        ("active_node_ids", ["11" * 8, "11" * 8], Rejection.INVALID_NODE_IDS),
        ("retired_node_ids", ["1122334455667788"], Rejection.OVERLAPPING_NODE_IDS),
        ("unknown-field", True, Rejection.INVALID_DOCUMENT),
    ),
)
def test_content_rejection_reason(
    tmp_path: Path, field: str, value: object, reason: Rejection
) -> None:
    document = _document()
    document[field] = value
    path = _write(tmp_path / "group.json", document)
    with pytest.raises(ReceiverGroupRejectedError) as captured:
        load_receiver_group(path)
    assert captured.value.reason is reason
    assert "sensitive-invalid-key" not in str(captured.value)


# Invalid byte/JSON/root/field encodings share the closed document reason.
@pytest.mark.parametrize("raw", (b"\xff", b"{", b"[]", b"{}"))
def test_invalid_document(tmp_path: Path, raw: bytes) -> None:
    path = tmp_path / "group.json"
    path.write_bytes(raw)
    path.chmod(0o600)
    with pytest.raises(ReceiverGroupRejectedError) as captured:
        load_receiver_group(path)
    assert captured.value.reason is Rejection.INVALID_DOCUMENT


# Real unsafe entries are rejected before their contents can supply credentials.
@pytest.mark.parametrize(
    ("case", "reason"),
    (
        ("symlink", Rejection.UNSAFE_FILE),
        ("fifo", Rejection.UNSAFE_FILE),
        ("directory", Rejection.UNSAFE_FILE),
        ("permissions", Rejection.UNSAFE_PERMISSIONS),
        ("owner", Rejection.OWNER_MISMATCH),
        ("parent-symlink", Rejection.UNSAFE_PARENT),
        ("parent-writable", Rejection.UNSAFE_PARENT),
        ("parent-file", Rejection.UNSAFE_PARENT),
    ),
)
def test_security_rejection_reason(
    tmp_path: Path, case: str, reason: Rejection
) -> None:
    parent = tmp_path / "private"
    parent.mkdir(mode=0o700)
    path = _write(parent / "group.json", _document())
    owner = os.geteuid()
    if case == "symlink":
        link = parent / "link"
        link.symlink_to(path)
        path = link
    elif case in {"fifo", "directory"}:
        path = parent / "entry"
        os.mkfifo(path, 0o600) if case == "fifo" else path.mkdir()
    elif case == "permissions":
        path.chmod(0o640)
    elif case == "owner":
        owner += 1
    elif case == "parent-symlink":
        link = tmp_path / "link"
        link.symlink_to(parent, target_is_directory=True)
        path = link / "group.json"
    elif case == "parent-writable":
        parent.chmod(0o777)
    elif case == "parent-file":
        path = path / "group.json"
    with pytest.raises(ReceiverGroupRejectedError) as captured:
        load_receiver_group(path, expected_owner_uid=owner)
    assert captured.value.reason is reason


# A missing file is an OS failure, preserving errno for the receiver's closed result.
def test_missing_file_retains_errno(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError) as captured:
        load_receiver_group(tmp_path / "missing.json")
    assert captured.value.errno == errno.ENOENT


# An unreliable descriptor read is injected at the real read boundary, not as fake JSON.
def test_read_error_retains_original_errno(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = _write(tmp_path / "group.json", _document())
    error = OSError(errno.EIO, "injected read failure")

    def fail_fdopen(*args: object, **kwargs: object) -> None:
        raise error

    monkeypatch.setattr(receiver_group.os, "fdopen", fail_fdopen)
    with pytest.raises(OSError) as captured:
        load_receiver_group(path)
    assert captured.value is error


# Bad caller arguments are distinct from rejected operator configuration.
@pytest.mark.parametrize("owner", (True, -1, "0"))
def test_invalid_owner_argument(tmp_path: Path, owner: object) -> None:
    with pytest.raises(ValueError):
        load_receiver_group(tmp_path / "missing.json", expected_owner_uid=owner)
