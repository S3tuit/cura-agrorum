from __future__ import annotations

import errno
import json
import os
from dataclasses import FrozenInstanceError
from pathlib import Path
from threading import Thread

import pytest

from cura_protocol_v2_lora.receiver_group import ReceiverGroupRejection
from cura_receiver import receiver_configuration
from cura_receiver.generated.receiver_enums_generated import DiagnosticOperation
from cura_receiver.platform.linux_boot_identity import read_linux_boot_id
from cura_receiver.receiver_configuration import (
    PersistenceControlInterfaceViolation,
    ReceiverConfigurationLoadStatus as Status,
    ReceiverConfigurationReader,
)

BOOT_TEXT = b"00112233-4455-6677-8899-aabbccddeeff"
BOOT_ID = bytes.fromhex("00112233445566778899aabbccddeeff")


def _configuration(tmp_path: Path) -> tuple[Path, Path]:
    path = tmp_path / "receiver-group.json"
    path.write_text(
        json.dumps(
            {
                "format_version": 1,
                "group_id": "0102030405060708",
                "group_master_key": "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f",
                "active_node_ids": ["1122334455667788"],
                "retired_node_ids": [],
            }
        ),
        encoding="utf-8",
    )
    path.chmod(0o600)
    boot = tmp_path / "boot_id"
    boot.write_bytes(BOOT_TEXT + b"\n")
    return path, boot


# A successful read returns both immutable identities without exposing secret material.
def test_configuration_snapshot(
    tmp_path: Path, caplog: pytest.LogCaptureFixture, capsys: pytest.CaptureFixture[str]
) -> None:
    path, boot = _configuration(tmp_path)
    original = path.read_bytes()
    result = ReceiverConfigurationReader(path, boot_id_path=boot).read()
    assert result.status is Status.LOADED
    assert result.operation is DiagnosticOperation.NONE
    assert result.linux_boot_id == BOOT_ID
    assert result.configuration.group_master_key == bytes(range(32))
    assert result.configuration.active_node_ids == frozenset(
        {bytes.fromhex("1122334455667788")}
    )
    assert result.interface_violation is PersistenceControlInterfaceViolation.NONE
    assert result.protocol_rejection is result.os_errno is None
    assert "configuration=" not in repr(result)
    assert "group_master_key" not in repr(result.configuration)
    with pytest.raises(FrozenInstanceError):
        result.linux_boot_id = bytes(16)
    with pytest.raises(FrozenInstanceError):
        result.configuration.group_id = bytes(8)
    assert path.read_bytes() == original
    assert caplog.text == ""
    assert capsys.readouterr() == ("", "")


# Failure projections contain neither configuration nor boot identity or exception text.
@pytest.mark.parametrize(
    ("case", "status", "reason", "os_errno"),
    (
        ("missing-config", Status.OS_ERROR, None, errno.ENOENT),
        ("missing-boot", Status.OS_ERROR, None, errno.ENOENT),
        (
            "malformed",
            Status.CONFIGURATION_REJECTED,
            ReceiverGroupRejection.INVALID_DOCUMENT,
            None,
        ),
        (
            "permissions",
            Status.CONFIGURATION_REJECTED,
            ReceiverGroupRejection.UNSAFE_PERMISSIONS,
            None,
        ),
        (
            "owner",
            Status.CONFIGURATION_REJECTED,
            ReceiverGroupRejection.OWNER_MISMATCH,
            None,
        ),
        ("boot-invalid", Status.HOST_IDENTITY_REJECTED, None, None),
    ),
)
def test_configuration_failure_projection(
    tmp_path: Path, case: str, status: Status, reason: object, os_errno: int | None
) -> None:
    path, boot = _configuration(tmp_path)
    owner = os.geteuid()
    if case == "missing-config":
        path = tmp_path / "missing.json"
    elif case == "missing-boot":
        boot = tmp_path / "missing-boot"
    elif case == "malformed":
        path.write_text("sensitive invalid document", encoding="utf-8")
    elif case == "permissions":
        path.chmod(0o644)
    elif case == "owner":
        owner += 1
    elif case == "boot-invalid":
        boot.write_bytes(b"invalid")
    result = ReceiverConfigurationReader(
        path, boot_id_path=boot, expected_owner_uid=owner
    ).read()
    assert result.status is status
    assert result.operation is DiagnosticOperation.READ
    assert result.interface_violation is PersistenceControlInterfaceViolation.NONE
    assert result.protocol_rejection is reason
    assert result.os_errno == os_errno
    assert result.configuration is result.linux_boot_id is None
    assert "sensitive" not in repr(result)


# Wrong-thread access is rejected before the strict loader or boot adapter performs I/O.
def test_only_owner_loads_configuration(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path, boot = _configuration(tmp_path)
    reader = ReceiverConfigurationReader(path, boot_id_path=boot)

    def forbidden(*args: object, **kwargs: object) -> None:
        pytest.fail("non-owner performed configuration I/O")

    monkeypatch.setattr(receiver_configuration, "load_receiver_group", forbidden)
    results = []
    child = Thread(target=lambda: results.append(reader.read()))
    child.start()
    child.join(timeout=5)
    assert not child.is_alive()
    assert len(results) == 1
    assert results[0].status is Status.INTERFACE_VIOLATION
    assert (
        results[0].interface_violation
        is PersistenceControlInterfaceViolation.WRONG_CALLER
    )
    assert results[0].configuration is results[0].linux_boot_id is None


# The proc representation permits only a canonical UUID with an optional single LF.
@pytest.mark.parametrize("raw", (BOOT_TEXT, BOOT_TEXT + b"\n"))
def test_canonical_boot_id(tmp_path: Path, raw: bytes) -> None:
    path = tmp_path / "boot"
    path.write_bytes(raw)
    assert read_linux_boot_id(path) == BOOT_ID


# UUID parsing must not normalize alternate spellings, whitespace or extra identities.
@pytest.mark.parametrize(
    "raw",
    (
        b"",
        BOOT_TEXT.upper(),
        BOOT_TEXT.replace(b"-", b""),
        b"{" + BOOT_TEXT + b"}",
        BOOT_TEXT + b"\r\n",
        b" " + BOOT_TEXT,
        BOOT_TEXT + b"\n\n",
        BOOT_TEXT + b"\n" + BOOT_TEXT,
        BOOT_TEXT[:-1],
        BOOT_TEXT + b"x",
        b"\xff" * 36,
    ),
)
def test_noncanonical_boot_id(tmp_path: Path, raw: bytes) -> None:
    path = tmp_path / "boot"
    path.write_bytes(raw)
    with pytest.raises(ValueError, match="canonical UUID"):
        read_linux_boot_id(path)
