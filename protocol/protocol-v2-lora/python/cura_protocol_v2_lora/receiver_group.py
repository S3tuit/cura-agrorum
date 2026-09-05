"""One strict, read-only receiver-group loader shared by tools and runtime."""

from __future__ import annotations

import errno
import grp
import json
import os
import pwd
import re
import stat
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path

GROUP_STATE_FILENAME = "receiver-group.json"
GROUP_STATE_FORMAT_VERSION = 1
GROUP_ID_SIZE = 8
GROUP_MASTER_KEY_SIZE = 32
NODE_ID_SIZE = 8
_HEX_RE = re.compile(r"^[0-9a-f]+$")


class ReceiverGroupRejection(Enum):
    """Closed content/security reasons; never a persisted numeric catalogue."""

    INVALID_DOCUMENT = auto()
    UNSUPPORTED_VERSION = auto()
    INVALID_GROUP_ID = auto()
    INVALID_MASTER_KEY = auto()
    INVALID_NODE_IDS = auto()
    OVERLAPPING_NODE_IDS = auto()
    UNSAFE_PARENT = auto()
    UNSAFE_FILE = auto()
    OWNER_MISMATCH = auto()
    UNSAFE_PERMISSIONS = auto()


class ReceiverGroupRejectedError(ValueError):
    """Unsafe configuration; only ``reason`` crosses the receiver boundary."""

    def __init__(self, reason: ReceiverGroupRejection, message: str) -> None:
        super().__init__(message)
        self.reason = reason


@dataclass(frozen=True, slots=True)
class ReceiverGroupState:
    group_id: bytes
    group_master_key: bytes = field(repr=False)
    active_node_ids: frozenset[bytes]
    retired_node_ids: frozenset[bytes]


def load_receiver_group(
    path: Path,
    *,
    expected_owner_uid: int | None = None,
) -> ReceiverGroupState:
    """Read the same descriptor whose path, owner, type and mode were checked."""

    if not isinstance(path, Path):
        raise TypeError("receiver group path must be a Path")
    if expected_owner_uid is None:
        expected_owner_uid = os.geteuid()
    if type(expected_owner_uid) is not int or expected_owner_uid < 0:
        raise ValueError(
            "receiver group expected owner UID must be a non-negative integer"
        )

    descriptor = _open_receiver_group(path, expected_owner_uid)
    try:
        stream = os.fdopen(descriptor, "r", encoding="utf-8")
        descriptor = -1
        with stream:
            document = json.load(stream)
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ReceiverGroupRejectedError(
            ReceiverGroupRejection.INVALID_DOCUMENT,
            "cannot read receiver group state: invalid UTF-8 or JSON",
        ) from exc
    finally:
        if descriptor >= 0:
            os.close(descriptor)

    if not isinstance(document, dict):
        raise ReceiverGroupRejectedError(
            ReceiverGroupRejection.INVALID_DOCUMENT,
            "receiver group state must be a JSON object",
        )
    if set(document) != {
        "format_version",
        "group_id",
        "group_master_key",
        "active_node_ids",
        "retired_node_ids",
    }:
        raise ReceiverGroupRejectedError(
            ReceiverGroupRejection.INVALID_DOCUMENT,
            "receiver group state fields differ",
        )
    if (
        type(document["format_version"]) is not int
        or document["format_version"] != GROUP_STATE_FORMAT_VERSION
    ):
        raise ReceiverGroupRejectedError(
            ReceiverGroupRejection.UNSUPPORTED_VERSION,
            "unsupported receiver group state format_version",
        )

    group_id = _decode_hex(
        document["group_id"],
        GROUP_ID_SIZE,
        "group_id",
        ReceiverGroupRejection.INVALID_GROUP_ID,
    )
    if group_id == bytes(GROUP_ID_SIZE):
        raise ReceiverGroupRejectedError(
            ReceiverGroupRejection.INVALID_GROUP_ID,
            "group_id must not be all zero",
        )
    group_master_key = _decode_hex(
        document["group_master_key"],
        GROUP_MASTER_KEY_SIZE,
        "group_master_key",
        ReceiverGroupRejection.INVALID_MASTER_KEY,
    )
    active = _decode_node_id_set(document["active_node_ids"], "active_node_ids")
    retired = _decode_node_id_set(document["retired_node_ids"], "retired_node_ids")
    if active & retired:
        raise ReceiverGroupRejectedError(
            ReceiverGroupRejection.OVERLAPPING_NODE_IDS,
            "active_node_ids and retired_node_ids overlap",
        )
    return ReceiverGroupState(
        group_id, group_master_key, frozenset(active), frozenset(retired)
    )


def _open_receiver_group(path: Path, expected_owner_uid: int) -> int:
    expanded = path.expanduser()
    absolute = expanded if expanded.is_absolute() else Path.cwd() / expanded
    components = absolute.parts[1:]
    if not components:
        raise ReceiverGroupRejectedError(
            ReceiverGroupRejection.UNSAFE_FILE,
            "receiver group state is not a regular, non-symlink file",
        )
    directory_flags = os.O_PATH | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
    file_flags = os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW | os.O_CLOEXEC
    directory_descriptor = os.open(absolute.anchor, directory_flags)
    try:
        root_owner_uid = os.fstat(directory_descriptor).st_uid
        trusted_owner_uids = frozenset(
            (0, root_owner_uid, os.geteuid(), expected_owner_uid)
        )
        current_parent = Path(absolute.anchor)
        _validate_parent_directory(
            directory_descriptor, current_parent, trusted_owner_uids
        )
        for component in components[:-1]:
            current_parent /= component
            try:
                next_descriptor = os.open(
                    component, directory_flags, dir_fd=directory_descriptor
                )
            except OSError as exc:
                if exc.errno in (errno.ELOOP, errno.ENOTDIR):
                    raise ReceiverGroupRejectedError(
                        ReceiverGroupRejection.UNSAFE_PARENT,
                        "receiver group path parent is unavailable, not a directory, or a symlink",
                    ) from exc
                raise
            previous_descriptor = directory_descriptor
            directory_descriptor = next_descriptor
            os.close(previous_descriptor)
            _validate_parent_directory(
                directory_descriptor, current_parent, trusted_owner_uids
            )
        try:
            descriptor = os.open(
                components[-1], file_flags, dir_fd=directory_descriptor
            )
        except OSError as exc:
            if exc.errno == errno.ELOOP:
                raise ReceiverGroupRejectedError(
                    ReceiverGroupRejection.UNSAFE_FILE,
                    "receiver group state is not a regular, non-symlink file",
                ) from exc
            raise
    finally:
        os.close(directory_descriptor)

    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise ReceiverGroupRejectedError(
                ReceiverGroupRejection.UNSAFE_FILE,
                "receiver group state is not a regular, non-symlink file",
            )
        if metadata.st_uid != expected_owner_uid:
            raise ReceiverGroupRejectedError(
                ReceiverGroupRejection.OWNER_MISMATCH,
                f"receiver group state owner does not match expected UID {expected_owner_uid}",
            )
        if stat.S_IMODE(metadata.st_mode) & 0o077:
            raise ReceiverGroupRejectedError(
                ReceiverGroupRejection.UNSAFE_PERMISSIONS,
                "receiver group state is accessible by group or others; run chmod 600",
            )
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _validate_parent_directory(
    descriptor: int,
    path: Path,
    trusted_owner_uids: frozenset[int],
) -> None:
    metadata = os.fstat(descriptor)
    mode = stat.S_IMODE(metadata.st_mode)
    if not stat.S_ISDIR(metadata.st_mode):
        raise ReceiverGroupRejectedError(
            ReceiverGroupRejection.UNSAFE_PARENT,
            f"receiver group path parent is not a directory: {path}",
        )
    if metadata.st_uid not in trusted_owner_uids:
        raise ReceiverGroupRejectedError(
            ReceiverGroupRejection.UNSAFE_PARENT,
            f"receiver group path parent is owned by an untrusted user: {path}",
        )
    untrusted_group_write = mode & stat.S_IWGRP and not _group_is_exclusively_trusted(
        metadata.st_gid,
        trusted_owner_uids,
    )
    if (untrusted_group_write or mode & stat.S_IWOTH) and not mode & stat.S_ISVTX:
        raise ReceiverGroupRejectedError(
            ReceiverGroupRejection.UNSAFE_PARENT,
            f"receiver group path parent is writable by an untrusted user: {path}",
        )


def _group_is_exclusively_trusted(
    group_gid: int, trusted_owner_uids: frozenset[int]
) -> bool:
    try:
        group = grp.getgrgid(group_gid)
        members = {
            account.pw_uid for account in pwd.getpwall() if account.pw_gid == group_gid
        }
        members.update(pwd.getpwnam(member_name).pw_uid for member_name in group.gr_mem)
    except (KeyError, OSError):
        return False
    return bool(members) and members <= trusted_owner_uids


def _decode_hex(
    value: object,
    size: int,
    name: str,
    reason: ReceiverGroupRejection,
) -> bytes:
    if (
        not isinstance(value, str)
        or len(value) != size * 2
        or _HEX_RE.fullmatch(value) is None
    ):
        raise ReceiverGroupRejectedError(
            reason, f"{name} must contain exactly {size * 2} lowercase hex digits"
        )
    return bytes.fromhex(value)


def _decode_node_id_set(value: object, name: str) -> set[bytes]:
    reason = ReceiverGroupRejection.INVALID_NODE_IDS
    if not isinstance(value, list):
        raise ReceiverGroupRejectedError(reason, f"{name} must be a JSON array")
    result: set[bytes] = set()
    for index, encoded in enumerate(value):
        node_id = _decode_hex(encoded, NODE_ID_SIZE, f"{name}[{index}]", reason)
        if node_id == bytes(NODE_ID_SIZE):
            raise ReceiverGroupRejectedError(
                reason, f"{name}[{index}] must not be all zero"
            )
        if node_id in result:
            raise ReceiverGroupRejectedError(
                reason, f"{name} contains a duplicate node ID"
            )
        result.add(node_id)
    return result
