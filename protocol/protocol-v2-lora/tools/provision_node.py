#!/usr/bin/env python3
# Usage:
#   provision_node.py [--recv-dir PATH] [--node-dir PATH]
#   provision_node.py --replace-staged-identity [directory options]
#   provision_node.py --rotate-node-identity [directory options]
# --recv-dir selects the directory containing receiver-group.json.
# --node-dir selects where protocol_v2_lora_identity.h is generated.
# --replace-staged-identity overwrites that header but keeps its old ID active.
# --rotate-node-identity overwrites it and permanently retires its old ID.
"""Provision one LoRa v2 node identity from a receiver-group master key."""

from __future__ import annotations

import argparse
import stat
import sys
from pathlib import Path

from provisioning_common import (
    GROUP_STATE_FILENAME,
    NODE_IDENTITY_FILENAME,
    ProvisioningError,
    ReceiverGroupState,
    derive_node_key,
    generate_unique_node_id,
    load_receiver_group,
    parse_node_identity_id,
    render_node_identity,
    render_receiver_group,
    replace_secret,
    require_existing_directory,
    write_new_secret,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RECV_DIR = REPO_ROOT / "receiver"
DEFAULT_NODE_DIR = REPO_ROOT / "firmware" / "main"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="provision a Cura Agrorum LoRa v2 node"
    )
    parser.add_argument(
        "--recv-dir",
        type=Path,
        default=DEFAULT_RECV_DIR,
        help=(
            "directory containing receiver-group.json "
            f"(default: {DEFAULT_RECV_DIR})"
        ),
    )
    parser.add_argument(
        "--node-dir",
        type=Path,
        default=DEFAULT_NODE_DIR,
        help=(
            "directory receiving protocol_v2_lora_identity.h "
            f"(default: {DEFAULT_NODE_DIR})"
        ),
    )
    replacement = parser.add_mutually_exclusive_group()
    replacement.add_argument(
        "--replace-staged-identity",
        action="store_true",
        help="replace the generated header while leaving its old ID active",
    )
    replacement.add_argument(
        "--rotate-node-identity",
        action="store_true",
        help="replace the generated header and retire its old ID",
    )
    args = parser.parse_args(argv)

    try:
        recv_dir = require_existing_directory(
            args.recv_dir, "receiver directory"
        )
        node_dir = require_existing_directory(args.node_dir, "node directory")
        state_path = recv_dir / GROUP_STATE_FILENAME
        identity_path = node_dir / NODE_IDENTITY_FILENAME
        state = load_receiver_group(state_path)

        replacing = (
            args.replace_staged_identity or args.rotate_node_identity
        )
        identity_exists = identity_path.exists()
        if identity_exists and not replacing:
            raise ProvisioningError(
                f"node identity already exists: {identity_path}; use an "
                "explicit replacement or rotation option"
            )
        if replacing and not identity_exists:
            raise ProvisioningError(
                f"cannot replace an absent node identity: {identity_path}"
            )

        previous_identity: str | None = None
        previous_node_id: bytes | None = None
        if identity_exists:
            previous_identity = _read_identity(identity_path)
            previous_node_id = parse_node_identity_id(previous_identity)
            if previous_node_id not in state.active_node_ids:
                raise ProvisioningError(
                    "existing staged node ID is not active in this receiver "
                    "group"
                )

        node_id = generate_unique_node_id(state)
        node_key = derive_node_key(state.group_master_key, node_id)
        next_state = _state_after_provisioning(
            state,
            node_id,
            previous_node_id if args.rotate_node_identity else None,
        )
        identity_content = render_node_identity(node_id, node_key)

        if identity_exists:
            replace_secret(identity_path, identity_content)
        else:
            write_new_secret(identity_path, identity_content)

        try:
            replace_secret(state_path, render_receiver_group(next_state))
        except ProvisioningError:
            _restore_identity(identity_path, previous_identity)
            raise

        print(f"Provisioned node_id {node_id.hex()}")
        if args.rotate_node_identity:
            assert previous_node_id is not None
            print(f"Retired previous node_id {previous_node_id.hex()}")
        elif args.replace_staged_identity:
            assert previous_node_id is not None
            print(f"Left previous node_id {previous_node_id.hex()} active")
        print(f"Wrote {identity_path}")
        print(f"Updated {state_path}")
        return 0
    except ProvisioningError as exc:
        print(f"node provisioning failed: {exc}", file=sys.stderr)
        return 2


def _state_after_provisioning(
    state: ReceiverGroupState,
    node_id: bytes,
    rotated_node_id: bytes | None,
) -> ReceiverGroupState:
    active = set(state.active_node_ids)
    retired = set(state.retired_node_ids)
    if rotated_node_id is not None:
        active.remove(rotated_node_id)
        retired.add(rotated_node_id)
    active.add(node_id)
    return ReceiverGroupState(
        group_id=state.group_id,
        group_master_key=state.group_master_key,
        active_node_ids=frozenset(active),
        retired_node_ids=frozenset(retired),
    )


def _read_identity(path: Path) -> str:
    try:
        metadata = path.stat()
    except OSError as exc:
        raise ProvisioningError(
            f"cannot inspect existing node identity {path}: {exc}"
        ) from exc
    if not stat.S_ISREG(metadata.st_mode):
        raise ProvisioningError(
            f"existing node identity is not a regular file: {path}"
        )
    if stat.S_IMODE(metadata.st_mode) & 0o077:
        raise ProvisioningError(
            f"existing node identity is accessible by group or others; "
            f"run chmod 600 {path}"
        )

    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise ProvisioningError(
            f"cannot read existing node identity {path}: {exc}"
        ) from exc


def _restore_identity(path: Path, previous: str | None) -> None:
    try:
        if previous is None:
            path.unlink(missing_ok=True)
        else:
            replace_secret(path, previous)
    except (OSError, ProvisioningError) as exc:
        raise ProvisioningError(
            f"receiver state update failed and node identity rollback also "
            f"failed: {exc}"
        ) from exc


if __name__ == "__main__":
    raise SystemExit(main())
