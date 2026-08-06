#!/usr/bin/env python3
# Usage:
#   init_receiver_group.py [--recv-dir PATH]
#   init_receiver_group.py --rotate-master-key \
#       --acknowledge-all-nodes-require-reprovisioning [--recv-dir PATH]
# --recv-dir selects the directory containing receiver-group.json.
# --rotate-master-key replaces the group ID and master key and retires all IDs.
# --acknowledge-all-nodes-require-reprovisioning confirms that destructive
# consequence and is required together with --rotate-master-key.
"""Create or deliberately rotate a LoRa v2 receiver-group master key."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from provisioning_common import (
    GROUP_STATE_FILENAME,
    ProvisioningError,
    generate_receiver_group,
    load_receiver_group,
    render_receiver_group,
    replace_secret,
    require_existing_directory,
    rotate_receiver_group,
    write_new_secret,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RECV_DIR = REPO_ROOT / "receiver"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="initialize a Cura Agrorum LoRa v2 receiver group"
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
        "--rotate-master-key",
        action="store_true",
        help="replace the group master key and retire every active node ID",
    )
    parser.add_argument(
        "--acknowledge-all-nodes-require-reprovisioning",
        action="store_true",
        help="confirm that every node must receive a fresh identity and key",
    )
    args = parser.parse_args(argv)

    if (
        args.rotate_master_key
        != args.acknowledge_all_nodes_require_reprovisioning
    ):
        parser.error(
            "--rotate-master-key and "
            "--acknowledge-all-nodes-require-reprovisioning "
            "must be supplied together"
        )

    try:
        recv_dir = require_existing_directory(
            args.recv_dir, "receiver directory"
        )
        state_path = recv_dir / GROUP_STATE_FILENAME

        if not args.rotate_master_key:
            if state_path.exists():
                raise ProvisioningError(
                    f"receiver group already exists: {state_path}"
                )
            state = generate_receiver_group()
            write_new_secret(state_path, render_receiver_group(state))
            print(f"Initialized receiver group {state.group_id.hex()}")
            print(f"Wrote {state_path}")
            return 0

        if not state_path.exists():
            raise ProvisioningError(
                "cannot rotate an absent receiver group; initialize it "
                "without rotation flags first"
            )
        previous = load_receiver_group(state_path)
        state = rotate_receiver_group(previous)
        replace_secret(state_path, render_receiver_group(state))
        print(f"Rotated receiver group to {state.group_id.hex()}")
        print(
            f"Retired {len(previous.active_node_ids)} previously active "
            "node ID(s)"
        )
        print(f"Wrote {state_path}")
        return 0
    except ProvisioningError as exc:
        print(f"receiver-group initialization failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
