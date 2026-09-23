"""Read the kernel's canonical boot UUID on the persistence owner's thread."""

from __future__ import annotations

import re
from pathlib import Path
from uuid import UUID

LINUX_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
_CANONICAL_UUID = re.compile(
    rb"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}\n?"
)


def read_linux_boot_id(path: Path = LINUX_BOOT_ID_PATH) -> bytes:
    """Accept one lowercase hyphenated UUID and the kernel's optional final LF."""

    with path.open("rb") as stream:
        value = stream.read(38)  # One byte past the longest accepted representation.
    if _CANONICAL_UUID.fullmatch(value) is None:
        raise ValueError("Linux boot ID is not one canonical UUID")
    return UUID(value.rstrip(b"\n").decode("ascii")).bytes
