"""Offline pilot LittleFS evidence decoding; no device access or repair."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import tempfile

REPO = Path(__file__).resolve().parents[2]
LFS = REPO / "firmware/managed_components/joltwallet__littlefs/src/littlefs"
COMPONENTS = REPO / "firmware/components"


def build_reader(destination: Path) -> dict[str, str]:
    """Compile exactly the checked-out production libraries; no new dependency."""
    sources = [Path(__file__).with_name("node_image.c"), LFS / "lfs.c",
               LFS / "lfs_util.c",
               COMPONENTS / "node_persistence/node_persistence_record.c",
               COMPONENTS / "protocol_v2_lora/protocol_v2_lora_schema_generated.c"]
    includes = [LFS, COMPONENTS / "node_persistence/private_include",
                COMPONENTS / "node_persistence/include",
                COMPONENTS / "node_common/include",
                COMPONENTS / "protocol_v2_lora/include"]
    inputs = set(sources)
    for directory in includes:
        inputs.update(directory.glob("*.h"))
    hashes = {str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(inputs)}
    command = ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-DLFS_READONLY", "-DLFS_NO_DEBUG", "-DLFS_NO_WARN", "-DLFS_NO_ERROR",
                    *("-I" + str(p) for p in includes), *(str(p) for p in sources),
                    "-o", str(destination)]
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError("node reader build failed: " + result.stderr)
    return hashes


def decode_record(record: dict) -> dict:
    """Decode only current RF identity/outcome fields; keep context as bytes."""
    payload = bytes.fromhex(record["payload"])
    kind = record["type"]
    result = dict(record)
    if kind in (1, 2):
        result.update(sample_id=struct.unpack_from("<I", payload)[0],
                      reading_body=payload.hex())
    elif kind == 6:
        sample, message = struct.unpack_from("<II", payload)
        result.update(sample_id=sample, message_id=message, frame=payload[8:].hex())
    elif kind in (4, 5):
        cycle, sample, message, domain = struct.unpack_from("<IIIB", payload)
        result.update(cycle_sample_id=cycle, sample_id=sample,
                      message_id=message, domain=domain)
        if kind == 4:
            result["start_offset_ms"] = struct.unpack_from("<I", payload, 13)[0]
        else:
            result.update(attempt_count=payload[13], final_result=payload[14])
    elif kind == 3:
        domain, code, flags, offset, cycle, message, operation, length, schema = (
            struct.unpack_from("<HHHIIIHBB", payload))
        result.update(error_domain=domain, error_code=code, flags=flags,
                      application_offset_ms=offset if flags & 1 else None,
                      cycle_sample_id=cycle if flags & 2 else None,
                      message_id=message if flags & 4 else None,
                      operation=operation, context_schema=schema,
                      context=payload[22:22 + length].hex())
    else:
        raise ValueError("unsupported record type")
    return result


def decode_image(image: Path, reader: Path) -> dict:
    before = hashlib.sha256(image.read_bytes()).hexdigest()
    capture = subprocess.run([str(reader), str(image)], capture_output=True,
                             text=True, timeout=30)
    if hashlib.sha256(image.read_bytes()).hexdigest() != before:
        raise ValueError("source image changed during decoding")
    if capture.returncode:
        raise ValueError(capture.stderr.strip() or "image reader failed")
    logs = json.loads(capture.stdout)
    decoded = {name: None if records is None else [decode_record(r) for r in records]
               for name, records in logs.items()}
    previous = None
    for record in decoded["pending.log"] or []:
        if record["type"] == 6 and (previous is None or previous["type"] != 1 or
                                   previous["sample_id"] != record["sample_id"]):
            raise ValueError("orphan or mismatched backlog binding")
        previous = record
    return dict(schema=1, image_sha256=before, logs=decoded)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("output", type=Path, help="new JSON file; never overwritten")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="cura-node-reader-") as directory:
        reader = Path(directory) / "node-image"
        sources = build_reader(reader)
        report = decode_image(args.image, reader)
        report["decoder_sources"] = sources
    with args.output.open("x") as output:
        json.dump(report, output, indent=2, sort_keys=True)
        output.write("\n")


if __name__ == "__main__":
    main()
