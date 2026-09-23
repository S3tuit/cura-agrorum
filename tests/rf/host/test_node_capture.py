"""Real LittleFS images with independently serialized contract records."""
import gzip
import json
import hashlib
from pathlib import Path
import struct
import subprocess
import zlib

import pytest

from node_capture import LFS, build_reader, decode_image


def record(kind, payload, version=2):
    raw = struct.pack("<IBBH", 0x756FEC23, version, kind, len(payload)) + payload
    raw += struct.pack("<H", len(payload) + 14)
    return raw + struct.pack("<I", zlib.crc32(raw))


@pytest.fixture(scope="module")
def binaries(tmp_path_factory):
    root = tmp_path_factory.mktemp("image-binaries")
    reader, writer = root / "reader", root / "writer"
    build_reader(reader)
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-DLFS_NO_DEBUG", "-DLFS_NO_WARN", "-DLFS_NO_ERROR",
                    "-I" + str(LFS), str(Path(__file__).with_name("node_image_fixture.c")),
                    str(LFS / "lfs.c"), str(LFS / "lfs_util.c"), "-o", str(writer)],
                   check=True, capture_output=True, text=True)
    return reader, writer


def image_from(tmp_path, binaries, logs):
    for name, data in logs.items():
        (tmp_path / name).write_bytes(data)
    image = tmp_path / "storage.bin"
    subprocess.run([str(binaries[1]), str(image)], cwd=tmp_path, check=True)
    return image


def test_real_image_all_required_families_and_no_mutation(tmp_path, binaries):
    # Concrete LE field values, independent of production record encoding.
    body = struct.pack("<I", 42) + bytes(28)
    frame = bytes([0x20, 2]) + bytes(8) + struct.pack("<I", 73) + bytes(40)
    logs = {
        "pending.log": record(1, body) + record(6, struct.pack("<II", 42, 73) + frame),
        "quarantine.log": record(2, body),
        "delivery.log": record(4, struct.pack("<IIIBI", 51, 42, 73, 2, 900)) +
                        record(5, struct.pack("<IIIBBB", 51, 42, 73, 2, 3, 4)),
        "diagnostic.log": record(3, struct.pack("<HHHIIIHBB", 1, 6, 7, 12, 51, 73, 1, 7, 1) + bytes(7)),
    }
    image = image_from(tmp_path, binaries, logs)
    before = image.read_bytes()
    result = decode_image(image, binaries[0])
    assert image.read_bytes() == before
    assert result["image_sha256"] == hashlib.sha256(before).hexdigest()
    assert result["logs"]["pending.log"][1]["message_id"] == 73
    assert result["logs"]["quarantine.log"][0]["sample_id"] == 42
    start, finish = result["logs"]["delivery.log"]
    assert start["start_offset_ms"] == 900
    assert (finish["attempt_count"], finish["final_result"]) == (3, 4)
    assert result["logs"]["diagnostic.log"][0]["context"] == "00" * 7


def test_missing_is_distinct_from_empty(tmp_path, binaries):
    image = image_from(tmp_path, binaries, {"pending.log": b""})
    logs = decode_image(image, binaries[0])["logs"]
    assert logs["pending.log"] == []
    assert logs["delivery.log"] is None


@pytest.mark.parametrize("bad", [
    b"truncated",
    record(1, bytes(32))[:-1],
    record(1, bytes(32))[:-1] + b"\xff",
    record(1, bytes(32), version=99),
    record(5, struct.pack("<IIIBBB", 1, 1, 1, 2, 1, 0)),
    record(2, bytes(32)),  # Valid framing, wrong physical file.
])
def test_invalid_records_never_repaired(tmp_path, binaries, bad):
    image = image_from(tmp_path, binaries, {"pending.log": bad})
    before = image.read_bytes()
    with pytest.raises(ValueError, match="record"):
        decode_image(image, binaries[0])
    assert image.read_bytes() == before


@pytest.mark.parametrize("prefix", [b"", record(1, struct.pack("<I", 41) + bytes(28))])
def test_orphan_or_mismatched_binding(tmp_path, binaries, prefix):
    frame = bytes([0x20, 2]) + bytes(8) + struct.pack("<I", 73) + bytes(40)
    image = image_from(tmp_path, binaries, {
        "pending.log": prefix + record(6, struct.pack("<II", 42, 73) + frame)})
    with pytest.raises(ValueError, match="binding"):
        decode_image(image, binaries[0])


@pytest.mark.parametrize("size", [12, 2944 * 1024])
def test_wrong_size_or_unformatted_image(tmp_path, binaries, size):
    image = tmp_path / "storage.bin"
    image.write_bytes(b"\xff" * size)
    with pytest.raises(ValueError, match="size|mount"):
        decode_image(image, binaries[0])


def test_actual_c6_formatter_image(tmp_path, binaries):
    evidence = Path(__file__).resolve().parents[3] / 'receiver/tests/evidence/2026-09-19-pilot-runtime'
    binding = json.loads((evidence / 'NODE_IMAGE_DISCOVERY.json').read_text())
    compressed = (evidence / 'empty-node-storage.bin.gz').read_bytes()
    assert hashlib.sha256(compressed).hexdigest() == binding['gzip_sha256']
    raw = gzip.decompress(compressed)
    assert len(raw) == binding['image_bytes'] == 2944 * 1024
    assert hashlib.sha256(raw).hexdigest() == binding['image_sha256']
    image = tmp_path / 'actual-c6.bin'
    image.write_bytes(raw)
    result = decode_image(image, binaries[0])
    assert result['logs'] == {name: None for name in
                             ('pending.log', 'quarantine.log', 'diagnostic.log', 'delivery.log')}
    assert image.read_bytes() == raw
