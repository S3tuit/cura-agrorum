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


def finished_payload(cycle, sample, message, domain, attempts, result,
                     start_us=41_200, rssi=-96, snr=0):
    """Independent LE serialization of the only call history node_core produces
    when every call starts and completes: earlier calls time out, then the last
    call carries the ACK for an ACK result or the terminal failure."""
    calls = b""
    for index in range(attempts):
        set_tx = 1_000_000 * (index + 1)
        done = set_tx + 182_500
        last = index == attempts - 1
        ack = last and 1 <= result <= 4
        outcome = (1 if ack else 4 if last and result == 6
                   else 3 if last and result == 7 else 2)
        calls += struct.pack("<BBQQQhh", 3, outcome, set_tx, done,
                             done + 177_300 if ack else 0,
                             rssi if ack else 0, snr if ack else 0)
    calls += bytes(30 * (2 - attempts))
    return struct.pack("<IIIBBBQB", cycle, sample, message, domain, attempts,
                       result, start_us, attempts) + calls


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
                        record(5, finished_payload(51, 42, 73, 2, 2, 4)),
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
    assert (finish["attempt_count"], finish["final_result"]) == (2, 4)
    assert result["logs"]["diagnostic.log"][0]["context"] == "00" * 7


@pytest.mark.parametrize("outcome", range(1, 9))
def test_delivery_outcomes_in_real_image(tmp_path, binaries, outcome):
    attempts = 1 if outcome == 5 else 2
    image = image_from(tmp_path, binaries, {
        "delivery.log": record(5, finished_payload(51, 42, 73, 2, attempts, outcome))})
    before = image.read_bytes()
    finish, = decode_image(image, binaries[0])["logs"]["delivery.log"]
    assert (finish["attempt_count"], finish["final_result"]) == (attempts, outcome)
    assert image.read_bytes() == before


@pytest.mark.parametrize("outcome", [0, 9, 255])
def test_unknown_delivery_outcomes_rejected(tmp_path, binaries, outcome):
    image = image_from(tmp_path, binaries, {
        "delivery.log": record(5, finished_payload(51, 42, 73, 2, 2, outcome))})
    before = image.read_bytes()
    with pytest.raises(ValueError, match="record"):
        decode_image(image, binaries[0])
    assert image.read_bytes() == before


# Transmit calls decode in order; inactive fields are absent, zero SNR and
# negative statistics are values, and the ACK status stays the final result.
def test_transmit_calls_decode_with_absent_fields(tmp_path, binaries):
    image = image_from(tmp_path, binaries, {"delivery.log": record(
        5, finished_payload(51, 42, 73, 2, 2, 2, start_us=7_000, rssi=-241, snr=0))})
    finish, = decode_image(image, binaries[0])["logs"]["delivery.log"]
    assert finish["application_start_us"] == 7_000
    first, second = finish["tx_calls"]
    assert first == dict(tx_started=True, tx_done=True, outcome="ACK_TIMEOUT",
                         set_tx_at_us=1_000_000, tx_done_at_us=1_182_500,
                         ack_rx_done_at_us=None, ack_rssi_dbm_x2=None,
                         ack_snr_db_x4=None)
    assert second["outcome"] == "ACK_RECEIVED" and finish["final_result"] == 2
    assert (second["ack_rx_done_at_us"], second["ack_rssi_dbm_x2"],
            second["ack_snr_db_x4"]) == (2_359_800, -241, 0)


# A call that never started has no TX times; the record is still complete.
def test_unstarted_call_and_no_call_records(tmp_path, binaries):
    unstarted = (struct.pack("<IIIBBBQB", 51, 42, 73, 1, 0, 6, 5, 1)
                 + struct.pack("<BBQQQhh", 0, 4, 0, 0, 0, 0, 0) + bytes(30))
    preflight = struct.pack("<IIIBBBQB", 51, 43, 74, 2, 0, 5, 5, 0) + bytes(60)
    image = image_from(tmp_path, binaries, {
        "delivery.log": record(5, unstarted) + record(5, preflight)})
    first, second = decode_image(image, binaries[0])["logs"]["delivery.log"]
    call, = first["tx_calls"]
    assert call["outcome"] == "DEADLINE_EXPIRED"
    assert call["set_tx_at_us"] is None and call["tx_done_at_us"] is None
    assert second["tx_calls"] == []


# Non-canonical slots are rejected by the production validator, never guessed.
@pytest.mark.parametrize("offset,value", [(24, 2), (25, 1), (42, 1), (54, 1), (23, 3)])
def test_noncanonical_transmit_calls_rejected(tmp_path, binaries, offset, value):
    payload = bytearray(finished_payload(51, 42, 73, 2, 1, 5))
    payload[offset] = value
    image = image_from(tmp_path, binaries, {"delivery.log": record(5, bytes(payload))})
    with pytest.raises(ValueError, match="record"):
        decode_image(image, binaries[0])


# Valid framing and canonical inactive fields cannot make an impossible
# delivery history usable. In particular, silence requires completed TX,
# and attempt-limit exhaustion requires two completed, timed-out calls.
@pytest.mark.parametrize("attempts,result,calls", [
    (0, 8, []),
    (0, 7, []),
    (0, 8, [(0, 2)]),  # Review counterexample: no TX, yet ordinary ACK timeout.
    (0, 5, [(0, 2)]),
    (1, 8, [(1, 2)]),
    (1, 8, [(3, 2)]),
    (2, 8, [(3, 2), (1, 2)]),
    (2, 5, [(3, 2), (3, 2)]),
    (2, 6, [(3, 2), (3, 2)]),
    (1, 7, [(3, 2)]),
    (1, 5, [(3, 3)]),
    (1, 7, [(3, 4)]),
    (1, 6, [(1, 4)]),  # A deadline after SetTx is a local radio error.
    (1, 8, [(3, 2), (0, 4)]),
])
def test_impossible_delivery_histories_rejected(tmp_path, binaries, attempts, result, calls):
    slots = b""
    for index, (flags, outcome) in enumerate(calls):
        start = 1_000_000 * (index + 1) if flags & 1 else 0
        done = start + 182_500 if flags & 2 else 0
        slots += struct.pack("<BBQQQhh", flags, outcome, start, done, 0, 0, 0)
    payload = (struct.pack("<IIIBBBQB", 51, 42, 73, 2, attempts, result, 41_200, len(calls))
               + slots + bytes(30 * (2 - len(calls))))
    image = image_from(tmp_path, binaries, {"delivery.log": record(5, payload)})
    before = image.read_bytes()
    with pytest.raises(ValueError, match="record"):
        decode_image(image, binaries[0])
    assert image.read_bytes() == before


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
    record(5, finished_payload(1, 1, 1, 2, 1, 0)),
    record(5, finished_payload(1, 1, 1, 2, 1, 1)[:-1]),  # old/short layout
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
    fixtures = Path(__file__).resolve().parent / 'fixtures'
    binding = json.loads((fixtures / 'empty-node-storage.json').read_text())
    compressed = (fixtures / 'empty-node-storage.bin.gz').read_bytes()
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


@pytest.mark.parametrize("attempt,ack,header,crc", [(1, 1, 2, 0), (2, 0, 0, 65535)])
def test_ack_window_context_round_trip(tmp_path, binaries, attempt, ack, header, crc):
    context = struct.pack("<BBQHH", attempt, ack, 0x0102030405060708, header, crc)
    payload = struct.pack("<HHHIIIHBB", 4, 15, 7, 800, 51, 73, 17, len(context), 1) + context
    image = image_from(tmp_path, binaries, {"diagnostic.log": record(3, payload)})
    result, = decode_image(image, binaries[0])["logs"]["diagnostic.log"]
    assert result["context"] == context.hex()
    assert result["application_offset_ms"] == 800
    assert result["ack_window_phy"] == dict(attempt_index=attempt, valid_ack_received=bool(ack),
        first_rejection_at_us=0x0102030405060708, header_crc_count=header, payload_crc_count=crc)


@pytest.mark.parametrize("fault", ["attempt0", "attempt3", "ack", "zero", "length", "operation", "schema", "missing"])
def test_noncanonical_ack_window_context_rejected(tmp_path, binaries, fault):
    context = bytearray(struct.pack("<BBQHH", 1, 1, 500_000, 2, 0))
    operation, schema = 17, 1
    if fault == "attempt0": context[0] = 0
    if fault == "attempt3": context[0] = 3
    if fault == "ack": context[1] = 2
    if fault == "zero": context[10] = 0
    if fault == "length": context.pop()
    if fault == "operation": operation = 15
    if fault == "schema": schema = 2
    if fault == "missing": context, schema = b"", 0
    payload = struct.pack("<HHHIIIHBB", 4, 15, 7, 800, 51, 73, operation, len(context), schema) + context
    image = image_from(tmp_path, binaries, {"diagnostic.log": record(3, payload)})
    with pytest.raises(ValueError, match="record"):
        decode_image(image, binaries[0])
