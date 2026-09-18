"""Compile the real app-local C framer; drive fragmented input and clock bounds."""
import ctypes
from pathlib import Path
import subprocess

import pytest


class Line(ctypes.Structure):
    _fields_ = [("text", ctypes.c_char * 160), ("length", ctypes.c_size_t),
                ("first_byte_us", ctypes.c_uint64), ("started", ctypes.c_bool),
                ("status", ctypes.c_int)]


class Command(ctypes.Structure):
    _fields_ = [("run", ctypes.c_char * 33), ("selection", ctypes.c_char * 32),
                ("boot", ctypes.c_uint32), ("phase", ctypes.c_uint)]


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    source = Path(__file__).resolve().parents[3] / "firmware/test_apps/radio/main/radio_command.c"
    library = tmp_path_factory.mktemp("command") / "command.so"
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                    str(source), "-o", str(library)], check=True)
    return ctypes.CDLL(str(library))


@pytest.fixture(scope="module")
def feed(library):
    function = library.rf_line_feed
    function.argtypes = [ctypes.POINTER(Line), ctypes.c_int, ctypes.c_uint64]
    function.restype = ctypes.c_int
    return lambda line, byte, clock: function(ctypes.byref(line), byte, clock)


@pytest.fixture(scope="module")
def parse(library):
    function = library.rf_command_parse
    function.argtypes = [ctypes.c_char_p, ctypes.POINTER(Command)]
    function.restype = ctypes.c_bool
    return lambda text, command: function(text, ctypes.byref(command))


@pytest.mark.parametrize("ending", [b"\n", b"\r\n"])
def test_fragmented_command_survives_empty_reads(feed, ending):
    line = Line()
    command = b"RUN aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa RF-001.exchange 1234 0"
    assert feed(line, -1, 900000000) == 0  # No idle timeout.
    for index, byte in enumerate(command + ending):
        assert feed(line, -1, 900000000 + index * 20000) == 0
        status = feed(line, byte, 900000000 + index * 20000)
    assert status == 1 and line.text == command
    assert feed(line, ord("R"), 999999999) == 1  # Cannot replace a completed line.


@pytest.mark.parametrize("ending_at,status", [(1999999, 1), (2000000, 1), (2000001, 4)])
def test_newline_deadline_uses_first_byte(feed, ending_at, status):
    line = Line()
    assert feed(line, ord("R"), 0) == 0
    assert feed(line, ord("U"), 1000000) == 0  # Does not restart the deadline.
    assert feed(line, 10, ending_at) == status


def test_missing_newline_expires_without_more_input(feed):
    line = Line()
    assert feed(line, ord("R"), 100) == 0
    assert feed(line, -1, 2000101) == 4
    assert feed(line, 10, 2000102) == 4


@pytest.mark.parametrize("length,status", [(159, 1), (160, 3), (300, 3)])
def test_overflow_latches_and_cannot_execute_truncated_line(feed, length, status):
    line = Line()
    for byte in b"R" * length + b"\nRUN valid-looking-tail\n":
        result = feed(line, byte, 100)
    assert result == status and line.length == min(159, length)


@pytest.mark.parametrize("bad", [b"R\x00UN\n", b"R\tUN\n", b"R\rUN\n", b"R\xffUN\n"])
def test_invalid_bytes_latch(feed, bad):
    line = Line()
    for byte in bad:
        result = feed(line, byte, 100)
    assert result == 2


@pytest.mark.parametrize("boot,phase", [(0, 0), (657787608, 0), (4294967295, 1)])
def test_actual_selection_and_numeric_boundaries(parse, boot, phase):
    command = Command()
    assert parse(f"RUN a98f95a41f6e48c88bd18a7164b02ba1 RF-001.exchange {boot} {phase}".encode(), command)
    assert command.run == b"a98f95a41f6e48c88bd18a7164b02ba1"
    assert command.selection == b"RF-001.exchange" and command.boot == boot and command.phase == phase


@pytest.mark.parametrize("tail", [b"4294967296 0", b"-1 0", b"01 0", b"1 2", b"1 00", b"1 0 extra",
                                  b"1 0 ", b"1", b"", b"1 ", b"+1 0", b"1  0"])
def test_invalid_numeric_and_truncated_commands(parse, tail):
    command = Command(boot=123)
    assert not parse(b"RUN " + b"a"*32 + b" RF-001.exchange " + tail, command)
    assert command.boot == 123  # Parsing failure cannot partially update selection.


def test_all_truncated_prefixes_and_overlong_tokens_rejected(parse):
    valid = b"RUN " + b"a"*32 + b" RF-001.exchange 123 0"
    for length in range(len(valid)):
        assert not parse(valid[:length], Command())
    for bad in (valid.replace(b"a"*32, b"a"*31), valid.replace(b"a"*32, b"a"*33),
                valid.replace(b"a"*32, b"g"*32), valid.replace(b"RF-001.exchange", b"R"*32)):
        assert not parse(bad, Command())
