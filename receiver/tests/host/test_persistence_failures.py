import errno
import sqlite3

import pytest
from cura_receiver.generated.receiver_enums_generated import (
    DiagnosticOperation as Operation,
)
from cura_receiver.generated.receiver_enums_generated import (
    PersistenceAdmissionState as State,
)
from cura_receiver.generated.receiver_enums_generated import (
    QuarantineFailureReason as Reason,
)
from cura_receiver.persistence_failures import (
    classify_entity_failure,
    classify_global_failure,
)

# Enumerate binding-exposed result codes, not unrelated authorizer/configuration constants.
_RESULT_PREFIXES = (
    "SQLITE_IOERR",
    "SQLITE_BUSY",
    "SQLITE_LOCKED",
    "SQLITE_CORRUPT",
    "SQLITE_CANTOPEN",
    "SQLITE_READONLY",
    "SQLITE_CONSTRAINT",
    "SQLITE_ERROR",
    "SQLITE_ABORT",
    "SQLITE_AUTH",
    "SQLITE_NOTICE",
    "SQLITE_WARNING",
    "SQLITE_OK",
    "SQLITE_ROW",
    "SQLITE_DONE",
)
_CODES = sorted(
    {
        value
        for name, value in vars(sqlite3).items()
        if type(value) is int
        and name.startswith("SQLITE_")
        and (name.startswith(_RESULT_PREFIXES) or 0 <= value <= 28)
    }
)


# The closed classifier covers every exposed result, including corruption-specific IOERR and unknown values.
@pytest.mark.parametrize("code", _CODES + [0x7FFFFFFF])
def test_all_global_result_codes(code):
    error = sqlite3.OperationalError("not retained")
    error.sqlite_errorcode = code
    failure = classify_global_failure(error)
    if code & 255 == 13:
        expected = State.UNAVAILABLE_DISK_FULL
    elif code & 255 in (11, 26) or code in (8202, 8458):
        expected = State.UNAVAILABLE_CORRUPT
    elif code in (17, 1555, 2067, 2579):
        expected = State.UNAVAILABLE_INCOMPATIBLE_SCHEMA
    else:
        expected = State.UNAVAILABLE_IO
    assert failure.admission_state is expected
    assert (failure.sqlite_primary_code, failure.sqlite_extended_code) == (
        code & 255,
        code,
    )
    assert "not retained" not in repr(failure)


# No global, capacity, corruption, locking or expected identity result can become item poison.
@pytest.mark.parametrize("code", _CODES + [0x7FFFFFFF])
def test_only_entity_constraints_are_isolation_candidates(code):
    error = sqlite3.OperationalError("not retained")
    error.sqlite_errorcode = code
    candidate = classify_entity_failure(error, Operation.APPEND)
    eligible = code & 255 == 19 and code not in (1555, 2067, 2579)
    assert (candidate is not None) == eligible
    if eligible:
        assert candidate.reason is Reason.UNEXPECTED_SQL_CONSTRAINT
        assert candidate.sqlite_extended_code == code


# Host capacity evidence takes its capacity path even when SQLite exposes a general I/O code.
@pytest.mark.parametrize(
    "host_errno,expected",
    [
        (errno.ENOSPC, State.UNAVAILABLE_DISK_FULL),
        (errno.EROFS, State.UNAVAILABLE_IO),
        (errno.EACCES, State.UNAVAILABLE_IO),
        (errno.EIO, State.UNAVAILABLE_IO),
    ],
)
def test_host_failure_codes(host_errno, expected):
    error = OSError(host_errno, "not retained")
    error.sqlite_errorcode = sqlite3.SQLITE_IOERR
    assert classify_global_failure(error).admission_state is expected
    assert classify_entity_failure(error, Operation.APPEND) is None


# Binding, decoding, range and derivation defects remain bounded candidates until isolated reproduction.
@pytest.mark.parametrize(
    "error,operation,reason",
    [
        (TypeError(), Operation.APPEND, Reason.SQL_BINDING_INVARIANT),
        (TypeError(), Operation.DECODE, Reason.ENTITY_DECODING_INVARIANT),
        (ValueError(), Operation.DECODE, Reason.ENTITY_DECODING_INVARIANT),
        (ValueError(), Operation.VALIDATE, Reason.PERSISTENCE_DERIVATION_INVARIANT),
        (OverflowError(), Operation.APPEND, Reason.SQL_RANGE_VIOLATION),
    ],
)
def test_entity_defect_candidates(error, operation, reason):
    assert classify_entity_failure(error, operation).reason is reason
