"""RF-only per-boot evidence, retained at the terminal receipt boundary."""
from dataclasses import dataclass
import re


@dataclass(frozen=True)
class SleepObservation:
    boot: str
    reset_reason: int
    duration_us: int
    finalize: str


def observation_frames(raw):
    """Yield complete RF-only frames and their offsets; resync at each RS.

    Native USB can retain a prefix and lose its suffix before reconnection.
    Only an RS..US frame is evidence. A subsequent RS discards that incomplete
    prefix; every complete frame remains subject to strict semantic validation.
    """
    for match in re.finditer(rb'\x1e([^\x1e\x1f]*)\x1f', raw):
        payload = match[1]
        lines = payload.split(b'\n')
        if (len(payload) > 511 or lines[-1] != b'' or
                not (len(lines) == 3 and lines[0].startswith(b'RF_NODE_SLEEP ') and
                     lines[1].startswith(b'RF_FINALIZE ') or
                     len(lines) == 2 and lines[0].startswith((b'RF_PARKED ', b'RF_FINALIZE_FAILED ')))):
            raise ValueError('invalid observation frame')
        yield match.start(), payload


def sleep_observations(raw, duration_seconds, wakes, *, require_parked=False):
    # Chip resets can remove USB before ROM prints its banner. These application
    # records repeat at finalization; a repeat never constitutes another wake.
    records, bound = [], set()
    parked = False
    if b'panic' in raw.lower() or b'Guru Meditation' in raw:
        raise ValueError('node panic during accelerated run')
    lines = [line for _, frame in observation_frames(raw) for line in frame.split(b'\n')[:-1]]
    for line in lines:
        line = line.rstrip(b'\r')
        if b'panic' in line.lower() or b'Guru Meditation' in line:
            raise ValueError('node panic during accelerated run')
        if b'RF_NODE_SLEEP' in line:
            match = re.fullmatch(rb'RF_NODE_SLEEP boot=([0-9a-f]{16}) reset_reason=(\d+) '
                                 rb'duration_us=(\d+) finalize=([0-9a-f]{8})', line)
            if not match:
                raise ValueError('invalid sleep observation framing')
            boot, reason, duration, nonce = match.groups()
            record = SleepObservation(boot.decode(), int(reason), int(duration), nonce.decode())
            if record.duration_us != duration_seconds * 1_000_000 or not 0 <= record.reset_reason <= 255:
                raise ValueError('invalid sleep observation duration/reset')
            if parked:
                raise ValueError('wake observation after terminal park')
            if records and record.boot == records[-1].boot:
                if record != records[-1]:
                    raise ValueError('changed observation within one boot')
                continue
            if (any(record.boot == old.boot or record.finalize == old.finalize for old in records)
                    or records and records[-1].finalize not in bound):
                raise ValueError('reused identity or missing finalization before next boot')
            if ((not records and record.reset_reason == 8)
                    or (records and record.reset_reason != 8)):
                raise ValueError('unexpected reset during accelerated run')
            records.append(record)
            if len(records) > wakes:
                raise ValueError('extra wake observation')
        elif line.startswith(b'RF_FINALIZE_FAILED'):
            raise ValueError('node finalization failed')
        elif line.startswith(b'RF_FINALIZE '):
            match = re.fullmatch(rb'RF_FINALIZE ([0-9a-f]{8})', line)
            if (not match or not records or parked
                    or match[1].decode() != records[-1].finalize):
                raise ValueError('unbound finalization greeting')
            bound.add(records[-1].finalize)
        elif line.startswith(b'RF_PARKED '):
            match = re.fullmatch(rb'RF_PARKED ([0-9a-f]{8})', line)
            if (not match or not records or records[-1].finalize not in bound
                    or match[1].decode() != records[-1].finalize):
                raise ValueError('unbound terminal park')
            parked = True
    if require_parked and not parked:
        raise ValueError('missing terminal park observation')
    return tuple(record for record in records if record.finalize in bound)


def sleep_count(raw, duration_seconds, wakes):
    return len(sleep_observations(raw, duration_seconds, wakes))


def verify_wake_readings(raw, duration_seconds, wakes, reset_reasons):
    records = sleep_observations(raw, duration_seconds, wakes, require_parked=True)
    if len(records) != wakes or [r.reset_reason for r in records] != list(reset_reasons):
        raise ValueError('wake observations differ from authenticated current readings')
    return [dict(boot=r.boot, reset_reason=r.reset_reason, finalize=r.finalize) for r in records]
