"""Explicit offline test fixtures; never called by production startup or RF runs."""
import hashlib
from pathlib import Path
import re
import sqlite3

from cura_receiver.application_settings import ApplicationSettings
from cura_receiver.communicator_state_persistence import (
    classify_communicator_state_rows, validate_communicator_state,
)
from cura_receiver.database_initializer import initialize_database
from cura_receiver.elapsed_duration import minimum_wait_monotonic_us
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.persistence_control_values import CommunicatorStateLoadStatus
from cura_receiver.sqlite_repository import SqliteRepository
from cura_receiver.tx_airtime import recovery_state


def validate_silence(receipt, *, board_id, boot_id, now_monotonic_us):
    """Operator owns all-transmitter exclusion; elapsed time is checked locally."""
    if (set(receipt) != {'schema', 'board_id', 'boot_id', 'silent_since_monotonic_us',
                         'all_pi_transmitters_remain_silent', 'operator_record'}
            or receipt['schema'] != 1 or receipt['board_id'] != board_id
            or receipt['boot_id'] != boot_id
            or receipt['all_pi_transmitters_remain_silent'] is not True
            or not isinstance(receipt['operator_record'], str)
            or not receipt['operator_record'].strip()):
        raise ValueError('missing or mismatched operator silence attestation')
    since = receipt['silent_since_monotonic_us']
    settings = ApplicationSettings()
    wait = minimum_wait_monotonic_us(settings.airtime_policy.rolling_window_us,
        rate_bound_ppm=settings.time_policy.monotonic_elapsed_rate_bound_ppm)
    if type(since) is not int or not 0 <= since <= now_monotonic_us - wait:
        raise ValueError('complete conservative airtime silence window not established')
    return wait


def create_zero_airtime_database(destination, group_id, receipt, *, board_id, boot_id,
                                 now_monotonic_us, utc_us):
    """Create only a new offline candidate; retain any incomplete file on failure."""
    wait = validate_silence(receipt, board_id=board_id, boot_id=boot_id,
                            now_monotonic_us=now_monotonic_us)
    path = Path(destination)
    if not re.fullmatch(r'prepared-[0-9a-f]{32}\.sqlite3', path.name):
        raise ValueError('use a new prepared-RUN_ID.sqlite3 candidate')
    result = initialize_database(path, group_id)
    if not result.cleanup_complete:
        raise RuntimeError('database initialization cleanup incomplete; preserve candidate')
    policy = ApplicationSettings().airtime_policy
    state = recovery_state(policy, utc_us=utc_us, quality=E.SystemTimeQuality.UNTRUSTED,
                           rtc_health=E.RtcHealth.MISSING, synthetic=False)
    with sqlite3.connect(path) as db:
        db.execute('PRAGMA synchronous=FULL')
        db.execute('PRAGMA foreign_keys=ON')
        db.execute('BEGIN IMMEDIATE')
        repository = SqliteRepository(db)
        blob = validate_communicator_state(state, repository, policy)
        db.execute('INSERT INTO communicator_state VALUES (1,1,1,?,?)',
                   (blob, hashlib.sha256(blob).digest()))
        loaded = classify_communicator_state_rows(repository.read_communicator_state_rows(), repository, policy)
        if loaded.status is not CommunicatorStateLoadStatus.LOADED or loaded.state != state:
            raise ValueError('prepared state did not pass production validation')
        if db.execute('PRAGMA integrity_check').fetchone() != ('ok',) or db.execute('PRAGMA foreign_key_check').fetchall():
            raise ValueError('prepared database failed integrity validation')
    return dict(schema=1, database=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                group_id=group_id.hex(), generation=1, charged_airtime_us=0,
                rtc_provenance=None, silence=receipt, required_silence_us=wait,
                prepared_at_monotonic_us=now_monotonic_us,
                scope='offline test candidate; not installed; no live grant or clock trust asserted')
