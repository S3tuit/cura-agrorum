"""Zero-airtime fixtures require explicit silence and cannot replace history."""
from copy import deepcopy
import sqlite3

import pytest
from run_service import service_prerequisite_reasons
from service_preparation import create_zero_airtime_database, validate_silence
from cura_receiver.application_settings import ApplicationSettings
from cura_receiver.communicator_state_persistence import classify_communicator_state_rows
from cura_receiver.elapsed_duration import minimum_wait_monotonic_us
from cura_receiver.sqlite_repository import SqliteRepository
from tests.support.builders.persistence import GROUP, INSTANCE

RECEIPT = dict(schema=1, board_id='pi', boot_id='boot', silent_since_monotonic_us=100,
               all_pi_transmitters_remain_silent=True, operator_record='exclusive test radio; no other transmitters')
SETTINGS = ApplicationSettings(persistence_startup_budget_us=12_000_000)
WAIT = minimum_wait_monotonic_us(SETTINGS.airtime_policy.rolling_window_us,
    rate_bound_ppm=SETTINGS.time_policy.monotonic_elapsed_rate_bound_ppm)


def create(path, receipt=RECEIPT, now=WAIT + 100):
    return create_zero_airtime_database(path, GROUP, receipt, board_id='pi', boot_id='boot',
        now_monotonic_us=now)


def test_zero_history_passes_production_validation_without_trusting_clock_or_rtc(tmp_path):
    path = tmp_path / ('prepared-' + 'a'*32 + '.sqlite3')
    report = create(path)
    with sqlite3.connect(path) as db:
        repo = SqliteRepository(db)
        loaded = classify_communicator_state_rows(repo.read_communicator_state_rows(), repo, SETTINGS.airtime_policy)
        assert loaded.status.name == 'STATE_UNAVAILABLE'
        assert loaded.state_condition.name == 'MISSING'
        assert loaded.commissioning.name == 'PENDING'
        assert loaded.state is None
        assert db.execute('select count(*) from receiver_instances').fetchone() == (0,)
    before = path.read_bytes()
    with pytest.raises(FileExistsError):
        create(path)
    assert path.read_bytes() == before
    assert report['charged_airtime_us'] is None
    assert report['generation'] == 0
    assert report['commissioning'] == 'PENDING'


@pytest.mark.parametrize('damage', ['short', 'future', 'boot', 'board', 'unconfirmed', 'empty'])
def test_inadequate_silence_never_creates_file(tmp_path, damage):
    receipt = deepcopy(RECEIPT)
    now = WAIT + 100
    if damage == 'short': now -= 1
    if damage == 'future': receipt['silent_since_monotonic_us'] = now + 1
    if damage == 'boot': receipt['boot_id'] = 'old'
    if damage == 'board': receipt['board_id'] = 'other'
    if damage == 'unconfirmed': receipt['all_pi_transmitters_remain_silent'] = False
    if damage == 'empty': receipt['operator_record'] = ''
    path = tmp_path / ('prepared-' + 'b'*32 + '.sqlite3')
    with pytest.raises(ValueError): create(path, receipt, now)
    assert not path.exists()


def ready():
    return dict(started=100, observed_monotonic_us=200,
                health=dict(radio_state_id=2, communicator_sampled_at_monotonic_us=150),
                clock=dict(system_time_quality_id=2, clock_state_generation=1, sampled_at_monotonic_us=150),
                airtime=dict(status='LOADED', total_charged_us=8_000_000, budget_us=36_000_000))


def test_prerequisites_require_fresh_network_and_valid_history():
    assert not service_prerequisite_reasons(ready())
    for quality in (0, 1):
        observed = ready(); observed['clock']['system_time_quality_id'] = quality
        assert 'fresh_CHRONY_SYNCED_observation_required' in service_prerequisite_reasons(observed)
    observed = ready(); observed['observed_monotonic_us'] = 80_000_000
    assert len(service_prerequisite_reasons(observed)) == 2
    observed = ready(); observed['airtime']['total_charged_us'] = 36_000_000
    assert service_prerequisite_reasons(observed) == ['conservative_airtime_budget_has_no_headroom']
    observed = ready(); observed['airtime']['status'] = 'STATE_UNAVAILABLE'
    assert service_prerequisite_reasons(observed) == ['validated_airtime_history_required']


def test_database_observation_distinguishes_seed_and_corruption(tmp_path):
    from service_probe import database_observation
    path = tmp_path / ('prepared-' + 'c'*32 + '.sqlite3')
    create(path)
    with sqlite3.connect(path) as db:
        db.execute('INSERT INTO receiver_instances(instance_ordinal,receiver_instance_id,linux_boot_id,started_at_monotonic_us) VALUES (1,?,?,1)', (bytes(16), bytes(16)))
    observed = database_observation(path)
    assert observed['airtime']['status'] == 'STATE_UNAVAILABLE'
    assert observed['airtime']['condition'] == 'MISSING'
    assert observed['clock'] is None
    assert 'fresh_CHRONY_SYNCED_observation_required' in service_prerequisite_reasons(observed)
    with sqlite3.connect(path) as db:
        db.execute('INSERT INTO communicator_state VALUES (NULL,NULL,NULL,NULL,NULL)')
    observed = database_observation(path)
    assert observed['airtime']['condition'] == 'CORRUPT'
    assert 'validated_airtime_history_required' in service_prerequisite_reasons(observed)


@pytest.mark.parametrize('failure', ['untrusted', 'traffic', 'restart'])
def test_shared_waiter_cannot_pass_bad_preparation(tmp_path, monkeypatch, failure):
    from types import SimpleNamespace
    import run_service
    from run_service import InstalledService
    service = InstalledService(SimpleNamespace(remote='/var/tmp/test', root=tmp_path),
                               dict(package='/opt/test', user='cura-receiver'))
    state = dict(unit_sha256='unit', environment_sha256='env', configuration_sha256='group',
                 boot_id='boot', InvocationID='invocation', MainPID='50', NRestarts='0',
                 ActiveState='active', SubState='running')
    observed = ready() | dict(instance='instance', ordinal=2, linux_boot_id='boot',
                             clean_stop=None, profiles=0, samples=0)
    service.running = state
    service.prior = dict(instance='prior', ordinal=1)
    ticks = iter([0, 0, 2])
    monkeypatch.setattr(run_service.time, 'monotonic', lambda: next(ticks))
    monkeypatch.setattr(run_service.time, 'sleep', lambda _: None)
    if failure == 'untrusted': observed['clock']['system_time_quality_id'] = 0
    if failure == 'traffic': observed['profiles'] = 1
    current = dict(state)
    if failure == 'restart': current['InvocationID'] = 'other'
    service.probe = lambda action: current if action == 'inspect' else observed
    with pytest.raises(TimeoutError if failure == 'untrusted' else ValueError):
        service.wait_for_prerequisites(timeout_seconds=1)
    if failure == 'untrusted':
        assert 'fresh_CHRONY_SYNCED_observation_required' in (tmp_path/'service-prerequisites.json').read_text()


def test_prepared_fixture_commissions_through_real_worker_without_utc(tmp_path):
    from cura_receiver.communicator_state_owner import CommunicatorStateOwner
    from cura_receiver.receiver_startup import ReceiverInstanceStart
    from cura_receiver.tx_airtime import TxAirtimePolicy, AirtimeReason
    from tests.support.coordination.persistence_worker import CheckedPersistenceWorker, prepare_worker_files
    from tests.support.fakes.os_clock import FakeOsClock
    _, config, boot = prepare_worker_files(tmp_path)
    path = tmp_path / ('prepared-' + 'd'*32 + '.sqlite3')
    create(path)
    clock = FakeOsClock(monotonic_us=100)
    worker = CheckedPersistenceWorker(instance=ReceiverInstanceStart(INSTANCE, 0),
        database_path=path, configuration_path=config, boot_id_path=boot, clock=clock)
    worker.start()
    try:
        loaded = worker.wait_started(deadline_monotonic_us=5_000_100).state_load
        owner = CommunicatorStateOwner.from_load(control=worker.control, loaded=loaded)
        policy = TxAirtimePolicy(state_owner=owner, clock=clock)
        assert policy.available_charge_us == 0
        assert policy.recover(deadline_monotonic_us=5_000_100).reason is AirtimeReason.STATE_READY
        assert policy.total_used == 0 and policy.state.generation == 1
        assert policy.state.airtime_snapshot is None and policy.state.rtc_provenance is None
        assert policy.try_spend().reason is AirtimeReason.ALLOWED
        with sqlite3.connect(path) as db:
            assert db.execute('SELECT * FROM airtime_commissioning').fetchall() == []
    finally:
        worker.finish_test()
