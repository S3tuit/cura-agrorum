"""Application startup uses the real worker, policy owners and radio state machine."""

from dataclasses import replace
import sqlite3

from cura_receiver.application import ReceiverApplication, authentication_keys
from cura_receiver.application_settings import ApplicationSettings
from cura_receiver.communicator_scheduler import Work
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.ports.ds3231 import Ds3231ReadResult, Ds3231ReadStatus
from cura_receiver.radio import Radio
from cura_receiver.sx1262 import Sx1262
from cura_receiver.receiver_startup import ReceiverInstanceStart
from cura_protocol_v2_lora.receiver_group import ReceiverGroupState
from tests.support.builders.persistence import INSTANCE
from tests.support.coordination.persistence_worker import CheckedPersistenceWorker, prepare_worker_files
from tests.support.fakes.os_clock import FakeOsClock
from tests.support.fakes.radio_io import PhysicalPort, Wait
from tests.support.fakes.ds3231 import FakeDs3231Control
from tests.support.fakes.chrony import FakeChronyControl
from tests.support.fakes.kernel_clock import FakeKernelClock


def test_authentication_keys_match_public_protocol_vector():
    node = bytes.fromhex('0102030405060708')
    configuration = ReceiverGroupState(bytes(16), bytes(range(32)), frozenset((node,)), frozenset((b'r' * 8,)))
    assert authentication_keys(configuration) == {node: bytes.fromhex('c0f9a1a0f386692e01028082be92330e')}


def make_application(tmp_path, *, reuse_storage=False, instance_id=INSTANCE):
    database, config, boot = ((tmp_path / "worker.db", tmp_path / "test-group.json", tmp_path / "boot-id")
        if reuse_storage else prepare_worker_files(tmp_path))
    clock = FakeOsClock(monotonic_us=100)
    settings = replace(ApplicationSettings(), database_path=database, configuration_path=config,
        sqlite_temporary_directory=tmp_path / 'sqlite-temp', minimum_free_bytes=0)
    instance = ReceiverInstanceStart(instance_id, 100)
    worker = CheckedPersistenceWorker(instance=instance, database_path=database,
        configuration_path=config, boot_id_path=boot, clock=clock)
    io = PhysicalPort(clock)
    radio = Radio(Sx1262(io, clock, Wait(clock)))
    rtc = FakeDs3231Control()
    rtc.read_results.append(Ds3231ReadResult(Ds3231ReadStatus.MISSING, 100, 100))
    app = ReceiverApplication(instance=instance, settings=settings, worker=worker, clock=clock,
        kernel=FakeKernelClock(), rtc=rtc, chrony=FakeChronyControl(), radio=radio)
    return app, io, database, config


def test_offline_missing_state_starts_rx_untrusted_with_initial_health(tmp_path):
    app, io, database, _ = make_application(tmp_path)
    try:
        assert app.start().ready
        c = app.runtime.communicator
        assert c.time.state.quality is E.SystemTimeQuality.UNTRUSTED
        assert c.airtime.available_charge_us == 0
        assert app.radio.state is E.RadioState.RX_SINGLE
        assert app.runtime.step().work is Work.HEALTH
        assert app.runtime.telemetry.health_sequence == 1
        assert not app.chrony.calls
        assert not any(command[0] == 0x83 for command in io.commands)
        app.worker.request_stop(deadline_monotonic_us=app.clock.now_monotonic_us() + 5_000_000)
        app.worker.join(5)
        assert app.worker.failure is None
        with sqlite3.connect(database) as db:
            assert db.execute('SELECT count(*) FROM clock_observations').fetchone() == (1,)
            assert db.execute('SELECT error_domain_id, error_code_id FROM diagnostics').fetchall() == [
                (E.DiagnosticErrorDomain.PERSISTENCE_CONTROL.value, E.PersistenceControlDiagnosticErrorCode.STATE_MISSING.value)]
    finally:
        app.radio.shutdown()
        app.worker.finish_test()


def test_invalid_configuration_never_opens_radio(tmp_path):
    app, io, _, config = make_application(tmp_path)
    config.chmod(0o644)
    try:
        result = app.start()
        assert not result.ready
        assert result.failure == 'CONFIGURATION_REJECTED'
        assert not io.calls
        assert app.runtime is None
    finally:
        app.worker.finish_test()
