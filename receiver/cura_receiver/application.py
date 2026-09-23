"""Production startup composition; configuration and SQLite stay in the worker."""

from dataclasses import dataclass
from threading import Event

from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.hkdf import HKDF

from .communicator import Communicator
from .communicator_runtime import CommunicatorRuntime
from .communicator_scheduler import CommunicatorScheduler
from .communicator_state_owner import CommunicatorStateOwner
from .control_diagnostics import ControlEpisodeTracker, control_failure
from .elapsed_duration import checked_monotonic_deadline
from .generated import receiver_enums_generated as E
from .producer_admission import ProducerAdmission
from .protocol_ingress import ProtocolIngress
from .receiver_configuration import ReceiverConfigurationLoadStatus
from .runtime_time import RuntimeTime
from .tx_airtime import TxAirtimePolicy

_KEY_INFO = b'cura-agrorum/protocol-v2-lora/aes-128-ccm/node-key/v1'


def authentication_keys(configuration):
    """Protocol-owned HKDF parameters; only the active allowlist can authenticate."""
    return {node: HKDF(algorithm=hashes.SHA256(), length=16, salt=node, info=_KEY_INFO)
            .derive(configuration.group_master_key) for node in configuration.active_node_ids}


@dataclass(frozen=True, slots=True)
class ApplicationStartResult:
    ready: bool
    failure: str | None = None


class ReceiverApplication:
    def __init__(self, *, instance, settings, worker, clock, kernel, rtc, chrony, radio):
        self.instance, self.settings, self.worker = instance, settings, worker
        self.clock, self.kernel, self.rtc, self.chrony, self.radio = clock, kernel, rtc, chrony, radio
        self.admission = ProducerAdmission(worker.queue)
        self.stop_event = Event()
        self.stop_deadline = None
        self.runtime = None
        self.start_result = None
        self._started = False
        self._radio_started = False
        self.shutdown_result = None

    def request_stop(self):
        """Signal-handler-safe intent; radio and disk work remain on their owners."""
        if self.stop_deadline is None:
            self.stop_deadline = checked_monotonic_deadline(
                self.clock.now_monotonic_us(), self.settings.shutdown_budget_us)
        self.stop_event.set()
        self.radio.request_shutdown()

    def _deadline(self, budget):
        value = checked_monotonic_deadline(self.clock.now_monotonic_us(), budget)
        return value if self.stop_deadline is None else min(value, self.stop_deadline)

    def start(self):
        if self._started:
            raise RuntimeError('application startup is single-use')
        self._started = True
        self.worker.start()
        observed = self.clock.now_monotonic_us()
        startup = self.worker.wait_started(deadline_monotonic_us=self._deadline(
            self.settings.time_settings.control_budget_us))
        if self.stop_event.is_set():
            self.start_result = ApplicationStartResult(False, 'STOP_REQUESTED')
            return self.start_result
        if startup is None:
            self.start_result = ApplicationStartResult(False, 'PERSISTENCE_STARTUP_INCOMPLETE')
            return self.start_result
        if startup.configuration_load.status is not ReceiverConfigurationLoadStatus.LOADED:
            # No established lifecycle row / ordinary admission; diagnostics cannot
            # pass the initial clock boundary. Keep only bounded startup evidence.
            self.start_result = ApplicationStartResult(False, startup.configuration_load.status.name)
            return self.start_result
        if startup.database_failure is not None or startup.state_load is None:
            self.start_result = ApplicationStartResult(False, 'PERSISTENCE_STARTUP_FAILED')
            return self.start_result
        tracker = ControlEpisodeTracker()
        owner = CommunicatorStateOwner.from_load(control=self.worker.control,
            loaded=startup.state_load, clock=self.clock, observer=tracker)
        rtc_result = self.rtc.read_time(deadline_monotonic_us=self._deadline(
            self.settings.time_settings.rtc_read_budget_us))
        time = RuntimeTime(receiver_instance_id=self.instance.receiver_instance_id,
            clock=self.clock, kernel=self.kernel, queue=self.admission,
            policy=self.settings.time_policy, startup_rtc_result=rtc_result,
            state_owner=owner, settings=self.settings.time_settings)
        airtime = TxAirtimePolicy(state_owner=owner, clock=self.clock,
            policy=self.settings.airtime_policy,
            rate_bound_ppm=self.settings.time_policy.monotonic_elapsed_rate_bound_ppm)
        keys = authentication_keys(startup.configuration_load.configuration)
        ingress = ProtocolIngress(queue=self.admission, monotonic_clock=self.clock, auth_node_keys=keys)
        communicator = Communicator(instance_id=self.instance.receiver_instance_id,
            clock=self.clock, radio=self.radio, ingress=ingress, runtime_time=time,
            airtime=airtime, queue=self.admission)
        scheduler = CommunicatorScheduler(communicator, chrony=self.chrony, rtc=self.rtc,
            health_interval_us=self.settings.health_interval_us, initial_health_pending=True,
            stop_requested=self.stop_event.is_set, shutdown_deadline=lambda: self.stop_deadline)
        runtime = self.runtime = CommunicatorRuntime(scheduler, control_episodes=tracker)
        initial = time.observe_rtc(self.rtc, startup=True)
        if initial.failure is not None:
            runtime._pending.append((runtime.telemetry.time, initial.failure))
        state_failure = control_failure(startup.state_load,
            command=E.PersistenceControlCommand.LOAD_COMMUNICATOR_STATE,
            purpose=E.PersistenceControlPurpose.STARTUP_STATE,
            started=observed, finished=self.clock.now_monotonic_us())
        if state_failure is not None:
            runtime._pending.append((runtime.telemetry.control, state_failure))
        if self.stop_event.is_set():
            self.start_result = ApplicationStartResult(False, 'STOP_REQUESTED')
            return self.start_result
        self._radio_started = True
        result = self.radio.initialize()
        runtime._pending.extend((runtime.telemetry.radio, episode) for episode in result.episodes)
        if result.state is not E.RadioState.RX_SINGLE:
            runtime.flush_diagnostics()
            self.start_result = ApplicationStartResult(False, result.state.name)
            return self.start_result
        airtime.confirm_transmitter_disabled()
        runtime.flush_diagnostics()
        self.start_result = ApplicationStartResult(True)
        return self.start_result

    def run(self):
        """Stop-aware loop; every device/control action already has its own bound."""
        from .communicator_scheduler import Work
        if self.start_result is None or not self.start_result.ready:
            raise RuntimeError('application must start successfully before running')
        while not self.stop_event.is_set() and not self.runtime.terminal:
            if not self.worker.is_alive():
                self.runtime._terminate(RuntimeError('persistence worker terminated'))
                break
            turn = self.runtime.step()
            if turn is not None and turn.work is Work.WAIT:
                remaining = max(0, turn.wait_until_monotonic_us - self.clock.now_monotonic_us())
                self.stop_event.wait(remaining / 1_000_000)
        return 1 if self.runtime.failure is not None or (self.runtime.terminal and not self.stop_event.is_set()) else 0

    def shutdown(self, *, clean_requested=False, wait=None):
        """One bounded shutdown; exact marker retries never reopen diagnostics."""
        from .application_shutdown import shutdown_application
        if self.shutdown_result is None:
            self.shutdown_result = shutdown_application(self, clean_requested=clean_requested, wait=wait)
        return self.shutdown_result
