"""Single-owner packet composition over production radio, ingress, time and airtime.

No disk access, control-channel wait or ordinary persistence wait occurs in an
exchange. Periodic work and lifecycle composition run at its safe boundaries.
"""

from collections import deque
from dataclasses import dataclass

from .elapsed_duration import checked_monotonic_deadline
from .generated import receiver_enums_generated as E
from .ports.radio import RadioTxAuthorization, Outcome
from .radio import ReceiveDisposition
from .protocol_ingress import ProtocolIngressPacketV1, ProtocolIngressTerminalV1
from .generated.receiver_entities_generated import RadioObservationV1
from .tx_airtime import TxCertainty
from .producer_admission import ProducerAdmission
from .persist_queue import PersistQueueInterfaceError


_RECEIVING = (E.RadioState.RX_SINGLE, E.RadioState.RX_EVENT_PENDING)


@dataclass(frozen=True, slots=True)
class ExchangeResult:
    radio_result: object | None = None
    finalization: object | None = None
    radio_episodes: tuple = ()
    time_updates: tuple = ()
    boundary_blocked: bool = False
    discarded_unaccepted_packet: bool = False


class CommunicatorFailure(RuntimeError):
    """Fatal composition failure; packet publication has already been attempted.

    No arbitrary exception text is copied into a diagnostic. The caller may
    inspect the original exception for the closed CORE catalogue mapping.
    """

    def __init__(self, result, original, cleanup_error=None, *, observed_at=None, location=None, completed_episodes=()):
        super().__init__("communicator exchange terminated")
        self.result = result
        self.original = original
        self.cleanup_error = cleanup_error
        self.observed_at = observed_at
        self.location = location
        self.completed_episodes = completed_episodes


class Communicator:
    def __init__(self, *, instance_id, clock, radio, ingress, runtime_time, airtime, queue):
        if type(queue) is not ProducerAdmission:
            raise TypeError("communicator requires ProducerAdmission")
        if ingress.admission is not queue or runtime_time.queue is not queue:
            raise ValueError("all communicator producers must share one admission adapter")
        self.queue = queue
        self._failed_receive = None
        self.instance_id = instance_id
        self.clock = clock
        self.radio = radio
        self.observation_sequence = self.observations_dropped = 0
        self.pending_radio_observations = deque(maxlen=16)
        self.radio.observation_sink = self.retain_radio_observation
        self.ingress = ingress
        self.time = runtime_time
        self.airtime = airtime
        self.occurrence_sequence = 0
        self._active = None
        self._spend = None
        self._result = None
        self._episodes = []
        self._fallback = E.AckTxResult.NOT_APPLICABLE
        self.failure_location = (E.CorePhase.PACKET_PROCESSING, E.CoreFailureStage.INVOKE_ADAPTER, E.DiagnosticOperation.RECEIVE)

    def identify_radio_observation(self, snapshot):
        self.observation_sequence = checked_monotonic_deadline(self.observation_sequence, 1)
        return RadioObservationV1(self.instance_id, self.observation_sequence, snapshot)

    def retain_radio_observation(self, snapshot):
        observation = self.identify_radio_observation(snapshot)
        if len(self.pending_radio_observations) == self.pending_radio_observations.maxlen:
            self.observations_dropped += 1
        self.pending_radio_observations.append(observation)
        return observation.observation_sequence

    def _record(self, result):
        self._result = result
        self._episodes.extend(result.episodes)
        return result

    def _recover(self):
        # A stream failure immediately following successful recovery is a new
        # bounded episode. Yield to the caller rather than loop indefinitely.
        if self.radio.state is E.RadioState.RECOVERING:
            self.failure_location = (E.CorePhase.PACKET_PROCESSING, E.CoreFailureStage.INVOKE_ADAPTER, E.DiagnosticOperation.RECOVER)
            self._record(self.radio.recover())
        return self._result

    def _packet(self, packet):
        self.occurrence_sequence = checked_monotonic_deadline(self.occurrence_sequence, 1)
        busy = packet.busy
        return ProtocolIngressPacketV1(
            receiver_instance_id=self.instance_id,
            occurrence_sequence=self.occurrence_sequence,
            received_at_monotonic_us=packet.received_at_monotonic_us,
            frame=packet.frame,
            rssi_dbm_x2=packet.rssi_dbm_x2,
            snr_db_x4=packet.snr_db_x4,
            irq_status=packet.irq_status,
            device_errors=packet.device_errors,
            busy_wait_total_us=busy.total_us,
            busy_wait_max_us=busy.maximum_us,
            busy_wait_count=busy.count,
            busy_timeout_count=busy.timeout_count,
            last_busy_timeout_opcode=busy.last_timeout_opcode,
            t1_handler_started_monotonic_us=packet.t1_handler_started_monotonic_us,
            t2_packet_copied_monotonic_us=packet.t2_packet_copied_monotonic_us,
        )

    def _finish(self):
        self.failure_location = (E.CorePhase.POST_RESPONSE_FINALIZATION, E.CoreFailureStage.PUBLISH_QUEUE, E.DiagnosticOperation.APPEND)
        result = self._result
        tx = result.tx
        if self._spend is not None:
            certainty = TxCertainty.UNCERTAIN
            if tx is not None:
                facts = tx.facts
                if facts.set_tx_outcome is Outcome.DEFINITELY_NOT_APPLIED and not facts.profile_uncertain:
                    certainty = TxCertainty.NOT_STARTED
                elif facts.set_tx_outcome is Outcome.CONFIRMED_APPLIED:
                    certainty = TxCertainty.STARTED
            self.airtime.report_tx(self._spend.token, certainty)
            self._spend = None
        terminal = ProtocolIngressTerminalV1(
            self._fallback if tx is None else tx.ack_tx_result,
            None if tx is None else tx.t4_set_tx_attempted_monotonic_us,
            None if tx is None else tx.t5_tx_done_monotonic_us,
            result.t6_set_rx_issued_monotonic_us,
            radio_state=result.state,
        )
        completed = self.ingress.finalize(self._active, terminal)
        self._active = None
        return completed

    def _finish_failed_receive(self):
        # No authenticated occurrence exists; radio observations own this evidence.
        self._failed_receive = None
        self.time.expire_due()
        return None

    def receive_once(self, *, deadline_monotonic_us):
        """One bounded receive/exchange; pending boundaries never enter ingress.

        The caller handles stop intent and schedules another attempt after a
        blocked result. No sleeping or persistence polling is hidden here.
        """
        self._episodes = []
        updates = []
        try:
            # Complete an existing packet recovery before any new producer.
            if self._active is not None or self._failed_receive is not None:
                self._recover()
                if self.radio.state is E.RadioState.RECOVERING:
                    return ExchangeResult(self._result, radio_episodes=tuple(self._episodes))
                completed = self._finish() if self._active is not None else self._finish_failed_receive()
                return ExchangeResult(self._result, completed, tuple(self._episodes))

            if self.radio.state is E.RadioState.RECOVERING:
                self._recover()
                return ExchangeResult(self._result, radio_episodes=tuple(self._episodes))

            update = self.time.expire_due()
            updates.append(update)
            if self.time.ordinary_admission_blocked:
                if update.admission_result is None:
                    updates.append(self.time.publish_pending())
                if self.time.ordinary_admission_blocked:
                    return ExchangeResult(time_updates=tuple(updates), boundary_blocked=True)

            self.failure_location = (E.CorePhase.PACKET_PROCESSING, E.CoreFailureStage.INVOKE_ADAPTER, E.DiagnosticOperation.RECEIVE)
            result = self._record(self.radio.receive(deadline_monotonic_us=deadline_monotonic_us))
            packet = result.receive_event
            if packet is None:
                self._recover()
                return ExchangeResult(self._result, radio_episodes=tuple(self._episodes), time_updates=tuple(updates))

            if packet.disposition is ReceiveDisposition.HANDLED_NO_PACKET:
                # Radio already confirmed ordinary rearming; this is no packet occurrence.
                updates.append(self.time.expire_due())
                return ExchangeResult(self._result, radio_episodes=tuple(self._episodes), time_updates=tuple(updates))

            # receive() can cross a trust deadline. The boundary must precede
            # this packet's admission, even if the consumer just freed a slot.
            updates.append(self.time.expire_due())
            if self.time.ordinary_admission_blocked:
                if self.radio.state is E.RadioState.RX_EVENT_PENDING:
                    self._record(self.radio.rearm())
                self._recover()
                return ExchangeResult(
                    self._result, radio_episodes=tuple(self._episodes),
                    time_updates=tuple(updates), boundary_blocked=True,
                    discarded_unaccepted_packet=True,
                )

            if packet.disposition is ReceiveDisposition.FAILED:
                self.occurrence_sequence = checked_monotonic_deadline(self.occurrence_sequence, 1)
                self._failed_receive = packet
                self._recover()
                completed = None if self.radio.state is E.RadioState.RECOVERING else self._finish_failed_receive()
                return ExchangeResult(self._result, completed, tuple(self._episodes), tuple(updates))

            self.failure_location = (E.CorePhase.PACKET_PROCESSING, E.CoreFailureStage.VALIDATE_MESSAGE, E.DiagnosticOperation.VALIDATE)
            self._active = self.ingress.begin(self._packet(packet))
            frame = self._active.ack_frame
            self._fallback = E.AckTxResult.NOT_APPLICABLE if frame is None else E.AckTxResult.SET_TX_FAILED
            if self.radio.state is E.RadioState.RECOVERING:
                self._recover()
            elif frame is None:
                self._record(self.radio.rearm())
            else:
                self.airtime.update_time(self.time.airtime_correlation(), rtc_health=self.time.state.rtc_health)
                # Admission is in memory; persistence follows RX rearm.
                if self.airtime.available_charge_us == 0:
                    self._fallback = E.AckTxResult.SUPPRESSED_AIRTIME_BUDGET
                    self._record(self.radio.rearm())
                else:
                    self.failure_location = (E.CorePhase.ACK_PREPARATION, E.CoreFailureStage.INVOKE_ADAPTER, E.DiagnosticOperation.TRANSMIT)
                    self._record(self.radio.prepare_ack(frame, occurrence_sequence=self.occurrence_sequence))
                    if self.radio.state is E.RadioState.RX_EVENT_PENDING and self._result.tx is None:
                        spend = self.airtime.try_spend()
                        if spend.token is None:
                            self._fallback = E.AckTxResult.SUPPRESSED_AIRTIME_BUDGET
                            self._record(self.radio.rearm())
                        else:
                            self._spend = spend
                            self._record(self.radio.start_ack(RadioTxAuthorization(
                                spend.submission_deadline_monotonic_us,
                                self.occurrence_sequence,
                            )))
                            if self.radio.state is E.RadioState.TX_ACTIVE:
                                self._record(self.radio.finish_ack())
            self._recover()
            if self.radio.state is E.RadioState.RECOVERING:
                return ExchangeResult(self._result, radio_episodes=tuple(self._episodes), time_updates=tuple(updates))
            completed = self._finish()
            return ExchangeResult(self._result, completed, tuple(self._episodes), tuple(updates))
        except Exception as original:
            observed_at = self.clock.now_monotonic_us()
            location = self.failure_location
            completed_episodes = tuple(self._episodes)
            # begin may have bound admission before an exception escaped its caller.
            # The ingress owner retains the sole completion handle in that case.
            if self._active is None:
                self._active = self.ingress.active_occurrence
                if self._active is not None:
                    self._fallback = (E.AckTxResult.NOT_APPLICABLE if self._active.ack_frame is None
                                      else E.AckTxResult.SET_TX_FAILED)
            cleanup_error = None
            completed = None
            try:
                self.radio.capture_failure()
                self.radio.request_shutdown()
                self._record(self.radio.shutdown())
                queue_sound = not isinstance(original, PersistQueueInterfaceError) or original.queue_known_sound
                if queue_sound and self._active is not None:
                    completed = self._finish()
                elif queue_sound and self._failed_receive is not None:
                    completed = self._finish_failed_receive()
            except Exception as error:
                cleanup_error = error
            evidence = ExchangeResult(self._result, completed, tuple(self._episodes), tuple(updates))
            raise CommunicatorFailure(evidence, original, cleanup_error, observed_at=observed_at, location=location, completed_episodes=completed_episodes) from original
