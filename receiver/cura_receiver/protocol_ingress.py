"""Radio-independent protocol-v2 ingress and queue-admission boundary."""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping

from .generated import protocol_v2_lora_generated as protocol
from .generated.receiver_entities_generated import MessageProfilingV1
from .generated.receiver_enums_generated import (
    AckSelection,
    AckTxResult,
    AdmissionResult,
    PersistQueueEntityKind,
    ProcessingResult,
)
from .persist_queue import PersistQueue, PersistQueueReservation
from .persist_queue_entities import (
    MEASUREMENT_PROFILE_V1_SPEC,
    PROFILE_ONLY_V1_SPEC,
    AuthenticatedReadingCandidateV1,
    MeasurementProfileUnitV1,
    ProfileOnlyUnitV1,
)
from .ports.clocks import MonotonicClock
from .protocol_v2_lora_crypto import AuthenticationError, open_frame, seal_frame


_U8_MAX = (1 << 8) - 1
_U16_MAX = (1 << 16) - 1
_U32_MAX = (1 << 32) - 1
_U64_MAX = (1 << 64) - 1
_I16_MIN = -(1 << 15)
_I16_MAX = (1 << 15) - 1
_PROFILE_FRAME_CAPACITY = 255


class ProtocolIngressError(Exception):
    """Base class for protocol-ingress configuration and interface errors."""


class ProtocolIngressConfigurationError(ProtocolIngressError, ValueError):
    """The immutable startup boundary is not valid for protocol ingress."""


class ProtocolIngressInterfaceError(ProtocolIngressError, RuntimeError):
    """A caller supplied invalid occurrence or terminal radio facts."""


def _require_int_range(
    value: object,
    name: str,
    minimum: int,
    maximum: int,
    *,
    error_type: type[ProtocolIngressError] = ProtocolIngressInterfaceError,
) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise error_type(f"{name} must be an integer in {minimum}..{maximum}")
    return value


def _require_optional_int_range(
    value: object,
    name: str,
    minimum: int,
    maximum: int,
) -> int | None:
    if value is None:
        return None
    return _require_int_range(value, name, minimum, maximum)


def _require_exact_bytes(
    value: object,
    name: str,
    length: int,
    *,
    error_type: type[ProtocolIngressError] = ProtocolIngressInterfaceError,
) -> bytes:
    if type(value) is not bytes or len(value) != length:
        raise error_type(f"{name} must contain exactly {length} bytes")
    return value


@dataclass(frozen=True, slots=True)
class ProtocolIngressPacketV1:
    """Complete Pi-owned packet facts captured before protocol processing."""

    receiver_instance_id: bytes
    occurrence_sequence: int
    received_at_monotonic_us: int
    frame: bytes
    rssi_dbm_x2: int | None
    snr_db_x4: int | None
    irq_status: int | None
    device_errors: int | None
    busy_wait_total_us: int
    busy_wait_max_us: int
    busy_wait_count: int
    busy_timeout_count: int
    last_busy_timeout_opcode: int | None
    t1_handler_started_monotonic_us: int
    t2_packet_copied_monotonic_us: int

    def __post_init__(self) -> None:
        _require_exact_bytes(
            self.receiver_instance_id,
            "receiver_instance_id",
            16,
        )
        _require_int_range(
            self.occurrence_sequence,
            "occurrence_sequence",
            0,
            _U64_MAX,
        )
        _require_int_range(
            self.received_at_monotonic_us,
            "received_at_monotonic_us",
            0,
            _U64_MAX,
        )
        if type(self.frame) is not bytes or len(self.frame) > _PROFILE_FRAME_CAPACITY:
            raise ProtocolIngressInterfaceError(
                "frame must be bytes with length in 0..255"
            )
        _require_optional_int_range(
            self.rssi_dbm_x2,
            "rssi_dbm_x2",
            _I16_MIN,
            _I16_MAX,
        )
        _require_optional_int_range(
            self.snr_db_x4,
            "snr_db_x4",
            _I16_MIN,
            _I16_MAX,
        )
        if (self.rssi_dbm_x2 is None) != (self.snr_db_x4 is None):
            raise ProtocolIngressInterfaceError(
                "rssi_dbm_x2 and snr_db_x4 must be present together"
            )
        _require_optional_int_range(
            self.irq_status,
            "irq_status",
            0,
            _U16_MAX,
        )
        _require_optional_int_range(
            self.device_errors,
            "device_errors",
            0,
            _U16_MAX,
        )
        _require_int_range(
            self.busy_wait_total_us,
            "busy_wait_total_us",
            0,
            _U64_MAX,
        )
        _require_int_range(
            self.busy_wait_max_us,
            "busy_wait_max_us",
            0,
            _U64_MAX,
        )
        _require_int_range(
            self.busy_wait_count,
            "busy_wait_count",
            0,
            _U32_MAX,
        )
        _require_int_range(
            self.busy_timeout_count,
            "busy_timeout_count",
            0,
            _U32_MAX,
        )
        _require_optional_int_range(
            self.last_busy_timeout_opcode,
            "last_busy_timeout_opcode",
            0,
            _U8_MAX,
        )
        _require_int_range(
            self.t1_handler_started_monotonic_us,
            "t1_handler_started_monotonic_us",
            0,
            _U64_MAX,
        )
        _require_int_range(
            self.t2_packet_copied_monotonic_us,
            "t2_packet_copied_monotonic_us",
            0,
            _U64_MAX,
        )
        if not (
            self.received_at_monotonic_us
            <= self.t1_handler_started_monotonic_us
            <= self.t2_packet_copied_monotonic_us
        ):
            raise ProtocolIngressInterfaceError(
                "packet timestamps must satisfy T0 <= T1 <= T2"
            )
        if self.busy_wait_max_us > self.busy_wait_total_us:
            raise ProtocolIngressInterfaceError(
                "busy_wait_max_us cannot exceed busy_wait_total_us"
            )
        if self.busy_timeout_count > self.busy_wait_count:
            raise ProtocolIngressInterfaceError(
                "busy_timeout_count cannot exceed busy_wait_count"
            )
        if (self.busy_timeout_count == 0) != (
            self.last_busy_timeout_opcode is None
        ):
            raise ProtocolIngressInterfaceError(
                "last_busy_timeout_opcode is present exactly when a timeout occurred"
            )


@dataclass(frozen=True, slots=True)
class ProtocolIngressPreTxProfileV1:
    """Stable profile fields fixed before the terminal radio outcome."""

    receiver_instance_id: bytes
    occurrence_sequence: int
    received_at_monotonic_us: int
    received_frame_length: int
    received_frame: bytes
    claimed_control: int | None
    claimed_domain: int | None
    claimed_node_id: bytes | None
    claimed_message_id: int | None
    header_authenticated: bool
    decoded_sample_id: int | None
    rssi_dbm_x2: int | None
    snr_db_x4: int | None
    irq_status: int | None
    device_errors: int | None
    processing_result: ProcessingResult
    ack_selected: AckSelection
    ack_frame: bytes | None
    busy_wait_total_us: int
    busy_wait_max_us: int
    busy_wait_count: int
    busy_timeout_count: int
    last_busy_timeout_opcode: int | None
    t1_handler_started_monotonic_us: int
    t2_packet_copied_monotonic_us: int
    t3_authentication_completed_monotonic_us: int | None

    def __post_init__(self) -> None:
        _require_exact_bytes(self.receiver_instance_id, "receiver_instance_id", 16)
        _require_int_range(
            self.occurrence_sequence,
            "occurrence_sequence",
            0,
            _U64_MAX,
        )
        _require_int_range(
            self.received_at_monotonic_us,
            "received_at_monotonic_us",
            0,
            _U64_MAX,
        )
        _require_int_range(
            self.received_frame_length,
            "received_frame_length",
            0,
            _PROFILE_FRAME_CAPACITY,
        )
        _require_exact_bytes(
            self.received_frame,
            "received_frame",
            _PROFILE_FRAME_CAPACITY,
        )
        if any(self.received_frame[self.received_frame_length :]):
            raise ProtocolIngressInterfaceError(
                "received_frame tail after received_frame_length must be zero"
            )
        _require_optional_int_range(
            self.claimed_control,
            "claimed_control",
            0,
            _U8_MAX,
        )
        _require_optional_int_range(
            self.claimed_domain,
            "claimed_domain",
            0,
            _U8_MAX,
        )
        if self.claimed_node_id is not None:
            _require_exact_bytes(self.claimed_node_id, "claimed_node_id", 8)
        _require_optional_int_range(
            self.claimed_message_id,
            "claimed_message_id",
            0,
            _U32_MAX,
        )
        if type(self.header_authenticated) is not bool:
            raise ProtocolIngressInterfaceError(
                "header_authenticated must be a bool"
            )
        _require_optional_int_range(
            self.decoded_sample_id,
            "decoded_sample_id",
            0,
            _U32_MAX,
        )
        if type(self.processing_result) is not ProcessingResult:
            raise ProtocolIngressInterfaceError(
                "processing_result must be a ProcessingResult"
            )
        if type(self.ack_selected) is not AckSelection:
            raise ProtocolIngressInterfaceError(
                "ack_selected must be an AckSelection"
            )
        if (self.ack_selected is AckSelection.NONE) != (self.ack_frame is None):
            raise ProtocolIngressInterfaceError(
                "ack_frame is present exactly when an ACK is selected"
            )
        if self.ack_frame is not None:
            _require_exact_bytes(
                self.ack_frame,
                "ack_frame",
                protocol.ACK_FRAME_SIZE,
            )
        if self.header_authenticated:
            if any(
                value is None
                for value in (
                    self.claimed_control,
                    self.claimed_domain,
                    self.claimed_node_id,
                    self.claimed_message_id,
                    self.t3_authentication_completed_monotonic_us,
                )
            ):
                raise ProtocolIngressInterfaceError(
                    "authenticated profile must contain the complete header and T3"
                )
        elif self.decoded_sample_id is not None:
            raise ProtocolIngressInterfaceError(
                "decoded_sample_id requires an authenticated header"
            )


@dataclass(frozen=True, slots=True)
class ProtocolIngressAdmissionV1:
    """The one health-counter label produced by a queue admission attempt."""

    entity_kind: PersistQueueEntityKind
    result: AdmissionResult

    def __post_init__(self) -> None:
        if type(self.entity_kind) is not PersistQueueEntityKind:
            raise ProtocolIngressInterfaceError(
                "entity_kind must be a PersistQueueEntityKind"
            )
        if type(self.result) is not AdmissionResult:
            raise ProtocolIngressInterfaceError(
                "result must be an AdmissionResult"
            )


@dataclass(frozen=True, slots=True)
class ProtocolIngressTerminalV1:
    """Radio-owned terminal facts supplied after ACK TX or receive rearm."""

    ack_tx_result: AckTxResult
    t4_set_tx_attempted_monotonic_us: int | None
    t5_tx_done_monotonic_us: int | None
    t6_set_rx_issued_monotonic_us: int | None

    def __post_init__(self) -> None:
        if type(self.ack_tx_result) is not AckTxResult:
            raise ProtocolIngressInterfaceError(
                "ack_tx_result must be an AckTxResult"
            )
        for name, value in (
            (
                "t4_set_tx_attempted_monotonic_us",
                self.t4_set_tx_attempted_monotonic_us,
            ),
            ("t5_tx_done_monotonic_us", self.t5_tx_done_monotonic_us),
            ("t6_set_rx_issued_monotonic_us", self.t6_set_rx_issued_monotonic_us),
        ):
            _require_optional_int_range(value, name, 0, _U64_MAX)


class _ProtocolIngressOccurrenceToken:
    __slots__ = ("active", "owner", "reservation")

    def __init__(
        self,
        owner: ProtocolIngress,
        reservation: PersistQueueReservation | None,
    ) -> None:
        self.active = True
        self.owner = owner
        self.reservation = reservation


@dataclass(frozen=True, slots=True)
class ProtocolIngressDecisionV1:
    """Immutable protocol/admission decision awaiting terminal radio facts."""

    pre_tx_profile: ProtocolIngressPreTxProfileV1
    candidate: AuthenticatedReadingCandidateV1 | None
    admission: ProtocolIngressAdmissionV1 | None
    _token: _ProtocolIngressOccurrenceToken = field(repr=False, compare=False)


@dataclass(frozen=True, slots=True)
class ProtocolIngressFinalizationV1:
    """Publication result, including any admission deferred until rearm."""

    admission: ProtocolIngressAdmissionV1 | None
    published_entity: MeasurementProfileUnitV1 | ProfileOnlyUnitV1 | None


class ProtocolIngress:
    """Own ordered protocol processing between copied bytes and radio actions."""

    def __init__(
        self,
        *,
        queue: PersistQueue,
        monotonic_clock: MonotonicClock,
        auth_node_keys: Mapping[bytes, bytes],
    ) -> None:
        if type(queue) is not PersistQueue:
            raise ProtocolIngressConfigurationError(
                "queue must be the production PersistQueue"
            )
        if not isinstance(monotonic_clock, MonotonicClock):
            raise ProtocolIngressConfigurationError(
                "monotonic_clock must implement MonotonicClock"
            )
        if not isinstance(auth_node_keys, Mapping):
            raise ProtocolIngressConfigurationError(
                "auth_node_keys must be a mapping"
            )

        copied_keys: dict[bytes, bytes] = {}
        for node_id, node_key in auth_node_keys.items():
            _require_exact_bytes(
                node_id,
                "auth_node_keys node_id",
                8,
                error_type=ProtocolIngressConfigurationError,
            )
            _require_exact_bytes(
                node_key,
                "auth_node_keys node_key",
                16,
                error_type=ProtocolIngressConfigurationError,
            )
            copied_keys[node_id] = node_key

        self._queue = queue
        self._monotonic_clock = monotonic_clock
        self._auth_node_keys = MappingProxyType(copied_keys)

    def begin(self, packet: ProtocolIngressPacketV1) -> ProtocolIngressDecisionV1:
        """Validate one copied frame and make its pre-radio admission decision."""

        if type(packet) is not ProtocolIngressPacketV1:
            raise ProtocolIngressInterfaceError(
                "packet must be a ProtocolIngressPacketV1"
            )

        frame = packet.frame
        claimed_control = frame[0] if len(frame) >= 1 else None
        claimed_domain = frame[1] if len(frame) >= 2 else None
        claimed_node_id = frame[2:10] if len(frame) >= 10 else None
        claimed_message_id = (
            int.from_bytes(frame[10:14], "little") if len(frame) >= 14 else None
        )
        header_authenticated = False
        decoded_sample_id: int | None = None
        t3: int | None = None
        candidate: AuthenticatedReadingCandidateV1 | None = None
        node_key: bytes | None = None

        if not (
            self._minimum_authenticatable_frame_size()
            <= len(frame)
            <= protocol.READING_FRAME_SIZE
        ):
            processing_result = ProcessingResult.REJECTED_MALFORMED_LENGTH
            ack_selected = AckSelection.NONE
        elif claimed_node_id not in self._auth_node_keys:
            processing_result = ProcessingResult.UNKNOWN_NODE
            ack_selected = AckSelection.NONE
        else:
            assert claimed_node_id is not None
            node_key = self._auth_node_keys[claimed_node_id]
            try:
                authenticated = open_frame(node_key, frame)
            except AuthenticationError:
                t3 = self._authentication_completed_at(packet)
                processing_result = ProcessingResult.AUTHENTICATION_FAILED
                ack_selected = AckSelection.NONE
            else:
                t3 = self._authentication_completed_at(packet)
                header_authenticated = True
                header = authenticated.header
                if not protocol.is_supported_control(header.control):
                    processing_result = (
                        ProcessingResult.REJECTED_UNSUPPORTED_CONTROL
                    )
                    ack_selected = AckSelection.REJECTED_UNSUPPORTED
                elif protocol.domain_is_ack(header.domain):
                    processing_result = ProcessingResult.WRONG_DIRECTION
                    ack_selected = AckSelection.NONE
                elif not protocol.domain_is_reading(header.domain):
                    processing_result = ProcessingResult.REJECTED_UNSUPPORTED_DOMAIN
                    ack_selected = AckSelection.REJECTED_UNSUPPORTED
                elif len(authenticated.plaintext_body) != protocol.READING_BODY_SIZE:
                    processing_result = ProcessingResult.REJECTED_MALFORMED_LENGTH
                    ack_selected = AckSelection.REJECTED_MALFORMED
                else:
                    try:
                        reading = protocol.decode_reading(
                            authenticated.plaintext_body
                        )
                    except protocol.CodecError:
                        processing_result = ProcessingResult.REJECTED_MALFORMED_BODY
                        ack_selected = AckSelection.REJECTED_MALFORMED
                    else:
                        decoded_sample_id = reading.sample_id
                        processing_result = ProcessingResult.ACCEPTED
                        ack_selected = AckSelection.ACCEPTED
                        candidate = AuthenticatedReadingCandidateV1(
                            node_id=header.node_id,
                            message_id=header.message_id,
                            domain=header.domain,
                            sample_id=reading.sample_id,
                            reading_body=authenticated.plaintext_body,
                        )

        ack_frame = self._build_ack(
            node_key=node_key,
            claimed_node_id=claimed_node_id,
            claimed_message_id=claimed_message_id,
            ack_selected=ack_selected,
        )
        admission: ProtocolIngressAdmissionV1 | None = None
        reservation: PersistQueueReservation | None = None

        if ack_selected is not AckSelection.NONE:
            requested_spec = (
                MEASUREMENT_PROFILE_V1_SPEC
                if processing_result is ProcessingResult.ACCEPTED
                else PROFILE_ONLY_V1_SPEC
            )
            reserve_result = self._queue.try_reserve_one(requested_spec)
            admission = ProtocolIngressAdmissionV1(
                entity_kind=requested_spec.kind,
                result=reserve_result.status,
            )
            if reserve_result.status is AdmissionResult.RESERVED:
                if reserve_result.reservation is None:
                    raise ProtocolIngressInterfaceError(
                        "reserved queue result omitted its reservation"
                    )
                reservation = reserve_result.reservation
            else:
                if reserve_result.reservation is not None:
                    raise ProtocolIngressInterfaceError(
                        "failed queue admission returned a reservation"
                    )
                processing_result = self._retry_processing_result(
                    reserve_result.status
                )
                ack_selected = AckSelection.RETRY_LATER
                ack_frame = self._build_ack(
                    node_key=node_key,
                    claimed_node_id=claimed_node_id,
                    claimed_message_id=claimed_message_id,
                    ack_selected=ack_selected,
                )
                candidate = None

        pre_tx_profile = ProtocolIngressPreTxProfileV1(
            receiver_instance_id=packet.receiver_instance_id,
            occurrence_sequence=packet.occurrence_sequence,
            received_at_monotonic_us=packet.received_at_monotonic_us,
            received_frame_length=len(frame),
            received_frame=frame + bytes(_PROFILE_FRAME_CAPACITY - len(frame)),
            claimed_control=claimed_control,
            claimed_domain=claimed_domain,
            claimed_node_id=claimed_node_id,
            claimed_message_id=claimed_message_id,
            header_authenticated=header_authenticated,
            decoded_sample_id=decoded_sample_id,
            rssi_dbm_x2=packet.rssi_dbm_x2,
            snr_db_x4=packet.snr_db_x4,
            irq_status=packet.irq_status,
            device_errors=packet.device_errors,
            processing_result=processing_result,
            ack_selected=ack_selected,
            ack_frame=ack_frame,
            busy_wait_total_us=packet.busy_wait_total_us,
            busy_wait_max_us=packet.busy_wait_max_us,
            busy_wait_count=packet.busy_wait_count,
            busy_timeout_count=packet.busy_timeout_count,
            last_busy_timeout_opcode=packet.last_busy_timeout_opcode,
            t1_handler_started_monotonic_us=(
                packet.t1_handler_started_monotonic_us
            ),
            t2_packet_copied_monotonic_us=(
                packet.t2_packet_copied_monotonic_us
            ),
            t3_authentication_completed_monotonic_us=t3,
        )
        return ProtocolIngressDecisionV1(
            pre_tx_profile=pre_tx_profile,
            candidate=candidate,
            admission=admission,
            _token=_ProtocolIngressOccurrenceToken(self, reservation),
        )

    def finalize(
        self,
        decision: ProtocolIngressDecisionV1,
        terminal: ProtocolIngressTerminalV1,
    ) -> ProtocolIngressFinalizationV1:
        """Publish the terminal occurrence, admitting silent profiles after rearm."""

        if type(decision) is not ProtocolIngressDecisionV1:
            raise ProtocolIngressInterfaceError(
                "decision must be a ProtocolIngressDecisionV1"
            )
        if type(terminal) is not ProtocolIngressTerminalV1:
            raise ProtocolIngressInterfaceError(
                "terminal must be a ProtocolIngressTerminalV1"
            )
        token = decision._token
        if token.owner is not self:
            raise ProtocolIngressInterfaceError("foreign protocol-ingress decision")
        if not token.active:
            raise ProtocolIngressInterfaceError("stale protocol-ingress decision")
        self._validate_terminal(decision.pre_tx_profile, terminal)

        profile = self._complete_profile(decision.pre_tx_profile, terminal)
        admission: ProtocolIngressAdmissionV1 | None = None
        published_entity: MeasurementProfileUnitV1 | ProfileOnlyUnitV1 | None = None

        if decision.admission is None:
            reserve_result = self._queue.try_reserve_one(PROFILE_ONLY_V1_SPEC)
            admission = ProtocolIngressAdmissionV1(
                entity_kind=PersistQueueEntityKind.PROFILE_ONLY,
                result=reserve_result.status,
            )
            token.active = False
            if reserve_result.status is AdmissionResult.RESERVED:
                if reserve_result.reservation is None:
                    raise ProtocolIngressInterfaceError(
                        "reserved queue result omitted its reservation"
                    )
                published_entity = ProfileOnlyUnitV1(profile=profile)
                reserve_result.reservation.publish(published_entity)
            elif reserve_result.reservation is not None:
                raise ProtocolIngressInterfaceError(
                    "failed queue admission returned a reservation"
                )
        elif decision.admission.result is AdmissionResult.RESERVED:
            reservation = token.reservation
            if reservation is None:
                raise ProtocolIngressInterfaceError(
                    "reserved ingress decision lost its queue reservation"
                )
            if decision.candidate is None:
                published_entity = ProfileOnlyUnitV1(profile=profile)
            else:
                published_entity = MeasurementProfileUnitV1(
                    candidate=decision.candidate,
                    profile=profile,
                )
            token.active = False
            reservation.publish(published_entity)
        else:
            if token.reservation is not None:
                raise ProtocolIngressInterfaceError(
                    "failed ingress admission retained a reservation"
                )
            token.active = False

        return ProtocolIngressFinalizationV1(
            admission=admission,
            published_entity=published_entity,
        )

    @staticmethod
    def _minimum_authenticatable_frame_size() -> int:
        return protocol.CLEAR_HEADER_SIZE + protocol.ACK_BODY_SIZE + protocol.TAG_SIZE

    def _authentication_completed_at(
        self,
        packet: ProtocolIngressPacketV1,
    ) -> int:
        value = self._monotonic_clock.now_monotonic_us()
        _require_int_range(
            value,
            "authentication-completion monotonic time",
            packet.t2_packet_copied_monotonic_us,
            _U64_MAX,
        )
        return value

    @staticmethod
    def _build_ack(
        *,
        node_key: bytes | None,
        claimed_node_id: bytes | None,
        claimed_message_id: int | None,
        ack_selected: AckSelection,
    ) -> bytes | None:
        if ack_selected is AckSelection.NONE:
            return None
        if (
            node_key is None
            or claimed_node_id is None
            or claimed_message_id is None
        ):
            raise ProtocolIngressInterfaceError(
                "ACK selection requires authenticated key and transport identity"
            )
        try:
            status = protocol.ACK_STATUS_BY_DOMAIN[
                protocol.Domain(ack_selected.value)
            ]
        except (KeyError, ValueError) as exc:
            raise ProtocolIngressInterfaceError(
                "ACK selection has no protocol domain/status mapping"
            ) from exc
        return seal_frame(
            node_key,
            protocol.ClearHeader(
                control=protocol.CONTROL,
                domain=ack_selected.value,
                node_id=claimed_node_id,
                message_id=claimed_message_id,
            ),
            protocol.encode_ack(protocol.Ack(status=status.value)),
        )

    @staticmethod
    def _retry_processing_result(status: AdmissionResult) -> ProcessingResult:
        if status is AdmissionResult.PERSISTENCE_UNAVAILABLE:
            return ProcessingResult.RETRY_LATER_PERSISTENCE_UNAVAILABLE
        if status is AdmissionResult.QUEUE_FULL:
            return ProcessingResult.RETRY_LATER_QUEUE_FULL
        raise ProtocolIngressInterfaceError(
            "successful reservation cannot select retry-later"
        )

    @staticmethod
    def _validate_terminal(
        pre_tx: ProtocolIngressPreTxProfileV1,
        terminal: ProtocolIngressTerminalV1,
    ) -> None:
        ack_selected = pre_tx.ack_selected is not AckSelection.NONE
        if ack_selected == (terminal.ack_tx_result is AckTxResult.NOT_APPLICABLE):
            raise ProtocolIngressInterfaceError(
                "ACK selection and terminal ACK result disagree"
            )
        t4 = terminal.t4_set_tx_attempted_monotonic_us
        t5 = terminal.t5_tx_done_monotonic_us
        t6 = terminal.t6_set_rx_issued_monotonic_us

        if terminal.ack_tx_result in {
            AckTxResult.NOT_APPLICABLE,
            AckTxResult.SUPPRESSED_AIRTIME_BUDGET,
        }:
            if t4 is not None or t5 is not None:
                raise ProtocolIngressInterfaceError(
                    "non-TX terminal result cannot contain T4 or T5"
                )
        elif terminal.ack_tx_result in {
            AckTxResult.SET_TX_FAILED,
            AckTxResult.TX_TIMEOUT,
            AckTxResult.UNKNOWN_INTERRUPTED,
        }:
            if t4 is None or t5 is not None:
                raise ProtocolIngressInterfaceError(
                    "attempted non-TxDone result requires T4 and forbids T5"
                )
        elif terminal.ack_tx_result is AckTxResult.TX_DONE:
            if t4 is None or t5 is None:
                raise ProtocolIngressInterfaceError(
                    "TX_DONE requires both T4 and T5"
                )

        preceding = (
            pre_tx.t3_authentication_completed_monotonic_us
            if pre_tx.t3_authentication_completed_monotonic_us is not None
            else pre_tx.t2_packet_copied_monotonic_us
        )
        for name, timestamp in (
            ("T4", t4),
            ("T5", t5),
            ("T6", t6),
        ):
            if timestamp is not None:
                if timestamp < preceding:
                    raise ProtocolIngressInterfaceError(
                        f"{name} regresses from the preceding event"
                    )
                preceding = timestamp

    @staticmethod
    def _complete_profile(
        pre_tx: ProtocolIngressPreTxProfileV1,
        terminal: ProtocolIngressTerminalV1,
    ) -> MessageProfilingV1:
        return MessageProfilingV1(
            receiver_instance_id=pre_tx.receiver_instance_id,
            occurrence_sequence=pre_tx.occurrence_sequence,
            received_at_monotonic_us=pre_tx.received_at_monotonic_us,
            received_frame_length=pre_tx.received_frame_length,
            received_frame=pre_tx.received_frame,
            claimed_control=pre_tx.claimed_control,
            claimed_domain=pre_tx.claimed_domain,
            claimed_node_id=pre_tx.claimed_node_id,
            claimed_message_id=pre_tx.claimed_message_id,
            header_authenticated=pre_tx.header_authenticated,
            decoded_sample_id=pre_tx.decoded_sample_id,
            rssi_dbm_x2=pre_tx.rssi_dbm_x2,
            snr_db_x4=pre_tx.snr_db_x4,
            irq_status=pre_tx.irq_status,
            device_errors=pre_tx.device_errors,
            processing_result=pre_tx.processing_result,
            ack_selected=pre_tx.ack_selected,
            ack_tx_result=terminal.ack_tx_result,
            ack_frame=pre_tx.ack_frame,
            busy_wait_total_us=pre_tx.busy_wait_total_us,
            busy_wait_max_us=pre_tx.busy_wait_max_us,
            busy_wait_count=pre_tx.busy_wait_count,
            busy_timeout_count=pre_tx.busy_timeout_count,
            last_busy_timeout_opcode=pre_tx.last_busy_timeout_opcode,
            t1_handler_started_monotonic_us=(
                pre_tx.t1_handler_started_monotonic_us
            ),
            t2_packet_copied_monotonic_us=(
                pre_tx.t2_packet_copied_monotonic_us
            ),
            t3_authentication_completed_monotonic_us=(
                pre_tx.t3_authentication_completed_monotonic_us
            ),
            t4_set_tx_attempted_monotonic_us=(
                terminal.t4_set_tx_attempted_monotonic_us
            ),
            t5_tx_done_monotonic_us=terminal.t5_tx_done_monotonic_us,
            t6_set_rx_issued_monotonic_us=terminal.t6_set_rx_issued_monotonic_us,
        )


__all__ = [
    "ProtocolIngressError",
    "ProtocolIngressConfigurationError",
    "ProtocolIngressInterfaceError",
    "ProtocolIngressPacketV1",
    "ProtocolIngressPreTxProfileV1",
    "ProtocolIngressAdmissionV1",
    "ProtocolIngressTerminalV1",
    "ProtocolIngressDecisionV1",
    "ProtocolIngressFinalizationV1",
    "ProtocolIngress",
]
