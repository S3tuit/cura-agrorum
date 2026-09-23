"""Persistence-owned synchronous configuration work, without a control mailbox."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from threading import current_thread

from cura_protocol_v2_lora.receiver_group import (
    GROUP_STATE_FILENAME,
    ReceiverGroupRejectedError,
    ReceiverGroupRejection,
    ReceiverGroupState,
    load_receiver_group,
)

from .generated.receiver_enums_generated import DiagnosticOperation
from .platform.linux_boot_identity import LINUX_BOOT_ID_PATH, read_linux_boot_id

DEFAULT_CONFIGURATION_PATH = Path(__file__).resolve().parents[1] / GROUP_STATE_FILENAME


class ReceiverConfigurationLoadStatus(Enum):
    LOADED = auto()
    CONFIGURATION_REJECTED = auto()
    HOST_IDENTITY_REJECTED = auto()
    INTERFACE_VIOLATION = auto()
    OS_ERROR = auto()
    DEADLINE_EXCEEDED = auto()
    CHANNEL_CLOSED = auto()


class PersistenceControlInterfaceViolation(Enum):
    NONE = auto()
    INVALID_ARGUMENT = auto()
    INVALID_DEADLINE = auto()
    WRONG_CALLER = auto()
    INVALID_STATE = auto()
    GENERATION_CONTENT_CONFLICT = auto()
    STALE_GENERATION = auto()
    GENERATION_GAP = auto()
    CLEAN_STOP_PRECONDITION = auto()
    CLEAN_STOP_CONFLICT = auto()


@dataclass(frozen=True, slots=True)
class ReceiverConfigurationLoadResult:
    status: ReceiverConfigurationLoadStatus
    operation: DiagnosticOperation
    interface_violation: PersistenceControlInterfaceViolation = (
        PersistenceControlInterfaceViolation.NONE
    )
    protocol_rejection: ReceiverGroupRejection | None = None
    os_errno: int | None = None
    linux_boot_id: bytes | None = None
    configuration: ReceiverGroupState | None = field(default=None, repr=False)


class ReceiverConfigurationReader:
    """Construct and call on the persistence thread; never transfers file work.

    ``read`` performs the worker-side operation only. The public control channel
    will own its separate caller deadline, serialization and cancellation rules.
    No caller-side timeout guarantee is implied by this synchronous primitive.
    """

    def __init__(
        self,
        path: Path = DEFAULT_CONFIGURATION_PATH,
        *,
        expected_owner_uid: int | None = None,
        boot_id_path: Path = LINUX_BOOT_ID_PATH,
    ) -> None:
        if not isinstance(path, Path) or not isinstance(boot_id_path, Path):
            raise TypeError("configuration and boot-ID paths must be Path values")
        if expected_owner_uid is not None and (
            type(expected_owner_uid) is not int or expected_owner_uid < 0
        ):
            raise ValueError("expected_owner_uid must be a non-negative integer")
        self._owner = current_thread()
        self._path = path
        self._boot_id_path = boot_id_path
        self._expected_owner_uid = expected_owner_uid

    def read(self) -> ReceiverConfigurationLoadResult:
        if current_thread() is not self._owner:
            return ReceiverConfigurationLoadResult(
                ReceiverConfigurationLoadStatus.INTERFACE_VIOLATION,
                DiagnosticOperation.READ,
                interface_violation=PersistenceControlInterfaceViolation.WRONG_CALLER,
            )
        try:
            configuration = load_receiver_group(
                self._path,
                expected_owner_uid=self._expected_owner_uid,
            )
            try:
                boot_id = read_linux_boot_id(self._boot_id_path)
            except ValueError:
                return ReceiverConfigurationLoadResult(
                    ReceiverConfigurationLoadStatus.HOST_IDENTITY_REJECTED,
                    DiagnosticOperation.READ,
                )
        except ReceiverGroupRejectedError as exc:
            return ReceiverConfigurationLoadResult(
                ReceiverConfigurationLoadStatus.CONFIGURATION_REJECTED,
                DiagnosticOperation.READ,
                protocol_rejection=exc.reason,
            )
        except OSError as exc:
            return ReceiverConfigurationLoadResult(
                ReceiverConfigurationLoadStatus.OS_ERROR,
                DiagnosticOperation.READ,
                os_errno=exc.errno,
            )
        return ReceiverConfigurationLoadResult(
            ReceiverConfigurationLoadStatus.LOADED,
            DiagnosticOperation.NONE,
            linux_boot_id=boot_id,
            configuration=configuration,
        )
