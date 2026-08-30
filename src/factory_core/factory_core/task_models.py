from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum, IntEnum
from typing import Any
from uuid import NAMESPACE_URL, uuid5


class RobotTaskKind(str, Enum):
    """Long-running work that occupies the mobile manipulator."""

    UNLOAD_FINISHED = "unload_finished"
    LOAD_RAW = "load_raw"
    DOCK_AND_CHARGE = "dock_and_charge"


class RobotTaskStatus(str, Enum):
    PENDING = "pending"
    RESERVED = "reserved"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


class InterruptedTaskResolution(str, Enum):
    RETRY = "retry"
    MARK_SUCCEEDED = "mark_succeeded"
    CANCEL = "cancel"


class TaskPriority(IntEnum):
    SAFETY = 0
    UNLOAD = 10
    LOAD = 20
    BACKGROUND = 30


TERMINAL_TASK_STATUSES = {
    RobotTaskStatus.SUCCEEDED,
    RobotTaskStatus.FAILED,
    RobotTaskStatus.CANCELED,
}


def stable_task_id(deduplication_key: str) -> str:
    """Return the same readable ID when an event is received more than once."""

    suffix = uuid5(NAMESPACE_URL, deduplication_key).hex[:12]
    return f"task-{suffix}"


@dataclass
class RobotTask:
    """A recoverable robot mission, not a query or operator control command."""

    task_id: str
    kind: RobotTaskKind
    priority: TaskPriority
    deduplication_key: str
    machine_id: str = ""
    part_id: str = ""
    order_id: str = ""
    source_event_id: str = ""
    depends_on: tuple[str, ...] = ()
    status: RobotTaskStatus = RobotTaskStatus.PENDING
    attempts: int = 0
    created_sequence: int = 0
    detail: str = ""
    last_phase: str = ""
    last_feedback: str = ""
    reconciliation_note: str = ""

    @classmethod
    def create(
        cls,
        *,
        kind: RobotTaskKind,
        priority: TaskPriority,
        deduplication_key: str,
        machine_id: str = "",
        part_id: str = "",
        order_id: str = "",
        source_event_id: str = "",
        depends_on: tuple[str, ...] = (),
    ) -> "RobotTask":
        return cls(
            task_id=stable_task_id(deduplication_key),
            kind=kind,
            priority=priority,
            deduplication_key=deduplication_key,
            machine_id=machine_id,
            part_id=part_id,
            order_id=order_id,
            source_event_id=source_event_id,
            depends_on=depends_on,
        )

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["kind"] = self.kind.value
        value["priority"] = int(self.priority)
        value["status"] = self.status.value
        value["depends_on"] = list(self.depends_on)
        return value

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RobotTask":
        value = dict(data)
        value["kind"] = RobotTaskKind(value["kind"])
        value["priority"] = TaskPriority(value["priority"])
        value["status"] = RobotTaskStatus(value["status"])
        value["depends_on"] = tuple(value.get("depends_on", ()))
        return cls(**value)


@dataclass(frozen=True)
class MachineEvent:
    """Versioned PLC snapshot used to deduplicate pushed machine events."""

    machine_id: str
    controller_session: str
    sequence: int
    mode: int
    part_present: bool
    part_id: str = ""

    @property
    def event_id(self) -> str:
        return f"{self.machine_id}:{self.controller_session}:{self.sequence}"


@dataclass(frozen=True)
class ReconciliationContext:
    order_id: str
    production_part_ids: tuple[str, ...] = ()
    allow_loading: bool = True


@dataclass
class ReconciliationResult:
    accepted_event: bool
    tasks_added: list[str] = field(default_factory=list)
    tasks_canceled: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
