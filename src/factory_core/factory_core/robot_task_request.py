from __future__ import annotations

from dataclasses import dataclass

from .task_models import RobotTask, RobotTaskKind


@dataclass(frozen=True)
class PhysicalRobotTaskRequest:
    task_id: str
    kind: RobotTaskKind
    machine_id: str
    part_id: str


def physical_request_from_task(task: RobotTask) -> PhysicalRobotTaskRequest:
    """Validate and translate one queued task into the physical task DTO."""

    if task.kind not in (
        RobotTaskKind.LOAD_RAW,
        RobotTaskKind.UNLOAD_FINISHED,
    ):
        raise ValueError(f"{task.kind.value}: no physical workflow is defined")
    if not task.machine_id:
        raise ValueError(f"{task.task_id}: machine_id is required")
    if not task.part_id:
        raise ValueError(f"{task.task_id}: part_id is required")

    return PhysicalRobotTaskRequest(
        task_id=task.task_id,
        kind=task.kind,
        machine_id=task.machine_id,
        part_id=task.part_id,
    )
