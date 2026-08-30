from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from .task_models import (
    InterruptedTaskResolution,
    RobotTask,
    RobotTaskKind,
    RobotTaskStatus,
    TERMINAL_TASK_STATUSES,
)


class RobotTaskQueue:
    """Deterministic, idempotent queue for long-running robot missions."""

    def __init__(self, tasks: Iterable[RobotTask] = ()):
        self._tasks: dict[str, RobotTask] = {}
        self._task_id_by_key: dict[str, str] = {}
        self._next_sequence = 1
        for task in tasks:
            self.enqueue(task)

    def enqueue(self, task: RobotTask) -> tuple[RobotTask, bool]:
        if not isinstance(task, RobotTask):
            raise TypeError("only RobotTask instances belong in the robot work queue")

        existing_id = self._task_id_by_key.get(task.deduplication_key)
        if existing_id:
            return self._tasks[existing_id], False

        if task.task_id in self._tasks:
            raise ValueError(f"duplicate task id: {task.task_id}")

        if task.created_sequence <= 0:
            task.created_sequence = self._next_sequence
        self._next_sequence = max(self._next_sequence, task.created_sequence + 1)
        self._tasks[task.task_id] = task
        self._task_id_by_key[task.deduplication_key] = task.task_id
        return task, True

    def reserve_next(self) -> RobotTask | None:
        eligible = [
            task
            for task in self._tasks.values()
            if task.status == RobotTaskStatus.PENDING
            and self._dependencies_succeeded(task)
        ]
        if not eligible:
            return None

        task = min(eligible, key=lambda item: (int(item.priority), item.created_sequence))
        task.status = RobotTaskStatus.RESERVED
        task.attempts += 1
        return task

    def start(self, task_id: str) -> None:
        task = self.get(task_id)
        if task.status != RobotTaskStatus.RESERVED:
            raise ValueError(f"{task_id}: only a reserved task can start")
        task.status = RobotTaskStatus.RUNNING

    def succeed(self, task_id: str, detail: str = "") -> None:
        task = self.get(task_id)
        if task.status not in (RobotTaskStatus.RESERVED, RobotTaskStatus.RUNNING):
            raise ValueError(f"{task_id}: task is not active")
        task.status = RobotTaskStatus.SUCCEEDED
        task.detail = detail

    def fail(self, task_id: str, detail: str, *, retryable: bool = False) -> None:
        task = self.get(task_id)
        if task.status not in (RobotTaskStatus.RESERVED, RobotTaskStatus.RUNNING):
            raise ValueError(f"{task_id}: task is not active")
        task.detail = detail
        task.status = RobotTaskStatus.PENDING if retryable else RobotTaskStatus.FAILED

    def cancel(self, task_id: str, detail: str = "") -> None:
        task = self.get(task_id)
        if task.status in TERMINAL_TASK_STATUSES:
            return
        task.status = RobotTaskStatus.CANCELED
        task.detail = detail

    def record_progress(
        self,
        task_id: str,
        *,
        phase: str,
        feedback: str,
    ) -> None:
        """Persist the latest reported physical phase for restart diagnosis."""
        task = self.get(task_id)
        if task.status != RobotTaskStatus.RUNNING:
            raise ValueError(f"{task_id}: only a running task has progress")
        if not phase.strip():
            raise ValueError("physical task phase is required")
        task.last_phase = phase.strip()
        task.last_feedback = feedback.strip()

    def resolve_interrupted_task(
        self,
        task_id: str,
        *,
        resolution: InterruptedTaskResolution,
        physical_state_verified: bool,
        operator_note: str,
    ) -> RobotTask:
        """Apply an operator decision after reconciling the physical cell."""
        task = self.get(task_id)
        if task.status != RobotTaskStatus.FAILED:
            raise ValueError(
                f"{task_id}: only a failed task can be reconciled"
            )
        if not physical_state_verified:
            raise ValueError(
                "robot, part, inventory and PLC state must be verified"
            )
        note = operator_note.strip()
        if not note:
            raise ValueError("operator_note is required for the audit trail")

        if resolution == InterruptedTaskResolution.RETRY:
            task.status = RobotTaskStatus.PENDING
            outcome = "safe retry authorized"
        elif resolution == InterruptedTaskResolution.MARK_SUCCEEDED:
            task.status = RobotTaskStatus.SUCCEEDED
            outcome = "physical completion confirmed"
        elif resolution == InterruptedTaskResolution.CANCEL:
            task.status = RobotTaskStatus.CANCELED
            outcome = "task canceled after physical reconciliation"
        else:
            raise ValueError(f"unsupported task resolution: {resolution}")
        task.detail = f"{outcome}: {note}"
        task.reconciliation_note = f"{resolution.value}: {note}"
        return task

    def recover_interrupted_tasks(
        self,
    ) -> dict[str, RobotTaskStatus]:
        """Apply a fail-safe restart policy to in-flight robot motion.

        RESERVED means no action goal was accepted yet and is safe to retry.
        RUNNING has an unknown physical outcome after a process crash, so it
        must be reconciled by an operator instead of replayed automatically.
        """

        recovered: dict[str, RobotTaskStatus] = {}
        for task in self.tasks():
            if task.status == RobotTaskStatus.RESERVED:
                task.status = RobotTaskStatus.PENDING
                task.detail = "dispatcher restarted before action acceptance"
                recovered[task.task_id] = task.status
            elif task.status == RobotTaskStatus.RUNNING:
                task.status = RobotTaskStatus.FAILED
                checkpoint = ""
                if task.last_phase:
                    checkpoint = f"; last phase={task.last_phase}"
                    if task.last_feedback:
                        checkpoint += f" ({task.last_feedback})"
                task.detail = (
                    "dispatcher restarted with unknown physical outcome; "
                    "reconcile robot, part, inventory and PLC state before "
                    f"choosing a resolution{checkpoint}"
                )
                recovered[task.task_id] = task.status
        return recovered

    def get(self, task_id: str) -> RobotTask:
        try:
            return self._tasks[task_id]
        except KeyError as exc:
            raise KeyError(f"unknown task: {task_id}") from exc

    def tasks(self) -> list[RobotTask]:
        return sorted(self._tasks.values(), key=lambda task: task.created_sequence)

    def tasks_of_kind(self, kind: RobotTaskKind) -> list[RobotTask]:
        return [task for task in self.tasks() if task.kind == kind]

    def assigned_part_ids(self) -> set[str]:
        return {
            task.part_id
            for task in self._tasks.values()
            if task.part_id and task.status != RobotTaskStatus.CANCELED
        }

    def cancel_pending_loads_for_machine(
        self, machine_id: str, detail: str
    ) -> list[str]:
        """Release raw parts assigned to a machine before robot execution."""
        canceled: list[str] = []
        for task in self.tasks_of_kind(RobotTaskKind.LOAD_RAW):
            if (
                task.machine_id == machine_id
                and task.status == RobotTaskStatus.PENDING
            ):
                self.cancel(task.task_id, detail)
                canceled.append(task.task_id)
        return canceled

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": 1,
            "next_sequence": self._next_sequence,
            "tasks": [task.to_dict() for task in self.tasks()],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RobotTaskQueue":
        if data.get("version") != 1:
            raise ValueError("unsupported robot task queue version")
        queue = cls(RobotTask.from_dict(item) for item in data.get("tasks", []))
        queue._next_sequence = max(int(data.get("next_sequence", 1)), queue._next_sequence)
        return queue

    def _dependencies_succeeded(self, task: RobotTask) -> bool:
        return all(
            dependency_id in self._tasks
            and self._tasks[dependency_id].status == RobotTaskStatus.SUCCEEDED
            for dependency_id in task.depends_on
        )
