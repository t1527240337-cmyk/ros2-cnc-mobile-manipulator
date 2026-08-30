from __future__ import annotations

from .domain import MachineMode
from .task_models import (
    MachineEvent,
    ReconciliationContext,
    ReconciliationResult,
    RobotTask,
    RobotTaskKind,
    TaskPriority,
)
from .task_queue import RobotTaskQueue


class MachineStateReconciler:
    """Turns authoritative PLC state into idempotent robot work."""

    def __init__(self, queue: RobotTaskQueue):
        self.queue = queue
        self._last_sequence: dict[tuple[str, str], int] = {}

    def apply(
        self,
        event: MachineEvent,
        context: ReconciliationContext,
    ) -> ReconciliationResult:
        if not event.machine_id:
            raise ValueError("machine_id is required")
        if event.sequence < 0:
            raise ValueError("event sequence cannot be negative")

        source = (event.machine_id, event.controller_session)
        if event.sequence <= self._last_sequence.get(source, -1):
            return ReconciliationResult(accepted_event=False)
        self._last_sequence[source] = event.sequence

        result = ReconciliationResult(accepted_event=True)
        mode = MachineMode(event.mode)
        if mode == MachineMode.DONE:
            self._reconcile_done_machine(event, context, result)
        elif mode == MachineMode.IDLE:
            self._reconcile_idle_machine(event, context, result)
        elif mode == MachineMode.FAULT:
            self._reconcile_faulted_machine(event, result)
        return result

    def _reconcile_done_machine(
        self,
        event: MachineEvent,
        context: ReconciliationContext,
        result: ReconciliationResult,
    ) -> None:
        if not event.part_present:
            result.warnings.append(
                f"{event.machine_id}: DONE state disagrees with part-present sensor"
            )
            return

        finished_part_id = event.part_id or self._recovery_part_id(event)
        unload_key = f"unload:{event.machine_id}:{finished_part_id}"
        unload = RobotTask.create(
            kind=RobotTaskKind.UNLOAD_FINISHED,
            priority=TaskPriority.UNLOAD,
            deduplication_key=unload_key,
            machine_id=event.machine_id,
            part_id=finished_part_id,
            order_id=context.order_id,
            source_event_id=event.event_id,
        )
        unload, added = self.queue.enqueue(unload)
        if added:
            result.tasks_added.append(unload.task_id)

        part_id = self._next_unassigned_part(context)
        if not context.allow_loading or not part_id:
            return
        load = self._load_task(
            event=event,
            context=context,
            part_id=part_id,
            depends_on=(unload.task_id,),
        )
        load, added = self.queue.enqueue(load)
        if added:
            result.tasks_added.append(load.task_id)

    def _reconcile_idle_machine(
        self,
        event: MachineEvent,
        context: ReconciliationContext,
        result: ReconciliationResult,
    ) -> None:
        if event.part_present:
            result.warnings.append(
                f"{event.machine_id}: IDLE state disagrees with part-present sensor"
            )
            return
        if not context.allow_loading:
            return

        part_id = self._next_unassigned_part(context)
        if not part_id:
            return
        load = self._load_task(event, context, part_id)
        load, added = self.queue.enqueue(load)
        if added:
            result.tasks_added.append(load.task_id)

    def _reconcile_faulted_machine(
        self,
        event: MachineEvent,
        result: ReconciliationResult,
    ) -> None:
        if event.part_present:
            result.warnings.append(
                f"{event.machine_id}: part is trapped in a faulted machine; "
                "manual intervention is required"
            )
            return

        detail = (
            f"{event.machine_id}: canceled before loading because the "
            "machine faulted"
        )
        result.tasks_canceled.extend(
            self.queue.cancel_pending_loads_for_machine(event.machine_id, detail)
        )

    def _load_task(
        self,
        event: MachineEvent,
        context: ReconciliationContext,
        part_id: str,
        depends_on: tuple[str, ...] = (),
    ) -> RobotTask:
        key = f"load:{context.order_id}:{event.machine_id}:{part_id}"
        return RobotTask.create(
            kind=RobotTaskKind.LOAD_RAW,
            priority=TaskPriority.LOAD,
            deduplication_key=key,
            machine_id=event.machine_id,
            part_id=part_id,
            order_id=context.order_id,
            source_event_id=event.event_id,
            depends_on=depends_on,
        )

    def _next_unassigned_part(self, context: ReconciliationContext) -> str:
        assigned = self.queue.assigned_part_ids()
        return next(
            (
                part_id
                for part_id in context.production_part_ids
                if part_id not in assigned
            ),
            "",
        )

    @staticmethod
    def _recovery_part_id(event: MachineEvent) -> str:
        return (
            f"recovery-{event.machine_id}-{event.controller_session}-{event.sequence}"
        )
