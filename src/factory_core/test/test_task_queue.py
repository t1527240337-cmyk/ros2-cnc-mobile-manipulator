import tempfile
import unittest
from pathlib import Path

from factory_core.domain import MachineMode
from factory_core.state_reconciler import MachineStateReconciler
from factory_core.task_dispatcher import DispatchKind, RobotTaskDispatcher
from factory_core.task_models import (
    InterruptedTaskResolution,
    MachineEvent,
    ReconciliationContext,
    RobotTask,
    RobotTaskKind,
    RobotTaskStatus,
    TaskPriority,
)
from factory_core.task_queue import RobotTaskQueue
from factory_core.task_queue_store import RobotTaskQueueStore


class MachineStateReconcilerTests(unittest.TestCase):
    def setUp(self):
        self.queue = RobotTaskQueue()
        self.reconciler = MachineStateReconciler(self.queue)
        self.context = ReconciliationContext(
            order_id="order-100",
            production_part_ids=("order:part:1", "order:part:2", "order:part:3"),
        )

    def test_done_event_creates_unload_then_dependent_load(self):
        result = self.reconciler.apply(
            self._event(MachineMode.DONE, part_present=True, part_id="finished_017"),
            self.context,
        )

        self.assertTrue(result.accepted_event)
        self.assertEqual(len(result.tasks_added), 2)
        unload = self.queue.tasks_of_kind(RobotTaskKind.UNLOAD_FINISHED)[0]
        load = self.queue.tasks_of_kind(RobotTaskKind.LOAD_RAW)[0]
        self.assertEqual(load.depends_on, (unload.task_id,))

        self.assertEqual(self.queue.reserve_next().task_id, unload.task_id)
        self.assertIsNone(self.queue.reserve_next())
        self.queue.succeed(unload.task_id)
        self.assertEqual(self.queue.reserve_next().task_id, load.task_id)

    def test_duplicate_and_old_events_do_not_duplicate_work(self):
        event = self._event(MachineMode.IDLE, part_present=False, sequence=8)
        first = self.reconciler.apply(event, self.context)
        duplicate = self.reconciler.apply(event, self.context)
        old = self.reconciler.apply(
            self._event(MachineMode.IDLE, part_present=False, sequence=7),
            self.context,
        )

        self.assertEqual(len(first.tasks_added), 1)
        self.assertFalse(duplicate.accepted_event)
        self.assertFalse(old.accepted_event)
        self.assertEqual(len(self.queue.tasks()), 1)

    def test_unknown_finished_part_becomes_traceable_recovery_part(self):
        self.reconciler.apply(
            self._event(MachineMode.DONE, part_present=True, part_id=""),
            ReconciliationContext(order_id="", allow_loading=False),
        )

        unload = self.queue.tasks_of_kind(RobotTaskKind.UNLOAD_FINISHED)[0]
        self.assertEqual(unload.part_id, "recovery-machine_1-boot-a-1")

    def test_faulted_machine_with_part_requires_manual_intervention(self):
        result = self.reconciler.apply(
            self._event(MachineMode.FAULT, part_present=True),
            self.context,
        )

        self.assertEqual(self.queue.tasks(), [])
        self.assertIn("trapped", result.warnings[0])

    def test_fault_before_loading_releases_part_for_another_machine(self):
        created = self.reconciler.apply(
            self._event(
                MachineMode.IDLE,
                machine_id="machine_1",
                part_present=False,
                sequence=1,
            ),
            self.context,
        )
        original = self.queue.get(created.tasks_added[0])

        fault = self.reconciler.apply(
            self._event(
                MachineMode.FAULT,
                machine_id="machine_1",
                part_present=False,
                sequence=2,
            ),
            self.context,
        )
        self.assertEqual(fault.tasks_canceled, [original.task_id])
        self.assertEqual(original.status, RobotTaskStatus.CANCELED)

        reassigned = self.reconciler.apply(
            self._event(
                MachineMode.IDLE,
                machine_id="machine_2",
                part_present=False,
            ),
            self.context,
        )
        replacement = self.queue.get(reassigned.tasks_added[0])
        self.assertEqual(replacement.part_id, original.part_id)
        self.assertEqual(replacement.machine_id, "machine_2")

    @staticmethod
    def _event(
        mode: MachineMode,
        *,
        machine_id: str = "machine_1",
        part_present: bool,
        part_id: str = "",
        sequence: int = 1,
    ) -> MachineEvent:
        return MachineEvent(
            machine_id=machine_id,
            controller_session="boot-a",
            sequence=sequence,
            mode=int(mode),
            part_present=part_present,
            part_id=part_id,
        )


class RobotTaskDispatcherTests(unittest.TestCase):
    def test_low_battery_preserves_queued_work_until_charge(self):
        queue = RobotTaskQueue()
        task, _ = queue.enqueue(self._unload_task())
        dispatcher = RobotTaskDispatcher(low_battery=0.25)

        low_battery = dispatcher.next_decision(queue, battery=0.20)
        self.assertEqual(low_battery.kind, DispatchKind.DOCK_AND_CHARGE)
        self.assertEqual(task.status, RobotTaskStatus.PENDING)

        after_charge = dispatcher.next_decision(queue, battery=0.80)
        self.assertEqual(after_charge.kind, DispatchKind.EXECUTE_TASK)
        self.assertEqual(after_charge.task.task_id, task.task_id)

    def test_query_objects_cannot_enter_robot_work_queue(self):
        queue = RobotTaskQueue()
        with self.assertRaisesRegex(TypeError, "only RobotTask"):
            queue.enqueue({"operation": "get_factory_state"})

    @staticmethod
    def _unload_task() -> RobotTask:
        return RobotTask.create(
            kind=RobotTaskKind.UNLOAD_FINISHED,
            priority=TaskPriority.UNLOAD,
            deduplication_key="unload:machine_1:finished_001",
            machine_id="machine_1",
            part_id="finished_001",
        )


class RobotTaskQueuePersistenceTests(unittest.TestCase):
    def test_queue_round_trip_preserves_dependencies_and_status(self):
        queue = RobotTaskQueue()
        unload, _ = queue.enqueue(RobotTaskDispatcherTests._unload_task())
        load, _ = queue.enqueue(
            RobotTask.create(
                kind=RobotTaskKind.LOAD_RAW,
                priority=TaskPriority.LOAD,
                deduplication_key="load:order-1:machine_1:raw_002",
                machine_id="machine_1",
                part_id="raw_002",
                order_id="order-1",
                depends_on=(unload.task_id,),
            )
        )
        queue.reserve_next()
        queue.succeed(unload.task_id)

        with tempfile.TemporaryDirectory() as directory:
            store = RobotTaskQueueStore(Path(directory) / "robot_tasks.json")
            store.save(queue)
            restored = store.load()

        self.assertEqual(restored.get(unload.task_id).status, RobotTaskStatus.SUCCEEDED)
        self.assertEqual(restored.get(load.task_id).depends_on, (unload.task_id,))
        self.assertEqual(restored.reserve_next().task_id, load.task_id)


class InterruptedTaskReconciliationTests(unittest.TestCase):
    def test_restart_reports_last_persisted_physical_phase(self):
        queue, task = self._running_queue()
        queue.record_progress(
            task.task_id,
            phase="place_machine_1",
            feedback="fixture contact verified",
        )

        recovered = queue.recover_interrupted_tasks()

        self.assertEqual(
            recovered, {task.task_id: RobotTaskStatus.FAILED}
        )
        self.assertIn("last phase=place_machine_1", task.detail)
        self.assertIn("fixture contact verified", task.detail)

    def test_reconciliation_requires_verified_state_and_operator_note(self):
        queue, task = self._failed_queue()

        with self.assertRaisesRegex(ValueError, "must be verified"):
            queue.resolve_interrupted_task(
                task.task_id,
                resolution=InterruptedTaskResolution.RETRY,
                physical_state_verified=False,
                operator_note="cell inspected",
            )
        with self.assertRaisesRegex(ValueError, "operator_note"):
            queue.resolve_interrupted_task(
                task.task_id,
                resolution=InterruptedTaskResolution.RETRY,
                physical_state_verified=True,
                operator_note="",
            )

    def test_verified_retry_returns_task_to_pending(self):
        queue, task = self._failed_queue()

        resolved = queue.resolve_interrupted_task(
            task.task_id,
            resolution=InterruptedTaskResolution.RETRY,
            physical_state_verified=True,
            operator_note="part remains in source slot",
        )

        self.assertEqual(resolved.status, RobotTaskStatus.PENDING)
        self.assertIn("safe retry authorized", resolved.detail)
        self.assertEqual(
            resolved.reconciliation_note,
            "retry: part remains in source slot",
        )

    def test_confirmed_completion_unblocks_dependent_work(self):
        queue, interrupted = self._running_queue()
        dependent, _ = queue.enqueue(
            RobotTask.create(
                kind=RobotTaskKind.LOAD_RAW,
                priority=TaskPriority.LOAD,
                deduplication_key="load:machine_2:raw_002",
                machine_id="machine_2",
                part_id="raw_002",
                depends_on=(interrupted.task_id,),
            )
        )
        queue.recover_interrupted_tasks()

        queue.resolve_interrupted_task(
            interrupted.task_id,
            resolution=InterruptedTaskResolution.MARK_SUCCEEDED,
            physical_state_verified=True,
            operator_note="PLC and fixture both contain raw_001",
        )

        self.assertEqual(
            queue.reserve_next().task_id,
            dependent.task_id,
        )

    def test_verified_cancel_is_terminal_and_audited(self):
        queue, task = self._failed_queue()

        queue.resolve_interrupted_task(
            task.task_id,
            resolution=InterruptedTaskResolution.CANCEL,
            physical_state_verified=True,
            operator_note="workpiece quarantined by operator",
        )

        self.assertEqual(task.status, RobotTaskStatus.CANCELED)
        self.assertEqual(
            task.reconciliation_note,
            "cancel: workpiece quarantined by operator",
        )

    @staticmethod
    def _running_queue():
        queue = RobotTaskQueue()
        task, _ = queue.enqueue(RobotTaskDispatcherTests._unload_task())
        queue.reserve_next()
        queue.start(task.task_id)
        return queue, task

    @classmethod
    def _failed_queue(cls):
        queue, task = cls._running_queue()
        queue.recover_interrupted_tasks()
        return queue, task


if __name__ == "__main__":
    unittest.main()
