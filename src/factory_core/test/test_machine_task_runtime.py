import tempfile
import unittest
from pathlib import Path

from factory_core.domain import MachineMode
from factory_core.machine_event_adapter import MachineEventAdapter
from factory_core.machine_task_runtime_store import (
    MachineTaskRuntimeState,
    MachineTaskRuntimeStore,
)
from factory_core.state_reconciler import MachineStateReconciler
from factory_core.task_models import (
    ReconciliationContext,
    RobotTask,
    RobotTaskKind,
    TaskPriority,
)
from factory_core.task_queue import RobotTaskQueue


class MachineEventAdapterTests(unittest.TestCase):
    def test_repeated_snapshot_and_door_only_changes_are_not_events(self):
        adapter = MachineEventAdapter(controller_session="plc-boot-a")

        first = adapter.observe(
            machine_id="machine_1", mode=MachineMode.IDLE, part_id=""
        )
        repeated = adapter.observe(
            machine_id="machine_1",
            mode=MachineMode.IDLE,
            part_id="",
            door_open=True,
        )

        self.assertEqual(first.sequence, 1)
        self.assertIsNone(repeated)

    def test_state_or_part_change_increments_machine_sequence(self):
        adapter = MachineEventAdapter(controller_session="plc-boot-a")
        adapter.observe(
            machine_id="machine_1", mode=MachineMode.IDLE, part_id=""
        )
        loaded = adapter.observe(
            machine_id="machine_1",
            mode=MachineMode.READY,
            part_id="raw_part_1",
        )

        self.assertEqual(loaded.sequence, 2)
        self.assertTrue(loaded.part_present)

    def test_part_sensor_can_report_an_unknown_finished_part(self):
        adapter = MachineEventAdapter(controller_session="plc-boot-a")
        event = adapter.observe(
            machine_id="machine_1",
            mode=MachineMode.DONE,
            part_id="",
            part_present=True,
        )

        self.assertTrue(event.part_present)


class MachineTaskRuntimePersistenceTests(unittest.TestCase):
    def test_restart_restores_event_cursor_and_prevents_duplicate_work(self):
        queue = RobotTaskQueue()
        adapter = MachineEventAdapter(controller_session="plc-boot-a")
        context = ReconciliationContext(
            order_id="order-1",
            production_part_ids=("order-1:part:1",),
        )
        event = adapter.observe(
            machine_id="machine_1", mode=MachineMode.IDLE, part_id=""
        )
        MachineStateReconciler(queue).apply(event, context)

        with tempfile.TemporaryDirectory() as directory:
            store = MachineTaskRuntimeStore(Path(directory) / "runtime.json")
            store.save(MachineTaskRuntimeState(queue=queue, adapter=adapter))
            restored = store.load()

        repeated = restored.adapter.observe(
            machine_id="machine_1", mode=MachineMode.IDLE, part_id=""
        )
        self.assertIsNone(repeated)
        self.assertEqual(
            len(restored.queue.tasks_of_kind(RobotTaskKind.LOAD_RAW)), 1
        )

    def test_runtime_file_contains_queue_and_adapter_as_one_versioned_record(self):
        runtime = MachineTaskRuntimeState(
            queue=RobotTaskQueue(),
            adapter=MachineEventAdapter(controller_session="plc-boot-a"),
        )
        payload = runtime.to_dict()

        self.assertEqual(payload["version"], 1)
        self.assertIn("queue", payload)
        self.assertIn("event_adapter", payload)

    def test_runtime_round_trip_preserves_physical_progress(self):
        queue = RobotTaskQueue()
        task, _ = queue.enqueue(
            RobotTask.create(
                kind=RobotTaskKind.LOAD_RAW,
                priority=TaskPriority.LOAD,
                deduplication_key="load:machine_1:raw_part_1",
                machine_id="machine_1",
                part_id="raw_part_1",
            )
        )
        queue.reserve_next()
        queue.start(task.task_id)
        queue.record_progress(
            task.task_id,
            phase="pick_raw_bin",
            feedback="bilateral contact verified",
        )
        runtime = MachineTaskRuntimeState(
            queue=queue,
            adapter=MachineEventAdapter(controller_session="plc-boot-a"),
        )

        restored = MachineTaskRuntimeState.from_dict(runtime.to_dict())
        restored_task = restored.queue.get(task.task_id)

        self.assertEqual(restored_task.last_phase, "pick_raw_bin")
        self.assertEqual(
            restored_task.last_feedback,
            "bilateral contact verified",
        )


if __name__ == "__main__":
    unittest.main()
