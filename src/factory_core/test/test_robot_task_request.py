import unittest

from factory_core.robot_task_request import physical_request_from_task
from factory_core.task_models import (
    RobotTask,
    RobotTaskKind,
    RobotTaskStatus,
    TaskPriority,
)
from factory_core.task_queue import RobotTaskQueue


class PhysicalRobotTaskRequestTests(unittest.TestCase):
    def test_load_task_preserves_only_task_identity_and_machine(self):
        request = physical_request_from_task(
            self._task(RobotTaskKind.LOAD_RAW, "raw_part_2")
        )

        self.assertTrue(request.task_id.startswith("task-"))
        self.assertEqual(request.kind, RobotTaskKind.LOAD_RAW)
        self.assertEqual(request.machine_id, "machine_1")
        self.assertEqual(request.part_id, "raw_part_2")

    def test_unload_accepts_a_recovery_identity_without_name_parsing(self):
        request = physical_request_from_task(
            self._task(
                RobotTaskKind.UNLOAD_FINISHED,
                "recovery-machine_1-boot-a-4",
            )
        )

        self.assertEqual(request.kind, RobotTaskKind.UNLOAD_FINISHED)
        self.assertEqual(request.part_id, "recovery-machine_1-boot-a-4")

    def test_missing_machine_is_rejected_before_dispatch(self):
        task = self._task(RobotTaskKind.LOAD_RAW, "raw_part_1")
        task.machine_id = ""
        with self.assertRaisesRegex(ValueError, "machine_id"):
            physical_request_from_task(task)

    @staticmethod
    def _task(kind: RobotTaskKind, part_id: str) -> RobotTask:
        return RobotTask.create(
            kind=kind,
            priority=(
                TaskPriority.LOAD
                if kind == RobotTaskKind.LOAD_RAW
                else TaskPriority.UNLOAD
            ),
            deduplication_key=f"{kind.value}:machine_1:{part_id}",
            machine_id="machine_1",
            part_id=part_id,
            order_id="order-1",
        )


class InterruptedTaskRecoveryTests(unittest.TestCase):
    def test_reserved_task_returns_to_pending_but_running_task_stops(self):
        queue = RobotTaskQueue(
            [
                PhysicalRobotTaskRequestTests._task(
                    RobotTaskKind.LOAD_RAW, "raw_part_1"
                ),
                PhysicalRobotTaskRequestTests._task(
                    RobotTaskKind.LOAD_RAW, "raw_part_2"
                ),
            ]
        )
        reserved = queue.reserve_next()
        queue.start(reserved.task_id)
        second = queue.reserve_next()

        recovered = queue.recover_interrupted_tasks()

        self.assertEqual(
            queue.get(reserved.task_id).status,
            RobotTaskStatus.FAILED,
        )
        self.assertEqual(
            queue.get(second.task_id).status,
            RobotTaskStatus.PENDING,
        )
        self.assertEqual(
            recovered,
            {
                reserved.task_id: RobotTaskStatus.FAILED,
                second.task_id: RobotTaskStatus.PENDING,
            },
        )


if __name__ == "__main__":
    unittest.main()
