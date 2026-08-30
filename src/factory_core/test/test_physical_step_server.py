from types import SimpleNamespace
import threading
import unittest

from factory_interfaces.action import ExecutePhysicalStep
from factory_core.physical_order_plan import (
    MachineOperation,
    Manipulation,
    StepKind,
    TransferEvent,
)
from factory_core.physical_step_server import PhysicalStepServer


class PhysicalStepProtocolTests(unittest.TestCase):
    def setUp(self):
        self.server = PhysicalStepServer.__new__(PhysicalStepServer)
        self.server._owner = SimpleNamespace(
            _stations={
                "raw_bin": object(),
                "finished_bin": object(),
                "machine_1": object(),
            },
            _machine_ids=("machine_1",),
            _state_lock=threading.RLock(),
            _active_order_id="task-1",
            _pause_requested=True,
            _cancel_requested=True,
            _pending_finished_slot=2,
            _goal_reserved=True,
        )

    def goal(self, kind):
        goal = ExecutePhysicalStep.Goal()
        goal.task_id = "task-1"
        goal.step_kind = kind
        return goal

    def test_rejects_unknown_station_before_motion(self):
        goal = self.goal(ExecutePhysicalStep.Goal.DOCK)
        goal.station_id = "missing_station"
        self.assertIn("unknown station", self.server._validation_error(goal))

    def test_source_pick_requires_part_but_not_a_source_slot(self):
        goal = self.goal(ExecutePhysicalStep.Goal.PICK)
        goal.station_id = "raw_bin"
        self.assertIn("part_id", self.server._validation_error(goal))

        goal.part_id = "raw_part_1"
        self.assertEqual(self.server._validation_error(goal), "")
        step = self.server._physical_step(goal)
        self.assertEqual(step.kind, StepKind.MANIPULATE)
        self.assertEqual(step.manipulation, Manipulation.PICK)

    def test_machine_and_inventory_commands_are_typed(self):
        machine_goal = self.goal(ExecutePhysicalStep.Goal.CONFIRM_LOAD)
        machine_goal.machine_id = "machine_1"
        machine_goal.part_id = "raw_part_1"
        machine_step = self.server._physical_step(machine_goal)
        self.assertEqual(machine_step.kind, StepKind.MACHINE_COMMAND)
        self.assertEqual(
            machine_step.machine_operation, MachineOperation.CONFIRM_LOAD
        )

        transfer_goal = self.goal(ExecutePhysicalStep.Goal.COMMIT_PICK_RAW)
        transfer_goal.part_id = "raw_part_1"
        transfer_step = self.server._physical_step(transfer_goal)
        self.assertEqual(transfer_step.kind, StepKind.COMMIT_TRANSFER)
        self.assertEqual(
            transfer_step.transfer_event, TransferEvent.PICK_FROM_RAW
        )

    def test_recharge_result_survives_last_feedback_race(self):
        self.server._owner._battery_percentage = 0.803
        self.server._owner._ensure_cycle_energy = lambda *args, **kwargs: True
        goal = self.goal(ExecutePhysicalStep.Goal.ENSURE_ENERGY)
        goal.auto_recharge = True

        detail = self.server._execute_operation(goal, SimpleNamespace())

        self.assertEqual(
            detail, "charged to 80.3%; resuming production"
        )

    def test_close_session_does_not_release_in_flight_goal(self):
        self.server._close_session()
        self.assertEqual(self.server._owner._active_order_id, "")
        self.assertFalse(self.server._owner._pause_requested)
        self.assertFalse(self.server._owner._cancel_requested)
        self.assertIsNone(self.server._owner._pending_finished_slot)
        self.assertTrue(self.server._owner._goal_reserved)


if __name__ == "__main__":
    unittest.main()
