import unittest

from factory_interfaces.srv import MachineCommand

from factory_core.domain import Machine, MachineMode
from factory_core.ros_runtime import FactoryRuntime


class MachineHandshakeTests(unittest.TestCase):
    def _processing_machine(self):
        machine = Machine("machine_1", cycle_seconds=2.0)
        machine.open_door()
        machine.load("raw_001")
        machine.close_door()
        machine.start()
        return machine

    def test_hold_freezes_cycle_and_resume_continues(self):
        machine = self._processing_machine()
        machine.tick(0.5)
        machine.hold()
        remaining = machine.remaining_seconds
        machine.tick(1.0)
        self.assertEqual(machine.remaining_seconds, remaining)
        machine.resume()
        machine.tick(remaining)
        self.assertEqual(machine.mode, MachineMode.DONE)

    def test_completion_exposes_safe_unload_state(self):
        machine = self._processing_machine()
        machine.tick(2.0)
        self.assertEqual(machine.mode, MachineMode.DONE)
        self.assertTrue(machine.door_open)


    def test_robot_acknowledgements_update_machine_part_register(self):
        machine = Machine("machine_1", cycle_seconds=1.0)
        machine.open_door()
        FactoryRuntime._apply_machine_command(
            machine,
            MachineCommand.Request.CONFIRM_LOAD,
            "raw_part_2",
        )
        self.assertEqual(machine.part_id, "raw_part_2")
        self.assertEqual(machine.mode, MachineMode.READY)

        machine.close_door()
        machine.start()
        machine.tick(1.0)
        FactoryRuntime._apply_machine_command(
            machine,
            MachineCommand.Request.CONFIRM_UNLOAD,
            "raw_part_2",
        )
        self.assertEqual(machine.part_id, "")
        self.assertEqual(machine.mode, MachineMode.IDLE)

    def test_unload_acknowledgement_rejects_wrong_part_id(self):
        machine = self._processing_machine()
        machine.tick(2.0)

        with self.assertRaisesRegex(ValueError, "not another_part"):
            FactoryRuntime._apply_machine_command(
                machine,
                MachineCommand.Request.CONFIRM_UNLOAD,
                "another_part",
            )
        self.assertEqual(machine.part_id, "raw_001")


if __name__ == "__main__":
    unittest.main()
