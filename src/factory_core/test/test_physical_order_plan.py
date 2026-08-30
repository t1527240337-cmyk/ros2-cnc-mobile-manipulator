from types import SimpleNamespace
import threading
import time
import unittest

from factory_interfaces.srv import MachineCommand
from factory_core.domain import FactoryState, HeldPartKind
from factory_core.physical_order_plan import (
    EnergyDecision,
    MachineOperation,
    Manipulation,
    PART_CYCLE_STEP_COUNT,
    PhysicalPartAssignment,
    StepKind,
    TransferEvent,
    build_load_cycle,
    build_part_cycle,
    build_unload_cycle,
    energy_decision,
    production_batch_sizes,
)
from factory_core.physical_order_executor import (
    PhysicalMachineFault,
    PhysicalOrderExecutor,
)
from factory_core.ros_runtime import FactoryRuntime


class PhysicalOrderPlanTests(unittest.TestCase):
    def test_cycle_commits_only_after_physical_actions(self):
        steps = build_part_cycle(
            machine_id="machine_2",
            part_id="raw_part_2",
        )

        self.assertEqual(len(steps), PART_CYCLE_STEP_COUNT)
        assignment = PhysicalPartAssignment("machine_2", "raw_part_2")
        load_steps = build_load_cycle(assignment)
        unload_steps = build_unload_cycle(assignment)
        self.assertEqual(steps, load_steps + unload_steps)
        self.assertEqual((len(load_steps), len(unload_steps)), (11, 9))
        self.assertEqual(unload_steps[0].kind, StepKind.WAIT_MACHINE_DONE)
        self.assertEqual(unload_steps[0].part_id, "raw_part_2")
        raw_pick = next(
            index
            for index, step in enumerate(steps)
            if step.manipulation == Manipulation.PICK
            and step.station_id == "raw_bin"
        )
        raw_commit = next(
            index
            for index, step in enumerate(steps)
            if step.transfer_event == TransferEvent.PICK_FROM_RAW
        )
        cnc_place = next(
            index
            for index, step in enumerate(steps)
            if step.manipulation == Manipulation.PLACE
            and step.station_id == "machine_2"
        )
        confirm_load = next(
            index
            for index, step in enumerate(steps)
            if step.machine_operation == MachineOperation.CONFIRM_LOAD
        )
        finished_place = next(
            index
            for index, step in enumerate(steps)
            if step.manipulation == Manipulation.PLACE
            and step.station_id == "finished_bin"
        )
        finished_commit = next(
            index
            for index, step in enumerate(steps)
            if step.transfer_event == TransferEvent.PLACE_IN_FINISHED
        )

        self.assertLess(raw_pick, raw_commit)
        self.assertLess(cnc_place, confirm_load)
        self.assertLess(finished_place, finished_commit)
        self.assertEqual(steps[-1].kind, StepKind.UNDOCK)

        machine_undocks = tuple(
            step
            for step in steps
            if step.kind == StepKind.UNDOCK
            and step.station_id == "machine_2"
        )
        self.assertTrue(machine_undocks)
        self.assertTrue(all(not step.stow_arm for step in machine_undocks))

    def test_cycle_rejects_invalid_identity(self):
        with self.assertRaisesRegex(ValueError, "machine_id"):
            build_part_cycle(
                machine_id="",
                part_id="raw_part_2",
            )
        with self.assertRaisesRegex(ValueError, "part_id"):
            build_part_cycle(
                machine_id="machine_2",
                part_id="",
            )

    def test_order_batches_fill_distinct_machines_before_reuse(self):
        self.assertEqual(production_batch_sizes(3, 3), (3,))
        self.assertEqual(production_batch_sizes(3, 2), (2, 1))
        self.assertEqual(production_batch_sizes(3, 1), (1, 1, 1))

    def test_order_batches_reject_impossible_capacity(self):
        with self.assertRaisesRegex(ValueError, "quantity"):
            production_batch_sizes(0, 2)
        with self.assertRaisesRegex(ValueError, "machine"):
            production_batch_sizes(2, 0)

    def test_energy_policy_recharges_only_at_low_battery(self):
        self.assertEqual(
            energy_decision(
                0.24,
                auto_recharge=True,
                low_threshold=0.25,
            ),
            EnergyDecision.RECHARGE,
        )
        self.assertEqual(
            energy_decision(
                0.25,
                auto_recharge=True,
                low_threshold=0.25,
            ),
            EnergyDecision.CONTINUE,
        )

    def test_energy_policy_blocks_when_automatic_recharge_is_disabled(self):
        self.assertEqual(
            energy_decision(
                0.10,
                auto_recharge=False,
                low_threshold=0.25,
            ),
            EnergyDecision.BLOCK,
        )

    def test_energy_policy_rejects_invalid_percentages(self):
        with self.assertRaisesRegex(ValueError, "battery_percentage"):
            energy_decision(1.01, auto_recharge=True, low_threshold=0.25)

    def test_assignment_validates_identity(self):
        with self.assertRaisesRegex(ValueError, "part_id"):
            PhysicalPartAssignment("machine_2", "").validate()

    def test_faulted_loaded_machine_requires_manual_intervention(self):
        executor = PhysicalOrderExecutor.__new__(PhysicalOrderExecutor)
        executor._state_lock = threading.RLock()
        executor._machine_states = {
            "machine_2": SimpleNamespace(
                state=4,
                part_id="raw_part_2",
                fault_code=73,
            )
        }
        executor._machine_state_received_at = {"machine_2": time.monotonic()}
        executor._raise_if_cancelled = lambda _goal_handle: None
        executor.get_parameter = lambda _name: SimpleNamespace(value=1.0)

        with self.assertRaises(PhysicalMachineFault) as caught:
            executor._wait_for_machine_done(
                "machine_2",
                goal_handle=SimpleNamespace(),
            )

        fault = caught.exception
        self.assertEqual(fault.machine_id, "machine_2")
        self.assertEqual(fault.part_id, "raw_part_2")
        self.assertEqual(fault.fault_code, 73)
        self.assertIn("manual intervention", str(fault))

    def test_unload_rejects_a_different_part_in_the_machine(self):
        executor = PhysicalOrderExecutor.__new__(PhysicalOrderExecutor)
        executor._state_lock = threading.RLock()
        executor._machine_states = {
            "machine_2": SimpleNamespace(
                state=3,
                door_open=True,
                part_id="raw_part_1",
                fault_code=0,
            )
        }
        executor._machine_state_received_at = {"machine_2": time.monotonic()}
        executor._raise_if_cancelled = lambda _goal_handle: None
        executor.get_parameter = lambda _name: SimpleNamespace(value=1.0)

        with self.assertRaisesRegex(
            RuntimeError,
            "not expected raw_part_2",
        ):
            executor._wait_for_machine_done(
                "machine_2",
                goal_handle=SimpleNamespace(),
                expected_part_id="raw_part_2",
            )

    def test_runtime_inventory_follows_physical_acknowledgements(self):
        runtime = FactoryRuntime.__new__(FactoryRuntime)
        runtime.state = FactoryState.default()
        runtime.state.raw_part_count = 3

        runtime._commit_raw_pick("raw_part_2")
        self.assertEqual(runtime.state.raw_part_count, 2)
        self.assertEqual(runtime.state.held_part_id, "raw_part_2")
        self.assertEqual(runtime.state.held_part_kind, HeldPartKind.RAW)

        load = SimpleNamespace(
            command=MachineCommand.Request.CONFIRM_LOAD,
            part_id="raw_part_2",
        )
        runtime._validate_machine_inventory_transition(load)
        runtime._commit_machine_inventory_transition(load)
        self.assertEqual(runtime.state.held_part_id, "")

        unload = SimpleNamespace(
            command=MachineCommand.Request.CONFIRM_UNLOAD,
            part_id="raw_part_2",
        )
        runtime._validate_machine_inventory_transition(unload)
        runtime._commit_machine_inventory_transition(unload)
        self.assertEqual(runtime.state.held_part_kind, HeldPartKind.FINISHED)

        runtime._commit_finished_place("raw_part_2")
        self.assertEqual(runtime.state.finished_part_count, 1)
        self.assertEqual(runtime.state.held_part_id, "")
        self.assertEqual(runtime.state.held_part_kind, HeldPartKind.NONE)

    def test_runtime_rejects_impossible_inventory_transition(self):
        runtime = FactoryRuntime.__new__(FactoryRuntime)
        runtime.state = FactoryState.default()
        runtime.state.held_part_id = "raw_part_1"
        runtime.state.held_part_kind = HeldPartKind.RAW

        with self.assertRaisesRegex(ValueError, "not raw_part_2"):
            runtime._validate_machine_inventory_transition(
                SimpleNamespace(
                    command=MachineCommand.Request.CONFIRM_LOAD,
                    part_id="raw_part_2",
                )
            )
        with self.assertRaisesRegex(ValueError, "has not completed machining"):
            runtime._commit_finished_place("raw_part_1")


if __name__ == "__main__":
    unittest.main()
