import tempfile
import unittest
from pathlib import Path

from factory_core.checkpoint import CheckpointStore
from factory_core.domain import FactoryState, Machine, MachineMode, ProductionOrder
from factory_core.scheduler import DecisionKind, Scheduler, SimulationEngine


class MachineTests(unittest.TestCase):
    def test_valid_cycle(self):
        machine = Machine("machine_1", cycle_seconds=2.0)
        machine.open_door()
        machine.load("raw_001")
        machine.close_door()
        machine.start()
        machine.tick(2.0)
        self.assertEqual(machine.mode, MachineMode.DONE)
        machine.open_door()
        self.assertEqual(machine.unload(), "raw_001")

    def test_door_cannot_open_during_processing(self):
        machine = Machine("machine_1")
        machine.open_door()
        machine.load("raw_001")
        machine.close_door()
        machine.start()
        with self.assertRaises(ValueError):
            machine.open_door()


class MissionTests(unittest.TestCase):
    def test_three_machine_order_completes(self):
        state = FactoryState.default(cycle_seconds=5.0, raw_part_count=3)
        order = ProductionOrder("order-1", 3, list(state.machines))
        result = SimulationEngine(state, order).run()
        self.assertEqual(result.kind, DecisionKind.COMPLETE)
        self.assertEqual(order.completed, 3)
        self.assertEqual(state.finished_part_count, 3)
        self.assertEqual(state.raw_part_count, 0)

    def test_order_above_physical_inventory_is_rejected(self):
        state = FactoryState.default(raw_part_count=2)
        order = ProductionOrder("too-many-parts", 3, list(state.machines))

        with self.assertRaisesRegex(
            ValueError, "quantity exceeds available raw-part stock"
        ):
            order.validate(set(state.machines), state.raw_part_count)

    def test_order_size_is_not_bounded_by_a_taught_slot_count(self):
        state = FactoryState.default(raw_part_count=5)
        ProductionOrder("five-parts", 5, list(state.machines)).validate(
            set(state.machines), state.raw_part_count
        )

    def test_low_battery_docks_before_pick(self):
        state = FactoryState.default()
        state.battery = 0.20
        order = ProductionOrder("order-2", 1, list(state.machines))
        engine = SimulationEngine(state, order)
        self.assertEqual(engine.step().kind, DecisionKind.DOCK)
        self.assertGreaterEqual(state.battery, Scheduler().charge_target)

    def test_held_part_is_placed_before_docking(self):
        state = FactoryState.default(cycle_seconds=1.0)
        order = ProductionOrder("order-3", 1, list(state.machines))
        engine = SimulationEngine(state, order)
        self.assertEqual(engine.step().kind, DecisionKind.PICK_RAW)
        state.battery = 0.10
        self.assertEqual(engine.step().kind, DecisionKind.LOAD_MACHINE)

    def test_held_raw_part_is_reassigned_before_loading(self):
        state = FactoryState.default(cycle_seconds=1.0)
        order = ProductionOrder("order-reassign", 1, ["machine_1", "machine_2"])
        engine = SimulationEngine(state, order)
        picked = engine.step()
        self.assertEqual(picked.kind, DecisionKind.PICK_RAW)
        self.assertEqual(state.pending_machine_id, "machine_1")

        state.machines["machine_1"].inject_fault(31)
        reassigned = engine.step()

        self.assertEqual(reassigned.kind, DecisionKind.LOAD_MACHINE)
        self.assertEqual(reassigned.machine_id, "machine_2")
        self.assertEqual(state.machines["machine_2"].part_id, "raw_001")
        self.assertEqual(state.machines["machine_2"].mode, MachineMode.DONE)
        self.assertEqual(state.machines["machine_1"].mode, MachineMode.FAULT)

    def test_fault_with_trapped_part_blocks(self):
        state = FactoryState.default(cycle_seconds=20.0)
        order = ProductionOrder("order-4", 1, ["machine_1"])
        engine = SimulationEngine(state, order)
        engine.step()
        engine.step()
        state.machines["machine_1"].inject_fault(7)
        result = engine.run()
        self.assertEqual(result.kind, DecisionKind.BLOCKED)
        self.assertIn("manual intervention", result.detail)

    def test_checkpoint_round_trip(self):
        state = FactoryState.default()
        order = ProductionOrder("order-5", 2, list(state.machines))
        engine = SimulationEngine(state, order)
        engine.step()
        with tempfile.TemporaryDirectory() as directory:
            store = CheckpointStore(Path(directory) / "checkpoint.json")
            store.save(state, order)
            loaded_state, loaded_order = store.load()
        self.assertEqual(loaded_state.held_part_id, state.held_part_id)
        self.assertEqual(loaded_order.order_id, order.order_id)


if __name__ == "__main__":
    unittest.main()
