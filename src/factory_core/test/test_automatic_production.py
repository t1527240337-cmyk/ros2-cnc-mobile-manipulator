import unittest

from factory_core.automatic_production import (
    FactoryAvailability,
    choose_automatic_order,
)


class AutomaticProductionPolicyTests(unittest.TestCase):
    def test_dispatches_only_idle_allowed_machines_in_preferred_order(self):
        decision = choose_automatic_order(
            FactoryAvailability(
                raw_part_count=5,
                battery_percentage=0.80,
                idle_machine_ids=("machine_1", "machine_2"),
            ),
            allowed_machine_ids=("machine_2", "machine_3", "machine_1"),
            max_batch_size=3,
            minimum_battery=0.25,
        )

        self.assertTrue(decision.should_dispatch)
        self.assertEqual(decision.quantity, 2)
        self.assertEqual(
            decision.allowed_machine_ids,
            ("machine_2", "machine_1"),
        )

    def test_batch_is_bounded_by_inventory(self):
        decision = choose_automatic_order(
            FactoryAvailability(
                raw_part_count=1,
                battery_percentage=0.80,
                idle_machine_ids=("machine_1", "machine_2", "machine_3"),
            ),
            allowed_machine_ids=("machine_1", "machine_2", "machine_3"),
            max_batch_size=3,
            minimum_battery=0.25,
        )

        self.assertEqual(decision.quantity, 1)
        self.assertEqual(decision.allowed_machine_ids, ("machine_1",))

    def test_low_battery_waits_without_creating_work(self):
        decision = choose_automatic_order(
            FactoryAvailability(
                raw_part_count=3,
                battery_percentage=0.20,
                idle_machine_ids=("machine_1",),
            ),
            allowed_machine_ids=("machine_1",),
            max_batch_size=1,
            minimum_battery=0.25,
        )

        self.assertFalse(decision.should_dispatch)
        self.assertEqual(decision.state, "waiting_battery")

    def test_active_manual_order_blocks_automatic_dispatch(self):
        decision = choose_automatic_order(
            FactoryAvailability(
                raw_part_count=3,
                battery_percentage=0.80,
                idle_machine_ids=("machine_1",),
                active_order_id="operator-order",
            ),
            allowed_machine_ids=("machine_1",),
            max_batch_size=1,
            minimum_battery=0.25,
        )

        self.assertFalse(decision.should_dispatch)
        self.assertEqual(decision.state, "waiting_robot")

    def test_empty_inventory_has_readable_wait_state(self):
        decision = choose_automatic_order(
            FactoryAvailability(
                raw_part_count=0,
                battery_percentage=0.80,
                idle_machine_ids=("machine_1",),
            ),
            allowed_machine_ids=("machine_1",),
            max_batch_size=1,
            minimum_battery=0.25,
        )

        self.assertEqual(decision.state, "waiting_material")
        self.assertIn("empty", decision.reason)


if __name__ == "__main__":
    unittest.main()
