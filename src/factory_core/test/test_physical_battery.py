import math
import unittest

from ros_gz_interfaces.msg import Contact, Contacts

from factory_core.physical_battery import (
    BatteryModel,
    bounded_speed,
    charging_contacts_present,
)


class ChargingContactTests(unittest.TestCase):
    @staticmethod
    def _contacts(first: str, second: str) -> Contacts:
        message = Contacts()
        contact = Contact()
        contact.collision1.name = first
        contact.collision2.name = second
        message.contacts.append(contact)
        return message

    def test_accepts_only_robot_to_dock_electrode_contact(self):
        message = self._contacts(
            "factory_robot::base_link::charging_contact_collision",
            "charge_dock::dock::charging_contact_collision",
        )
        self.assertTrue(charging_contacts_present(message))

    def test_rejects_robot_contact_with_an_unrelated_obstacle(self):
        message = self._contacts(
            "factory_robot::base_link::charging_contact_collision",
            "temporary_obstacle::link::collision",
        )
        self.assertFalse(charging_contacts_present(message))

    def test_rejects_empty_contact_message(self):
        self.assertFalse(charging_contacts_present(Contacts()))


class SpeedSanitizationTests(unittest.TestCase):
    def test_caps_speed_magnitude(self):
        self.assertEqual(bounded_speed(-12.0, 2.0), 2.0)

    def test_rejects_non_finite_measurements_without_drain(self):
        self.assertEqual(bounded_speed(math.nan, 2.0), 0.0)
        self.assertEqual(bounded_speed(math.inf, 2.0), 0.0)

    def test_rejects_non_positive_limit(self):
        with self.assertRaises(ValueError):
            bounded_speed(1.0, 0.0)


class BatteryModelTests(unittest.TestCase):
    def setUp(self):
        self.model = BatteryModel(
            0.40,
            charge_rate=0.02,
            idle_drain_rate=0.001,
            linear_drain_rate=0.01,
            angular_drain_rate=0.002,
            arm_drain_rate=0.003,
        )

    def test_motion_consumes_more_energy_than_idle(self):
        idle = BatteryModel(
            0.40,
            charge_rate=0.02,
            idle_drain_rate=0.001,
            linear_drain_rate=0.01,
            angular_drain_rate=0.002,
            arm_drain_rate=0.003,
        )
        idle.advance(
            2.0,
            charging_contact=False,
            linear_speed=0.0,
            angular_speed=0.0,
            arm_speed_sum=0.0,
        )
        self.model.advance(
            2.0,
            charging_contact=False,
            linear_speed=0.5,
            angular_speed=0.4,
            arm_speed_sum=1.5,
        )
        self.assertLess(self.model.percentage, idle.percentage)

    def test_contact_charges_and_clamps_at_full(self):
        self.model.advance(
            100.0,
            charging_contact=True,
            linear_speed=0.0,
            angular_speed=0.0,
            arm_speed_sum=0.0,
        )
        self.assertEqual(self.model.percentage, 1.0)
        self.assertTrue(self.model.charging)


if __name__ == "__main__":
    unittest.main()
