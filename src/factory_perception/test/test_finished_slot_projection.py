import unittest

import numpy as np

from factory_interfaces.msg import TrayOccupancy, TraySlotState
from factory_perception.finished_slot_detector_node import (
    configured_slot_targets,
    summarize_slots,
)
from factory_perception.slots import (
    PinholeIntrinsics,
    SlotTarget,
    detect_slots,
    project_slot,
)


class FinishedSlotProjectionTests(unittest.TestCase):
    def test_projects_sample_and_predicts_surface_depth(self):
        rotation = np.asarray(
            (
                (0.0, 1.0, 0.0),
                (0.0, 0.0, -1.0),
                (1.0, 0.0, 0.0),
            )
        )
        # The optical camera is one metre above the output-frame surface.
        translation = (0.0, 1.0, 0.0)
        slot = project_slot(
            SlotTarget(1, 2.0, 0.0, 0.05),
            PinholeIntrinsics(100.0, 100.0, 100.0, 100.0),
            rotation,
            translation,
            surface_z=0.0,
            image_shape=(200, 200),
            half_size=3,
        )
        self.assertIsNotNone(slot)
        self.assertEqual(slot.center_u, 100)
        self.assertGreater(slot.plane_depth, 2.0)

        depth = np.full((200, 200), slot.plane_depth, dtype=np.float32)
        depth[
            slot.center_v - 3 : slot.center_v + 4,
            slot.center_u - 3 : slot.center_u + 4,
        ] -= 0.08
        observation = detect_slots(
            depth, [slot], tray_plane_depth=0.0, minimum_height=0.04
        )[0]
        self.assertTrue(observation.occupied)

    def test_rejects_slot_outside_camera_view(self):
        slot = project_slot(
            SlotTarget(1, 1.0, 10.0, 0.0),
            PinholeIntrinsics(100.0, 100.0, 50.0, 50.0),
            np.eye(3),
            (0.0, 0.0, 0.0),
            surface_z=-1.0,
            image_shape=(100, 100),
            half_size=3,
        )
        self.assertIsNone(slot)

    def test_configuration_requires_one_xy_pair_per_slot(self):
        with self.assertRaisesRegex(ValueError, "x/y for every slot"):
            configured_slot_targets((1, 2), (0.7, -0.2), sample_z=0.25)

    def test_operator_summary_distinguishes_unknown_from_empty(self):
        message = TrayOccupancy()
        unknown = TraySlotState(slot_id=3, observable=False, occupied=False)
        empty = TraySlotState(slot_id=4, observable=True, occupied=False)
        occupied = TraySlotState(slot_id=2, observable=True, occupied=True)
        message.slots = [unknown, empty, occupied]
        self.assertEqual(
            summarize_slots(message),
            "3:unknown, 4:empty, 2:occupied",
        )


if __name__ == "__main__":
    unittest.main()
