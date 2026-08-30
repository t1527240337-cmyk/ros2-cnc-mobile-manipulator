import unittest

import numpy as np

from factory_perception.slots import (
    PinholeIntrinsics,
    Slot,
    SlotTarget,
    detect_slots,
    detect_slots_in_point_cloud,
)


class SlotDetectorTests(unittest.TestCase):
    def test_detects_object_above_plane(self):
        depth = np.ones((20, 20), dtype=np.float32)
        depth[7:14, 7:14] = 0.94
        result = detect_slots(depth, [Slot(0, 10, 10, 3, 0.0, 0.0)], tray_plane_depth=1.0)
        self.assertTrue(result[0].occupied)
        self.assertAlmostEqual(result[0].height_above_tray, 0.06, places=4)

    def test_rejects_missing_depth(self):
        depth = np.full((10, 10), np.nan, dtype=np.float32)
        result = detect_slots(depth, [Slot(0, 5, 5, 2, 0.0, 0.0)], tray_plane_depth=1.0)
        self.assertFalse(result[0].occupied)

    def _detect_point_cloud_slot(self, depth):
        # A camera one metre above the tray looks straight down. The proper
        # 180-degree rotation has determinant +1 and maps shallower depth to
        # greater height in the output frame.
        return detect_slots_in_point_cloud(
            depth,
            PinholeIntrinsics(fx=100.0, fy=100.0, cx=50.0, cy=50.0),
            np.diag((1.0, -1.0, -1.0)),
            (0.0, 0.0, 1.0),
            (SlotTarget(1, 0.0, 0.0, 0.10),),
            surface_z=0.0,
            slot_half_size=0.07,
            minimum_height=0.025,
            maximum_height=0.18,
            minimum_region_points=30,
            minimum_object_points=12,
        )[0]

    def test_point_cloud_detects_small_object_without_majority_pixels(self):
        depth = np.ones((101, 101), dtype=np.float32)
        depth[47:54, 47:54] = 0.90

        result = self._detect_point_cloud_slot(depth)

        self.assertTrue(result.observable)
        self.assertTrue(result.occupied)
        self.assertAlmostEqual(result.height_above_tray, 0.10, places=3)

    def test_point_cloud_rejects_high_foreground_occluder(self):
        depth = np.ones((101, 101), dtype=np.float32)
        depth[45:56, 45:56] = 0.40

        result = self._detect_point_cloud_slot(depth)

        self.assertTrue(result.observable)
        self.assertFalse(result.occupied)

    def test_point_cloud_marks_missing_slot_unknown(self):
        depth = np.full((101, 101), np.nan, dtype=np.float32)

        result = self._detect_point_cloud_slot(depth)

        self.assertFalse(result.observable)
        self.assertFalse(result.occupied)


if __name__ == "__main__":
    unittest.main()
