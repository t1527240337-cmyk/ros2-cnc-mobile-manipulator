import math
import unittest

from factory_core.base_kinematics import (
    LocalMotion,
    PlanarPose,
    is_planar_motion_settled,
    local_motion,
    motion_error,
    normalize_angle,
)


class BaseKinematicsTests(unittest.TestCase):
    def test_local_motion_uses_start_heading(self):
        start = PlanarPose(1.0, 2.0, math.pi / 2.0)
        end = PlanarPose(1.0, 3.0, math.pi)

        motion = local_motion(start, end)

        self.assertAlmostEqual(motion.forward, 1.0)
        self.assertAlmostEqual(motion.lateral, 0.0, places=6)
        self.assertAlmostEqual(motion.yaw, math.pi / 2.0)

    def test_motion_error_wraps_yaw(self):
        reference = LocalMotion(1.0, 0.0, math.pi - 0.1)
        measured = LocalMotion(1.1, -0.1, -math.pi + 0.1)

        error = motion_error(reference, measured)

        self.assertAlmostEqual(error.forward, 0.1)
        self.assertAlmostEqual(error.lateral, -0.1)
        self.assertAlmostEqual(error.yaw, 0.2)
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi)

    def test_settle_requires_both_linear_and_angular_motion_to_stop(self):
        self.assertTrue(
            is_planar_motion_settled(
                0.003,
                0.004,
                0.010,
                linear_tolerance=0.005,
                angular_tolerance=0.010,
            )
        )
        self.assertFalse(
            is_planar_motion_settled(
                0.006,
                0.0,
                0.0,
                linear_tolerance=0.005,
                angular_tolerance=0.010,
            )
        )
        self.assertFalse(
            is_planar_motion_settled(
                0.0,
                0.0,
                0.011,
                linear_tolerance=0.005,
                angular_tolerance=0.010,
            )
        )


if __name__ == "__main__":
    unittest.main()
