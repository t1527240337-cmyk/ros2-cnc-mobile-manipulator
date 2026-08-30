import unittest

import numpy as np

from factory_perception.sparse_bin import (
    CameraIntrinsics,
    Region3D,
    detect_sparse_parts,
    fit_upright_cylinder_center,
    quaternion_rotation_matrix,
)


class SparseBinDetectorTests(unittest.TestCase):
    def test_detects_two_separated_components(self):
        depth = np.full((30, 40), np.nan, dtype=np.float32)
        depth[8:14, 7:13] = 1.0
        depth[17:24, 26:33] = 1.0

        candidates = detect_sparse_parts(
            depth,
            CameraIntrinsics(100.0, 100.0, 20.0, 15.0),
            np.eye(3),
            (0.0, 0.0, 0.0),
            Region3D((-0.25, -0.15, 0.8), (0.25, 0.15, 1.2)),
            minimum_component_pixels=10,
        )

        self.assertEqual(len(candidates), 2)
        self.assertGreater(candidates[0].y, candidates[1].y)
        self.assertTrue(
            all(candidate.pixel_count >= 36 for candidate in candidates)
        )

    def test_rejects_merged_oversized_component(self):
        depth = np.full((20, 50), np.nan, dtype=np.float32)
        depth[8:13, 5:46] = 1.0

        candidates = detect_sparse_parts(
            depth,
            CameraIntrinsics(100.0, 100.0, 25.0, 10.0),
            np.eye(3),
            (0.0, 0.0, 0.0),
            Region3D((-0.30, -0.20, 0.8), (0.30, 0.20, 1.2)),
            minimum_component_pixels=10,
            maximum_component_span=0.14,
        )

        self.assertEqual(candidates, [])

    def test_fits_axis_from_partial_upright_cylinder_side_wall(self):
        center = np.asarray((0.84, -0.012))
        radius = 0.025
        angles = np.linspace(-1.1, 1.1, 80)
        levels = np.linspace(0.90, 0.99, 5)
        points = np.asarray(
            [
                (
                    center[0] + radius * np.cos(angle),
                    center[1] + radius * np.sin(angle),
                    level,
                )
                for level in levels
                for angle in angles
            ]
        )

        fitted = fit_upright_cylinder_center(
            points, expected_radius=radius, radius_tolerance=0.002
        )

        self.assertIsNotNone(fitted)
        np.testing.assert_allclose(fitted, center, atol=1e-6)

    def test_single_layer_support_height_overrides_occluded_depth_median(self):
        depth = np.full((30, 40), np.nan, dtype=np.float32)
        depth[8:14, 7:13] = 1.0

        candidates = detect_sparse_parts(
            depth,
            CameraIntrinsics(100.0, 100.0, 20.0, 15.0),
            np.eye(3),
            (0.0, 0.0, 0.0),
            Region3D((-0.25, -0.15, 0.8), (0.25, 0.15, 1.2)),
            minimum_component_pixels=10,
            supported_center_height=0.95,
        )

        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0].z, 0.95)

    def test_supported_height_must_be_inside_work_volume(self):
        with self.assertRaisesRegex(ValueError, "inside the detection region"):
            detect_sparse_parts(
                np.ones((3, 3), dtype=np.float32),
                CameraIntrinsics(100.0, 100.0, 1.0, 1.0),
                np.eye(3),
                (0.0, 0.0, 0.0),
                Region3D((-0.1, -0.1, 0.8), (0.1, 0.1, 1.2)),
                supported_center_height=1.3,
            )

    def test_rejects_component_with_wrong_cylinder_radius(self):
        center = np.asarray((0.84, 0.0))
        angles = np.linspace(-1.0, 1.0, 40)
        points = np.asarray(
            [
                (
                    center[0] + 0.06 * np.cos(angle),
                    center[1] + 0.06 * np.sin(angle),
                    0.93 + 0.001 * index,
                )
                for index, angle in enumerate(angles)
            ]
        )

        self.assertIsNone(
            fit_upright_cylinder_center(
                points, expected_radius=0.025, radius_tolerance=0.005
            )
        )

    def test_quaternion_matrix_rotates_x_to_y(self):
        half_turn = np.sqrt(0.5)
        rotation = quaternion_rotation_matrix(
            0.0, 0.0, half_turn, half_turn
        )
        rotated = rotation @ np.asarray((1.0, 0.0, 0.0))
        np.testing.assert_allclose(rotated, (0.0, 1.0, 0.0), atol=1e-7)


if __name__ == "__main__":
    unittest.main()
