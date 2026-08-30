import math
import unittest

from factory_perception.unordered_layout import (
    WorkspaceBounds,
    sample_unordered_layout,
)


class UnorderedLayoutTests(unittest.TestCase):
    def test_every_formal_seed_fits_six_parts_without_slots(self):
        bounds = WorkspaceBounds(x=(-4.24, -4.04), y=(-3.00, -2.40))
        signatures = set()
        for seed in range(101, 131):
            points = sample_unordered_layout(
                seed=seed,
                count=6,
                bounds=bounds,
                minimum_center_distance=0.13,
            )
            self.assertEqual(len(points), 6)
            self.assertTrue(all(bounds.x[0] <= point.x <= bounds.x[1] for point in points))
            self.assertTrue(all(bounds.y[0] <= point.y <= bounds.y[1] for point in points))
            self.assertTrue(
                all(
                    math.hypot(first.x - second.x, first.y - second.y) >= 0.13
                    for index, first in enumerate(points)
                    for second in points[index + 1 :]
                )
            )
            signatures.add(tuple((round(point.x, 3), round(point.y, 3)) for point in points))
        self.assertEqual(len(signatures), 30)

    def test_same_seed_reproduces_the_same_layout(self):
        arguments = dict(
            seed=17,
            count=4,
            bounds=WorkspaceBounds(x=(-1.0, 1.0), y=(-2.0, 2.0)),
            minimum_center_distance=0.2,
        )
        self.assertEqual(
            sample_unordered_layout(**arguments),
            sample_unordered_layout(**arguments),
        )

    def test_impossible_layout_fails_instead_of_reducing_clearance(self):
        with self.assertRaisesRegex(ValueError, "could not generate"):
            sample_unordered_layout(
                seed=1,
                count=2,
                bounds=WorkspaceBounds(x=(0.0, 0.01), y=(0.0, 0.01)),
                minimum_center_distance=1.0,
                attempts_per_part=4,
                layout_restarts=2,
            )
