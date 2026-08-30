from types import SimpleNamespace
import unittest

from factory_perception.dock_pose_node import select_target_detection


def detection(tag_id: int, margin: float, hamming: int = 0):
    return SimpleNamespace(
        id=tag_id,
        decision_margin=margin,
        hamming=hamming,
    )


class DockPoseSelectionTests(unittest.TestCase):
    def test_does_not_select_a_tag_until_a_target_is_requested(self):
        result = select_target_detection(
            [detection(10, 80.0)],
            {10, 11},
            None,
            minimum_margin=25.0,
        )
        self.assertIsNone(result)

    def test_selects_requested_tag_instead_of_highest_other_tag(self):
        result = select_target_detection(
            [detection(1, 95.0), detection(10, 55.0)],
            {1, 10},
            target_tag_id=10,
            minimum_margin=25.0,
        )
        self.assertEqual(result.id, 10)

    def test_rejects_corrected_or_low_quality_detection(self):
        result = select_target_detection(
            [detection(10, 80.0, hamming=1), detection(10, 20.0)],
            {10},
            target_tag_id=10,
            minimum_margin=25.0,
        )
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
