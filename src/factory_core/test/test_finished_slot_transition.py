import unittest

from factory_core.finished_slot_perception import (
    SlotOccupancy,
    slot_has_stable_state,
)


def _frame(observable: bool, occupied: bool) -> dict[int, SlotOccupancy]:
    return {1: SlotOccupancy(1, observable, occupied)}


class FinishedSlotTransitionTests(unittest.TestCase):
    def test_three_observable_occupied_frames_confirm_placement(self):
        frames = tuple(_frame(True, True) for _ in range(3))
        self.assertTrue(
            slot_has_stable_state(
                frames, 1, occupied=True, required_observations=3
            )
        )

    def test_one_empty_frame_rejects_placement(self):
        frames = (_frame(True, True), _frame(True, False), _frame(True, True))
        self.assertFalse(
            slot_has_stable_state(
                frames, 1, occupied=True, required_observations=3
            )
        )

    def test_unobservable_frame_never_counts_as_occupied(self):
        frames = (_frame(True, True), _frame(False, True), _frame(True, True))
        self.assertFalse(
            slot_has_stable_state(
                frames, 1, occupied=True, required_observations=3
            )
        )


if __name__ == "__main__":
    unittest.main()
