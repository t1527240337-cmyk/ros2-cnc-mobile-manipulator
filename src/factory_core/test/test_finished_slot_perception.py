import unittest

from factory_interfaces.msg import TrayOccupancy, TraySlotState

from factory_core.finished_slot_perception import (
    FinishedSlotPerception,
    SlotOccupancy,
    choose_stable_empty_slot,
    fresh_occupancy_observations,
    unreserved_slot_preferences,
)


class _FakeTime:
    def __init__(self, seconds: float):
        self.nanoseconds = int(seconds * 1.0e9)


class _FakeClock:
    def __init__(self, seconds: float):
        self.seconds = seconds

    def now(self):
        return _FakeTime(self.seconds)


class _FakeNode:
    def __init__(self, clock: _FakeClock):
        self._clock = clock

    def get_clock(self):
        return self._clock

    def create_subscription(self, *_args):
        return object()


def _frame(*states: tuple[int, bool, bool]) -> dict[int, SlotOccupancy]:
    return {
        slot_id: SlotOccupancy(slot_id, observable, occupied)
        for slot_id, observable, occupied in states
    }


class FinishedSlotSelectionTests(unittest.TestCase):
    def test_selects_first_preferred_slot_empty_in_all_frames(self):
        observations = (
            _frame((1, True, True), (2, True, False), (3, True, False)),
            _frame((1, True, True), (2, True, False), (3, True, False)),
            _frame((1, True, True), (2, True, False), (3, True, False)),
        )
        self.assertEqual(
            choose_stable_empty_slot(
                observations, (2, 1, 3), required_observations=3
            ),
            2,
        )

    def test_unknown_depth_is_not_treated_as_empty(self):
        observations = (
            _frame((1, False, False), (2, True, True)),
            _frame((1, False, False), (2, True, True)),
            _frame((1, False, False), (2, True, True)),
        )
        self.assertIsNone(
            choose_stable_empty_slot(
                observations, (1, 2), required_observations=3
            )
        )

    def test_one_frame_occupancy_blocks_the_slot(self):
        observations = (
            _frame((1, True, False), (2, True, False)),
            _frame((1, True, True), (2, True, False)),
            _frame((1, True, False), (2, True, False)),
        )
        self.assertEqual(
            choose_stable_empty_slot(
                observations, (1, 2), required_observations=3
            ),
            2,
        )

    def test_uses_redundant_slot_when_other_slots_are_not_safe(self):
        observations = tuple(
            _frame(
                (1, True, True),
                (2, True, True),
                (3, False, False),
                (4, True, False),
            )
            for _ in range(3)
        )
        self.assertEqual(
            choose_stable_empty_slot(
                observations, (2, 1, 4, 3), required_observations=3
            ),
            4,
        )

    def test_requires_post_request_fresh_frames(self):
        history = (
            (9.0, _frame((1, True, False))),
            (10.1, _frame((1, True, False))),
            (10.2, _frame((1, True, False))),
        )
        result = fresh_occupancy_observations(
            history, requested_at=10.0, now=10.3, maximum_age=1.0
        )
        self.assertEqual(len(result), 2)

    def test_received_observations_use_ros_time(self):
        clock = _FakeClock(12.5)
        selector = FinishedSlotPerception(_FakeNode(clock))
        message = TrayOccupancy()
        message.tray_id = "finished_bin"
        slot = TraySlotState()
        slot.slot_id = 1
        slot.observable = True
        slot.occupied = False
        message.slots.append(slot)

        selector._remember(message)

        received_at, states = selector._history[-1]
        self.assertEqual(received_at, 12.5)
        self.assertFalse(states[1].occupied)
        clock.seconds = 13.0
        self.assertEqual(selector._now(), 13.0)

    def test_can_select_recent_frames_captured_during_alignment(self):
        clock = _FakeClock(12.5)
        selector = FinishedSlotPerception(_FakeNode(clock))
        selector._history.extend(
            (
                (12.0, _frame((4, True, False))),
                (12.2, _frame((4, True, False))),
                (12.4, _frame((4, True, False))),
            )
        )

        self.assertEqual(
            selector.wait_for_empty(
                (4,), timeout_sec=0.1, include_recent_history=True
            ),
            4,
        )

    def test_reserved_slots_are_removed_without_reordering_candidates(self):
        self.assertEqual(
            unreserved_slot_preferences(
                (2, 1, 4, 3),
                frozenset({1, 2}),
            ),
            (4, 3),
        )

    def test_all_reserved_slots_returns_no_candidate(self):
        self.assertEqual(
            unreserved_slot_preferences((2, 1), {1, 2}),
            (),
        )


if __name__ == "__main__":
    unittest.main()
