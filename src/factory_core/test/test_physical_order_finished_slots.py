from types import SimpleNamespace
import threading
import unittest

from action_msgs.msg import GoalStatus

from factory_core.physical_order_executor import (
    PhysicalOrderError,
    PhysicalOrderExecutor,
)
from factory_core.physical_order_plan import Manipulation


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


class _FinishedSlots:
    def __init__(self):
        self.requests = []

    def wait_for_empty(
        self,
        preferred_slots,
        *,
        timeout_sec,
        include_recent_history=False,
    ):
        self.requests.append(
            (preferred_slots, timeout_sec, include_recent_history)
        )
        return preferred_slots[0]


class _Future:
    def __init__(self, value):
        self.value = value


class _ManipulationClient:
    def __init__(self, *, success):
        self.success = success
        self.goals = []

    def send_goal_async(self, goal):
        self.goals.append(goal)
        result = SimpleNamespace(
            success=self.success,
            error_code=0 if self.success else 7,
            message="placed" if self.success else "contact lost",
        )
        wrapped = SimpleNamespace(
            status=(
                GoalStatus.STATUS_SUCCEEDED
                if self.success
                else GoalStatus.STATUS_ABORTED
            ),
            result=result,
        )
        handle = SimpleNamespace(
            accepted=True,
            get_result_async=lambda: _Future(wrapped),
            cancel_goal_async=lambda: None,
        )
        return _Future(handle)


def _executor(*, manipulation_success):
    executor = PhysicalOrderExecutor.__new__(PhysicalOrderExecutor)
    executor._use_finished_slot_perception = True
    executor._finished_slot_order = (2, 1, 4, 3)
    executor._finished_slot_perception_timeout = 8.0
    executor._placed_finished_slots = {2}
    executor._state_lock = threading.RLock()
    executor._pending_finished_slot = None
    executor._finished_slots = _FinishedSlots()
    executor._manipulation = _ManipulationClient(
        success=manipulation_success
    )
    executor._wait_future = lambda future, *_args, **_kwargs: future.value
    executor._raise_if_cancelled = lambda _goal_handle: None
    executor.get_parameter = lambda _name: _Parameter(600.0)
    logger = _Logger()
    executor.get_logger = lambda: logger
    return executor, logger


def _finished_place_step():
    return SimpleNamespace(
        manipulation=Manipulation.PLACE,
        station_id="finished_bin",
        part_id="raw_part_2",
    )


class PhysicalOrderFinishedSlotTests(unittest.TestCase):
    def test_successful_place_reserves_selected_destination(self):
        executor, logger = _executor(manipulation_success=True)
        executor._preselect_finished_bin_slot(object())

        executor._manipulate_part(_finished_place_step(), object())

        self.assertEqual(
            executor._finished_slots.requests, [((1, 4, 3), 8.0, True)]
        )
        self.assertEqual(
            executor._manipulation.goals[0].placement_slot_id, 1
        )
        self.assertEqual(executor._placed_finished_slots, {1, 2})
        self.assertEqual(
            logger.messages[-1],
            "Reserved finished-bin slot 1 after physical placement",
        )
        self.assertIsNone(executor._pending_finished_slot)

    def test_preselection_uses_redundant_slot_after_two_placements(self):
        executor, _logger = _executor(manipulation_success=True)
        executor._placed_finished_slots = {1, 2}

        executor._preselect_finished_bin_slot(object())

        self.assertEqual(
            executor._finished_slots.requests, [((4, 3), 8.0, True)]
        )
        self.assertEqual(executor._pending_finished_slot, 4)

    def test_failed_place_does_not_reserve_destination(self):
        executor, _logger = _executor(manipulation_success=False)

        executor._preselect_finished_bin_slot(object())

        with self.assertRaises(PhysicalOrderError):
            executor._manipulate_part(_finished_place_step(), object())

        self.assertEqual(executor._placed_finished_slots, {2})
        self.assertEqual(executor._pending_finished_slot, 1)
