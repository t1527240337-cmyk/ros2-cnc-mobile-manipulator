import unittest

from factory_core.physical_order_executor import PhysicalOrderExecutor


class _SilentLogger:
    def info(self, _message):
        pass


class _RecordingDock:
    def __init__(self, events):
        self._events = events

    def align_machine_tag(self, tag_frame, *, timeout_sec):
        self._events.append(("align_machine", tag_frame, timeout_sec))

    def align_bin_pose(self, tag_frame, *, timeout_sec):
        self._events.append(("align_bin", tag_frame, timeout_sec))


def _executor_with_recording_dependencies(*, perceive_finished_slots=False):
    events = []
    executor = PhysicalOrderExecutor.__new__(PhysicalOrderExecutor)
    executor._dock = _RecordingDock(events)
    executor._use_finished_slot_perception = perceive_finished_slots
    executor._departed_station_id = None
    executor.get_logger = lambda: _SilentLogger()
    executor._navigate_to_staging = (
        lambda station_id, goal_handle: events.append(
            ("navigate", station_id, goal_handle)
        )
    )
    executor._wait_for_base_settle = (
        lambda goal_handle: events.append(("settle", goal_handle))
    )
    executor._preselect_finished_bin_slot = (
        lambda goal_handle: events.append(("select", goal_handle))
    )
    return executor, events


class PhysicalOrderDockingFlowTests(unittest.TestCase):
    def test_machine_staging_settles_before_visual_alignment(self):
        executor, events = _executor_with_recording_dependencies()
        goal_handle = object()

        executor._dock_at_machine("machine_2", goal_handle)

        self.assertEqual(
            events,
            [
                ("navigate", "machine_2", goal_handle),
                ("settle", goal_handle),
                ("align_machine", "machine_2_tag", 75.0),
            ],
        )
        self.assertIsNone(executor._departed_station_id)

    def test_same_machine_reentry_uses_measured_retreat_corridor(self):
        executor, events = _executor_with_recording_dependencies()
        executor._departed_station_id = "machine_2"
        goal_handle = object()

        executor._dock_at_machine("machine_2", goal_handle)

        self.assertEqual(
            events,
            [
                ("settle", goal_handle),
                ("align_machine", "machine_2_tag", 75.0),
            ],
        )
        self.assertIsNone(executor._departed_station_id)

    def test_bin_staging_settles_before_visual_alignment(self):
        executor, events = _executor_with_recording_dependencies()
        goal_handle = object()

        executor._dock_at_bin("raw_bin", goal_handle)

        self.assertEqual(
            events,
            [
                ("navigate", "raw_bin", goal_handle),
                ("settle", goal_handle),
                ("align_bin", "raw_bin_tag", 180.0),
            ],
        )

    def test_finished_slot_is_selected_after_final_alignment(self):
        executor, events = _executor_with_recording_dependencies(
            perceive_finished_slots=True
        )
        goal_handle = object()

        executor._dock_at_bin("finished_bin", goal_handle)

        self.assertEqual(
            events,
            [
                ("navigate", "finished_bin", goal_handle),
                ("settle", goal_handle),
                ("align_bin", "finished_bin_tag", 180.0),
                ("select", goal_handle),
            ],
        )
