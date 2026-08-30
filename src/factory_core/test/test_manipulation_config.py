from pathlib import Path
import tempfile
import unittest

from factory_core.manipulation_config import (
    PICK_OPERATION,
    PLACE_OPERATION,
    load_manipulation_stations,
    resolve_manipulation_request,
)


CONFIG_PATH = Path(__file__).parents[1] / "config" / "manipulation.yaml"


class ManipulationConfigTests(unittest.TestCase):
    def test_raw_bin_has_a_perception_search_pose_not_source_slots(self):
        station = load_manipulation_stations(CONFIG_PATH)["raw_bin"]

        self.assertEqual(station.role, "source")
        self.assertEqual(station.source_selection_pose.frame_id, "base_link")
        self.assertEqual(
            station.source_selection_pose.position,
            (0.782, 0.0, 0.254),
        )
        self.assertEqual(station.placement_slots, {})

    def test_vertical_raw_approach_is_derived_from_detected_part_pose(self):
        station = load_manipulation_stations(CONFIG_PATH)["raw_bin"]
        search_reference = station.source_selection_pose

        nominal_grasp = search_reference.translated(station.grasp_offset)
        approach = nominal_grasp.translated(station.approach_offset)
        lift = nominal_grasp.translated(station.lift_offset)

        self.assertEqual(station.grasp_offset, (0.0, 0.0, 0.005))
        self.assertEqual(approach.position[:2], search_reference.position[:2])
        self.assertAlmostEqual(approach.position[2], 0.359)
        self.assertEqual(lift.position[:2], search_reference.position[:2])
        self.assertAlmostEqual(lift.position[2], 0.419)

    def test_source_orientation_keeps_gripper_closing_axis_horizontal(self):
        station = load_manipulation_stations(CONFIG_PATH)["raw_bin"]
        x, y, z, w = station.source_selection_pose.orientation

        closing_axis_world_z = 2.0 * (x * z - w * y)
        tool_axis_world_x = 2.0 * (x * z + w * y)
        tool_axis_world_y = 2.0 * (y * z - w * x)
        tool_axis_world_z = 1.0 - 2.0 * (x * x + y * y)

        self.assertAlmostEqual(closing_axis_world_z, 0.0, places=6)
        self.assertAlmostEqual(tool_axis_world_x, 1.0, places=6)
        self.assertAlmostEqual(tool_axis_world_y, 0.0, places=6)
        self.assertAlmostEqual(tool_axis_world_z, 0.0, places=6)

    def test_finished_bin_exposes_only_real_destination_slots(self):
        station = load_manipulation_stations(CONFIG_PATH)["finished_bin"]

        middle = station.require_placement_slot(2).pose
        redundant = station.require_placement_slot(4).pose
        self.assertEqual(middle.position, (0.72, 0.05, 0.315))
        self.assertEqual(redundant.position, (0.72, -0.32, 0.315))
        self.assertIsNone(station.source_selection_pose)
        self.assertIsNone(station.machine_workpiece_pose)

    def test_identical_cnc_stations_share_one_fixture_workpiece_pose(self):
        stations = load_manipulation_stations(CONFIG_PATH)

        workpiece_poses = {
            stations[f"machine_{index}"].machine_workpiece_pose.position
            for index in range(1, 4)
        }
        self.assertEqual(workpiece_poses, {(0.816, 0.0, 0.971)})
        for index in range(1, 4):
            station = stations[f"machine_{index}"]
            self.assertEqual(station.role, "machine")
            self.assertEqual(station.placement_slots, {})
            self.assertEqual(
                station.fixture_reference.frame_id,
                f"machine_{index}_tag",
            )
            self.assertEqual(
                station.fixture_reference.position,
                (0.365, -0.045, -0.240),
            )

    def test_raw_pick_accepts_any_traceable_business_part_id(self):
        stations = load_manipulation_stations(CONFIG_PATH)

        station, target = resolve_manipulation_request(
            stations,
            PICK_OPERATION,
            "raw_bin",
            "order-7:part:2",
        )

        self.assertEqual(station.name, "raw_bin")
        self.assertEqual(target.pose, station.source_selection_pose)
        self.assertEqual(target.placement_slot_id, 0)

    def test_finished_place_resolves_the_selected_destination(self):
        stations = load_manipulation_stations(CONFIG_PATH)

        station, target = resolve_manipulation_request(
            stations,
            PLACE_OPERATION,
            "finished_bin",
            "order-7:part:2",
            placement_slot_id=3,
        )

        self.assertEqual(station.name, "finished_bin")
        self.assertEqual(target.placement_slot_id, 3)
        self.assertEqual(
            target.pose,
            station.require_placement_slot(3).pose,
        )

    def test_source_pick_rejects_a_destination_slot(self):
        stations = load_manipulation_stations(CONFIG_PATH)

        with self.assertRaisesRegex(ValueError, "does not accept"):
            resolve_manipulation_request(
                stations,
                PICK_OPERATION,
                "raw_bin",
                "order-7:part:2",
                placement_slot_id=1,
            )

    def test_station_role_rejects_reversed_material_flow(self):
        stations = load_manipulation_stations(CONFIG_PATH)

        with self.assertRaisesRegex(ValueError, "Cannot place into source"):
            resolve_manipulation_request(
                stations,
                PLACE_OPERATION,
                "raw_bin",
                "raw_part_1",
            )
        with self.assertRaisesRegex(ValueError, "Cannot pick from sink"):
            resolve_manipulation_request(
                stations,
                PICK_OPERATION,
                "finished_bin",
                "raw_part_1",
            )

    def test_duplicate_destination_slot_is_rejected_before_motion(self):
        malformed = """
stations:
  finished_bin:
    role: sink
    approach_offset: [0, 0, 0.1]
    lift_offset: [0, 0, 0.2]
    grasp_offset: [0, 0, 0.01]
    placement_slots:
      - slot_id: 1
        pose: {frame_id: base_link, position: [0, 0, 0], orientation: [0, 0, 0, 1]}
      - slot_id: 1
        pose: {frame_id: base_link, position: [0, 0, 0], orientation: [0, 0, 0, 1]}
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(malformed, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "repeats placement slot 1"):
                load_manipulation_stations(path)

    def test_non_machine_cannot_define_a_fixture_reference(self):
        malformed = """
stations:
  raw_bin:
    role: source
    approach_offset: [0, 0, 0.1]
    lift_offset: [0, 0, 0.2]
    grasp_offset: [0, 0, 0.01]
    fixture_reference:
      frame_id: raw_bin_tag
      position: [0, 0, 0]
    source_selection_pose:
      {frame_id: base_link, position: [0, 0, 0], orientation: [0, 0, 0, 1]}
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad_reference.yaml"
            path.write_text(malformed, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "only valid for a machine"):
                load_manipulation_stations(path)


if __name__ == "__main__":
    unittest.main()
