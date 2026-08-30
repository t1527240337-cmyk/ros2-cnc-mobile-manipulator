import ast
from pathlib import Path
import tempfile
import math
import xml.etree.ElementTree as ET
import unittest

import yaml

from factory_core.dock_station import (
    BIN_HEADING_TOLERANCE,
    BIN_POSITION_TOLERANCE,
    BIN_USE_ODOMETRY_CORRECTIONS,
    goal_checker_for_station,
    uses_direct_machine_alignment,
)
from factory_core.navigation_client import load_station_poses


class NavigationConfigTests(unittest.TestCase):
    def test_loads_all_factory_staging_poses(self):
        config = Path(__file__).parents[1] / "config" / "stations.yaml"
        poses = load_station_poses(config)

        self.assertEqual(
            set(poses),
            {
                "raw_bin",
                "finished_bin",
                "machine_1",
                "machine_2",
                "machine_3",
                "charge_dock",
            },
        )
        self.assertAlmostEqual(poses["raw_bin"].x, -2.85)
        self.assertAlmostEqual(poses["finished_bin"].x, 2.85)
        self.assertAlmostEqual(poses["machine_1"].yaw, 1.5708)
        self.assertAlmostEqual(poses["machine_2"].y, 1.25)
        self.assertAlmostEqual(poses["machine_3"].y, 1.25)

    def test_rejects_malformed_staging_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "stations.yaml"
            config.write_text(
                "stations:\n"
                "  raw_bin:\n"
                "    type: non_charging\n"
                "    tag_id: 10\n"
                "    staging_pose: [1.0, 2.0]\n"
                "    dock_pose: [1.0, 2.0, 0.0]\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "staging_pose"):
                load_station_poses(config)

    def test_nav2_uses_separate_precise_and_bin_goal_checkers(self):
        config = (
            Path(__file__).parents[2]
            / "factory_bringup"
            / "config"
            / "nav2_params.yaml"
        )
        document = yaml.safe_load(config.read_text(encoding="utf-8"))
        controller = document["controller_server"]["ros__parameters"]
        precise_checker = controller["goal_checker"]
        bin_checker = controller["bin_staging_goal_checker"]
        docking = document["docking_server"]["ros__parameters"]

        self.assertEqual(precise_checker["xy_goal_tolerance"], 0.08)
        self.assertEqual(precise_checker["yaw_goal_tolerance"], 0.06)
        self.assertEqual(bin_checker["xy_goal_tolerance"], 0.08)
        self.assertEqual(bin_checker["yaw_goal_tolerance"], 0.12)
        self.assertLess(
            bin_checker["xy_goal_tolerance"],
            abs(docking["factory_bin_station"]["staging_x_offset"]),
        )

    def test_transit_and_docking_use_separate_speed_envelopes(self):
        config = (
            Path(__file__).parents[2]
            / "factory_bringup"
            / "config"
            / "nav2_params.yaml"
        )
        document = yaml.safe_load(config.read_text(encoding="utf-8"))
        controller = document["controller_server"]["ros__parameters"][
            "FollowPath"
        ]
        smoother = document["velocity_smoother"]["ros__parameters"]
        docking = document["docking_server"]["ros__parameters"]["controller"]

        self.assertEqual(controller["desired_linear_vel"], 0.55)
        self.assertEqual(smoother["max_velocity"][0], 0.55)
        self.assertEqual(controller["rotate_to_heading_angular_vel"], 0.60)
        self.assertEqual(smoother["max_velocity"][2], 0.60)
        self.assertEqual(controller["min_approach_linear_velocity"], 0.10)
        self.assertEqual(docking["v_linear_max"], 0.30)
        self.assertEqual(docking["v_angular_max"], 0.75)
        self.assertLess(
            docking["v_linear_max"], controller["desired_linear_vel"]
        )

    def test_charging_tag_calibration_matches_strict_acceptance(self):
        bringup = Path(__file__).parents[2] / "factory_bringup" / "config"
        configurations = (
            bringup / "nav2_params.yaml",
            bringup / "docking.yaml",
        )

        for config_path in configurations:
            with self.subTest(config=config_path.name):
                document = yaml.safe_load(
                    config_path.read_text(encoding="utf-8")
                )
                docking = document["docking_server"]["ros__parameters"]
                charger = docking["charging_station"]

                self.assertEqual(
                    charger["plugin"],
                    "opennav_docking::SimpleChargingDock",
                )
                self.assertLessEqual(
                    charger["docking_threshold"], 0.03
                )
                self.assertTrue(charger["use_battery_status"])
                dock_pose = docking["charge_dock"]["pose"]
                self.assertAlmostEqual(dock_pose[1], -3.087)

    def test_bin_work_stance_retains_physical_clearance(self):
        bringup = Path(__file__).parents[2] / "factory_bringup" / "config"
        documents = [
            yaml.safe_load((bringup / name).read_text(encoding="utf-8"))
            for name in ("nav2_params.yaml", "docking.yaml")
        ]
        plugins = [
            document["docking_server"]["ros__parameters"][
                "factory_bin_station"
            ]
            for document in documents
        ]

        for plugin in plugins:
            self.assertAlmostEqual(
                plugin["nominal_target_translation_x"], -0.532
            )
            self.assertAlmostEqual(
                plugin["external_detection_translation_x"], -0.57
            )
        self.assertEqual(plugins[0], plugins[1])

        # Geometry from factory.sdf and the robot URDF. The source-table
        # front plane is x=-3.9 m. At yaw=pi the base's +X electrode points
        # toward decreasing map X; its joint plus half collision length is
        # 0.502 + 0.064/2 = 0.534 m.
        reference_x = -3.85
        table_front_x = -3.90
        target_x = reference_x + plugins[0][
            "nominal_target_translation_x"
        ] * math.cos(math.pi)
        electrode_front_x = target_x - 0.534
        nominal_clearance = electrode_front_x - table_front_x
        worst_case_clearance = nominal_clearance - plugins[0][
            "max_refinement_translation"
        ]

        self.assertGreater(worst_case_clearance, 0.03)

    def test_auxiliary_rgbd_topics_are_bridged(self):
        bridge = (
            Path(__file__).parents[2]
            / "factory_bringup"
            / "config"
            / "gz_bridge.yaml"
        )
        topics = {
            entry["ros_topic_name"]
            for entry in yaml.safe_load(bridge.read_text(encoding="utf-8"))
        }

        self.assertTrue(
            {
                "/camera_aux/image_raw",
                "/camera_aux/depth/image_raw",
                "/camera_aux/camera_info",
            }.issubset(topics)
        )

    def test_perception_launch_nodes_are_independent_ros_actions(self):
        launch_file = (
            Path(__file__).parents[2]
            / "factory_bringup"
            / "launch"
            / "physical_stack.launch.py"
        )
        tree = ast.parse(launch_file.read_text(encoding="utf-8"))
        node_calls = [
            item
            for item in ast.walk(tree)
            if isinstance(item, ast.Call)
            and isinstance(item.func, ast.Name)
            and item.func.id == "Node"
        ]
        names = {
            keyword.value.value
            for call in node_calls
            for keyword in call.keywords
            if keyword.arg == "name"
            and isinstance(keyword.value, ast.Constant)
            and isinstance(keyword.value.value, str)
        }

        self.assertTrue(node_calls)
        self.assertTrue(all(not call.args for call in node_calls))
        self.assertTrue(
            {
                "sparse_bin_detector",
                "sparse_bin_detector_aux",
                "machine_fixture_detector",
            }.issubset(names)
        )

    def test_gazebo_camera_sensors_are_siblings_not_nested(self):
        xacro_file = (
            Path(__file__).parents[2]
            / "mobile_manipulator_description"
            / "urdf"
            / "mobile_manipulator.urdf.xacro"
        )
        root = ET.parse(xacro_file).getroot()
        sensors = list(root.iter("sensor"))

        self.assertFalse(
            any(sensor.find("gazebo") is not None for sensor in sensors),
            "a Gazebo sensor block must not be nested inside another sensor",
        )
        aux_containers = [
            gazebo
            for gazebo in root.iter("gazebo")
            if gazebo.attrib.get("reference") == "camera_aux_link"
            and any(
                sensor.attrib.get("name") == "aux_rgbd"
                for sensor in gazebo.findall("sensor")
            )
        ]
        self.assertEqual(
            len(aux_containers),
            1,
            "aux_rgbd must be a direct sensor of camera_aux_link",
        )

    def test_bin_alignment_keeps_project_acceptance_contract(self):
        self.assertEqual(BIN_POSITION_TOLERANCE, 0.030)
        self.assertLessEqual(BIN_HEADING_TOLERANCE, 0.0524)
        self.assertTrue(BIN_USE_ODOMETRY_CORRECTIONS)

    def test_gazebo_bridge_covers_the_complete_workpiece_pool(self):
        config = (
            Path(__file__).parents[2]
            / "factory_bringup"
            / "config"
            / "gz_bridge.yaml"
        )
        routes = yaml.safe_load(config.read_text(encoding="utf-8"))
        by_topic = {route["ros_topic_name"]: route for route in routes}

        for part_index in range(1, 7):
            prefix = f"/factory/fixture/raw_part_{part_index}"
            for command in ("attach", "detach"):
                route = by_topic[f"{prefix}/{command}"]
                self.assertEqual(route["direction"], "ROS_TO_GZ")
                self.assertEqual(route["ros_type_name"], "std_msgs/msg/Empty")
            state_route = by_topic[f"{prefix}/attached"]
            self.assertEqual(state_route["direction"], "GZ_TO_ROS")
            self.assertEqual(
                state_route["ros_type_name"],
                "std_msgs/msg/String",
            )

    def test_bins_use_coarse_staging_without_weakening_other_stations(self):
        self.assertEqual(
            goal_checker_for_station("raw_bin"),
            "bin_staging_goal_checker",
        )
        self.assertEqual(
            goal_checker_for_station("finished_bin"),
            "bin_staging_goal_checker",
        )
        self.assertEqual(
            goal_checker_for_station("machine_2"),
            "goal_checker",
        )
        self.assertEqual(
            goal_checker_for_station("charge_dock"),
            "goal_checker",
        )

    def test_factory_route_tree_always_selects_a_goal_checker(self):
        tree = (
            Path(__file__).parents[1]
            / "behavior_trees"
            / "navigate_through_factory_route.xml"
        ).read_text(encoding="utf-8")
        self.assertIn("<GoalCheckerSelector", tree)
        self.assertIn('default_goal_checker="goal_checker"', tree)
        self.assertIn(
            'goal_checker_id="{selected_goal_checker}"',
            tree,
        )

    def test_cnc_uses_one_continuous_visual_alignment_after_nav2(self):
        self.assertTrue(uses_direct_machine_alignment("machine_1"))
        self.assertTrue(uses_direct_machine_alignment("machine_3"))
        self.assertFalse(uses_direct_machine_alignment("raw_bin"))
        self.assertFalse(uses_direct_machine_alignment("finished_bin"))
        self.assertFalse(uses_direct_machine_alignment("charge_dock"))


if __name__ == "__main__":
    unittest.main()
