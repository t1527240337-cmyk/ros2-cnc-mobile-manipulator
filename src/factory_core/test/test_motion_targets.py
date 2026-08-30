import math
import unittest

from factory_core.manipulation_evidence import Point3, Quaternion
from factory_core.manipulate_part_server import (
    aligned_grasp_poses_from_measured_tcp,
    arm_joint_sample_is_stable,
    DEFAULT_MAX_ANCHOR_ALIGNMENT,
    LOADED_CARRY_X,
    FINISHED_BIN_SEATING_STEP,
    MAX_FINISHED_BIN_SEATING_DEPTH,
    MAX_GUARDED_SUPPORT_HORIZONTAL_ERROR,
    MAX_GUARDED_SUPPORT_VERTICAL_ERROR,
    MAX_MACHINE_FIXTURE_REFINEMENT,
    MAX_GRASP_SETTLING_POSITION_CHANGE,
    MAX_GRASP_SETTLING_VELOCITY,
    MACHINE_APPROACH_TRANSIT_HEIGHT,
    SUPPORTED_RELEASE_SEPARATION_DISTANCE,
    FINGERTIP_V_FACE_LENGTH,
    FINGERTIP_V_FACE_PITCH,
    FINGERTIP_V_FACE_THICKNESS,
    SUPPORTED_RELEASE_CLEARANCE,
    MACHINE_DOOR_CLEAR_X,
    MACHINE_FOLD_CLEARANCE_Y,
    MACHINE_FOLD_CLEARANCE_Z,
    finished_bin_seating_depths,
    guarded_support_pose_is_safe,
    machine_clearance_waypoints,
    machine_egress_free_space_planner,
    machine_seating_depths,
    MACHINE_SEATING_STEP,
    MAX_MACHINE_SEATING_DEPTH,
    planner_attempts,
    released_part_retention_is_valid,
    supported_part_center_from_tcp,
    supported_release_separation_distance,
    resolve_machine_target_position,
    resolve_observed_machine_part_center,
    uses_direct_bin_place_transit,
    uses_direct_pick_transit,
    tcp_orientation_error,
    tcp_position_error,
)
from factory_core.manipulation_config import CartesianPose
from factory_core.motion_client import (
    JOINT_ACCELERATION_SCALE,
    JOINT_VELOCITY_SCALE,
    JointTarget,
)
from factory_core.pose_motion_client import (
    EMPTY_LINEAR_ACCELERATION_SCALE,
    EMPTY_LINEAR_VELOCITY_SCALE,
    EMPTY_OMPL_ACCELERATION_SCALE,
    EMPTY_OMPL_VELOCITY_SCALE,
    EMPTY_PTP_ACCELERATION_SCALE,
    EMPTY_PTP_VELOCITY_SCALE,
    FIXTURE_EMPTY_LINEAR_ACCELERATION_SCALE,
    FIXTURE_EMPTY_LINEAR_VELOCITY_SCALE,
    FIXTURE_INSERTION_LINEAR_ACCELERATION_SCALE,
    FIXTURE_INSERTION_LINEAR_VELOCITY_SCALE,
    LOADED_EGRESS_LINEAR_ACCELERATION_SCALE,
    LOADED_EGRESS_LINEAR_VELOCITY_SCALE,
    LOADED_LINEAR_ACCELERATION_SCALE,
    LOADED_LINEAR_VELOCITY_SCALE,
    LOADED_OMPL_ACCELERATION_SCALE,
    LOADED_OMPL_VELOCITY_SCALE,
    LOADED_PTP_ACCELERATION_SCALE,
    LOADED_PTP_VELOCITY_SCALE,
    LOADED_TRANSPORT_LINEAR_ACCELERATION_SCALE,
    LOADED_TRANSPORT_LINEAR_VELOCITY_SCALE,
    PROOF_LINEAR_ACCELERATION_SCALE,
    PROOF_LINEAR_VELOCITY_SCALE,
    PoseTarget,
)


class MotionTargetTests(unittest.TestCase):
    def test_anchor_alignment_uses_measured_tcp_and_grasp_vector(self):
        approach = CartesianPose(
            frame_id="base_link",
            position=(0.62, 0.05, 0.24),
            orientation=(0.5, 0.5, 0.5, 0.5),
        )
        grasp = CartesianPose(
            frame_id="base_link",
            position=(0.72, 0.05, 0.24),
            orientation=(0.5, 0.5, 0.5, 0.5),
        )
        measured_orientation = Quaternion(0.48, 0.51, 0.49, 0.52)

        aligned_approach, aligned_grasp = (
            aligned_grasp_poses_from_measured_tcp(
                Point3(0.603, 0.052, 0.243),
                measured_orientation,
                Point3(0.017, -0.002, 0.003),
                approach,
                grasp,
            )
        )

        for actual, expected in zip(
            aligned_approach.position, (0.62, 0.05, 0.246), strict=True
        ):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(
            aligned_grasp.position, (0.72, 0.05, 0.246), strict=True
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(
            aligned_approach.orientation, (0.48, 0.51, 0.49, 0.52)
        )
        self.assertEqual(
            aligned_grasp.orientation, aligned_approach.orientation
        )

    def test_machine_seating_search_is_monotonic_and_bounded(self):
        depths = machine_seating_depths()

        self.assertEqual(depths[0], MACHINE_SEATING_STEP)
        self.assertAlmostEqual(depths[-1], MAX_MACHINE_SEATING_DEPTH)
        self.assertAlmostEqual(MACHINE_SEATING_STEP, 0.010)
        self.assertAlmostEqual(MAX_MACHINE_SEATING_DEPTH, 0.060)
        self.assertLessEqual(MAX_MACHINE_SEATING_DEPTH, 0.060)
        self.assertTrue(all(left < right for left, right in zip(depths, depths[1:])))
        self.assertTrue(all(depth <= MAX_MACHINE_SEATING_DEPTH for depth in depths))

    def test_finished_bin_contact_search_is_monotonic_and_bounded(self):
        depths = finished_bin_seating_depths()

        self.assertEqual(depths[0], FINISHED_BIN_SEATING_STEP)
        self.assertAlmostEqual(depths[-1], MAX_FINISHED_BIN_SEATING_DEPTH)
        self.assertTrue(
            all(left < right for left, right in zip(depths, depths[1:]))
        )
        self.assertAlmostEqual(MAX_FINISHED_BIN_SEATING_DEPTH, 0.060)
        self.assertLessEqual(MAX_FINISHED_BIN_SEATING_DEPTH, 0.060)

    def test_guarded_support_stop_is_bounded_around_place_target(self):
        target = (0.815, 0.010, 0.945)

        self.assertTrue(
            guarded_support_pose_is_safe(
                Point3(0.815, 0.010, 0.965), target
            )
        )
        self.assertFalse(
            guarded_support_pose_is_safe(
                Point3(
                    target[0] + MAX_GUARDED_SUPPORT_HORIZONTAL_ERROR + 0.001,
                    target[1],
                    target[2],
                ),
                target,
            )
        )
        self.assertFalse(
            guarded_support_pose_is_safe(
                Point3(
                    target[0],
                    target[1],
                    target[2] + MAX_GUARDED_SUPPORT_VERTICAL_ERROR + 0.001,
                ),
                target,
            )
        )

    def test_grasp_settling_limits_remain_bounded(self):
        self.assertLess(MAX_GRASP_SETTLING_POSITION_CHANGE, 0.010)
        self.assertLessEqual(MAX_GRASP_SETTLING_VELOCITY, 0.020)

    def test_joint_target_requires_six_positions(self):
        with self.assertRaises(ValueError):
            JointTarget("bad", (0.0,)).validate()

    def test_pose_target_requires_named_frame_and_xyzw(self):
        with self.assertRaises(ValueError):
            PoseTarget("pick", "", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)).validate()
        with self.assertRaises(ValueError):
            PoseTarget("pick", "map", (0.0, 0.0, 0.0), (0.0, 1.0)).validate()

    def test_loaded_linear_motion_is_bounded_but_not_timeout_slow(self):
        self.assertGreater(LOADED_LINEAR_VELOCITY_SCALE, 0.015)
        self.assertLessEqual(
            LOADED_LINEAR_VELOCITY_SCALE, EMPTY_LINEAR_VELOCITY_SCALE
        )
        self.assertLessEqual(EMPTY_LINEAR_VELOCITY_SCALE, 0.68)
        self.assertLessEqual(EMPTY_LINEAR_ACCELERATION_SCALE, 0.22)
        self.assertLessEqual(LOADED_LINEAR_VELOCITY_SCALE, 0.45)
        self.assertLessEqual(LOADED_LINEAR_ACCELERATION_SCALE, 0.18)
        self.assertGreater(LOADED_LINEAR_ACCELERATION_SCALE, 0.005)
        self.assertLess(
            LOADED_LINEAR_ACCELERATION_SCALE, EMPTY_LINEAR_ACCELERATION_SCALE
        )
        self.assertLessEqual(
            FIXTURE_INSERTION_LINEAR_VELOCITY_SCALE,
            LOADED_LINEAR_VELOCITY_SCALE,
        )
        self.assertLessEqual(FIXTURE_INSERTION_LINEAR_VELOCITY_SCALE, 0.40)
        self.assertLess(
            FIXTURE_INSERTION_LINEAR_ACCELERATION_SCALE,
            LOADED_LINEAR_ACCELERATION_SCALE,
        )
        self.assertLessEqual(
            FIXTURE_INSERTION_LINEAR_ACCELERATION_SCALE, 0.14
        )
        self.assertGreater(
            FIXTURE_INSERTION_LINEAR_ACCELERATION_SCALE,
            PROOF_LINEAR_ACCELERATION_SCALE,
        )
        self.assertGreater(PROOF_LINEAR_VELOCITY_SCALE, 0.05)
        self.assertLess(
            PROOF_LINEAR_VELOCITY_SCALE, LOADED_LINEAR_VELOCITY_SCALE
        )
        self.assertGreater(PROOF_LINEAR_ACCELERATION_SCALE, 0.02)
        self.assertLess(
            PROOF_LINEAR_ACCELERATION_SCALE, LOADED_LINEAR_ACCELERATION_SCALE
        )
        self.assertLessEqual(PROOF_LINEAR_VELOCITY_SCALE, 0.20)
        self.assertLessEqual(PROOF_LINEAR_ACCELERATION_SCALE, 0.08)
        self.assertGreater(
            LOADED_TRANSPORT_LINEAR_VELOCITY_SCALE,
            LOADED_LINEAR_VELOCITY_SCALE,
        )
        self.assertLessEqual(LOADED_TRANSPORT_LINEAR_VELOCITY_SCALE, 0.45)
        self.assertGreater(
            LOADED_TRANSPORT_LINEAR_ACCELERATION_SCALE,
            LOADED_LINEAR_ACCELERATION_SCALE,
        )
        self.assertLessEqual(LOADED_TRANSPORT_LINEAR_ACCELERATION_SCALE, 0.18)
        self.assertLessEqual(
            LOADED_EGRESS_LINEAR_VELOCITY_SCALE,
            LOADED_TRANSPORT_LINEAR_VELOCITY_SCALE,
        )
        self.assertLess(
            LOADED_EGRESS_LINEAR_ACCELERATION_SCALE,
            LOADED_TRANSPORT_LINEAR_ACCELERATION_SCALE,
        )
        self.assertLessEqual(LOADED_PTP_VELOCITY_SCALE, 0.70)
        self.assertLessEqual(LOADED_PTP_ACCELERATION_SCALE, 0.50)
        self.assertGreater(LOADED_PTP_VELOCITY_SCALE, 0.0)
        self.assertGreater(LOADED_PTP_ACCELERATION_SCALE, 0.0)
        self.assertLessEqual(LOADED_OMPL_VELOCITY_SCALE, 0.65)
        self.assertLessEqual(LOADED_OMPL_ACCELERATION_SCALE, 0.50)
        self.assertGreater(LOADED_OMPL_VELOCITY_SCALE, 0.0)
        self.assertGreater(LOADED_OMPL_ACCELERATION_SCALE, 0.0)
        self.assertLessEqual(JOINT_VELOCITY_SCALE, 0.85)
        self.assertLessEqual(JOINT_ACCELERATION_SCALE, 0.70)
        self.assertLessEqual(EMPTY_PTP_VELOCITY_SCALE, 0.80)
        self.assertLessEqual(EMPTY_PTP_ACCELERATION_SCALE, 0.65)
        self.assertLessEqual(EMPTY_OMPL_VELOCITY_SCALE, 0.70)
        self.assertLessEqual(EMPTY_OMPL_ACCELERATION_SCALE, 0.55)

    def test_empty_cnc_linear_motion_keeps_speed_and_acceleration_margin(self):
        self.assertEqual(
            FIXTURE_EMPTY_LINEAR_VELOCITY_SCALE,
            EMPTY_LINEAR_VELOCITY_SCALE,
        )
        self.assertLess(
            FIXTURE_EMPTY_LINEAR_ACCELERATION_SCALE,
            EMPTY_LINEAR_ACCELERATION_SCALE,
        )
        self.assertLessEqual(FIXTURE_EMPTY_LINEAR_ACCELERATION_SCALE, 0.15)

    def test_tcp_position_error_is_euclidean_distance(self):
        error = tcp_position_error(
            Point3(0.01, -0.02, 0.03), (0.01, -0.02, 0.05)
        )
        self.assertAlmostEqual(error, 0.02)

    def test_tcp_orientation_error_treats_quaternion_signs_as_equal(self):
        self.assertAlmostEqual(
            tcp_orientation_error(
                Quaternion(0.0, 0.0, 0.0, -1.0),
                (0.0, 0.0, 0.0, 1.0),
            ),
            0.0,
        )

    def test_bin_coarse_approach_has_collision_checked_ptp_candidate(self):
        self.assertEqual(
            planner_attempts("bin_approach_lin"),
            ((1, "lin"), (2, "ptp")),
        )

    def test_loaded_carry_has_deterministic_ptp_candidate(self):
        self.assertEqual(
            planner_attempts("loaded_ompl"),
            ((1, "loaded_ompl"), (2, "loaded_ptp")),
        )

    def test_loaded_ptp_falls_back_to_collision_checked_sampling(self):
        self.assertEqual(
            planner_attempts("loaded_ptp"),
            ((1, "loaded_ptp"), (2, "loaded_ompl")),
        )

    def test_loaded_transport_prefers_orientation_preserving_linear_path(self):
        self.assertEqual(
            planner_attempts("loaded_transport_lin"),
            ((1, "loaded_transport_lin"), (2, "loaded_ptp")),
        )
        self.assertEqual(
            planner_attempts("loaded_egress_lin"),
            ((1, "loaded_egress_lin"), (2, "loaded_egress_lin")),
        )

    def test_loaded_bin_place_uses_its_safe_station_transit(self):
        self.assertTrue(uses_direct_bin_place_transit("raw_bin"))
        self.assertTrue(uses_direct_bin_place_transit("finished_bin"))
        self.assertFalse(uses_direct_bin_place_transit("machine_2"))
        self.assertFalse(uses_direct_bin_place_transit("charge_dock"))

    def test_only_machine_pick_skips_the_collision_clearing_unfold(self):
        self.assertTrue(uses_direct_pick_transit("machine_2", "machine"))
        self.assertFalse(uses_direct_pick_transit("raw_bin", "source"))
        self.assertFalse(uses_direct_pick_transit("finished_bin", "sink"))
        self.assertFalse(
            uses_direct_pick_transit("inspection_table", "inspection")
        )

    def test_loaded_carry_keeps_stock_clear_of_forearm(self):
        self.assertGreaterEqual(LOADED_CARRY_X, 0.40)

    def test_machine_entry_and_exit_share_one_clearance_corridor(self):
        from factory_core.manipulation_config import CartesianPose

        transit = CartesianPose(
            frame_id="base_link",
            position=(0.88, -0.02, 1.25),
            orientation=(0.5, 0.5, 0.5, 0.5),
        )

        door, fold, travel = machine_clearance_waypoints(transit)

        self.assertEqual(door.position, (MACHINE_DOOR_CLEAR_X, -0.02, 1.25))
        self.assertEqual(
            fold.position,
            (
                MACHINE_DOOR_CLEAR_X,
                MACHINE_FOLD_CLEARANCE_Y,
                MACHINE_FOLD_CLEARANCE_Z,
            ),
        )
        self.assertEqual(travel.position[0], LOADED_CARRY_X)
        self.assertEqual(door.orientation, transit.orientation)
        self.assertEqual(fold.orientation, transit.orientation)
        self.assertEqual(travel.orientation, transit.orientation)

    def test_machine_egress_crosses_front_plane_before_free_motion(self):
        lintel_near_face_x = 0.64 - 0.10 / 2.0
        self.assertLess(MACHINE_DOOR_CLEAR_X, lintel_near_face_x)
        self.assertGreater(MACHINE_DOOR_CLEAR_X, LOADED_CARRY_X)

    def test_machine_egress_uses_lin_only_until_clear_of_door(self):
        self.assertEqual(machine_egress_free_space_planner("lin"), "ptp")
        self.assertEqual(
            machine_egress_free_space_planner("fixture_empty_lin"), "ptp"
        )
        self.assertEqual(
            machine_egress_free_space_planner("loaded_transport_lin"),
            "loaded_ptp",
        )
        self.assertEqual(
            machine_egress_free_space_planner("fixture_loaded_lin"),
            "loaded_ptp",
        )

    def test_machine_fold_waypoint_stays_inside_reachable_ik_grid(self):
        self.assertLessEqual(abs(MACHINE_FOLD_CLEARANCE_Y), 0.20)
        self.assertGreaterEqual(MACHINE_FOLD_CLEARANCE_Z, 1.10)
        self.assertLessEqual(MACHINE_FOLD_CLEARANCE_Z, 1.25)

    def test_machine_transit_clears_fixture_without_using_reach_edge(self):
        fixture_top_with_stock = 1.10
        former_unreliable_reach_edge = 1.25
        self.assertGreater(
            MACHINE_APPROACH_TRANSIT_HEIGHT, fixture_top_with_stock
        )
        self.assertLess(
            MACHINE_APPROACH_TRANSIT_HEIGHT, former_unreliable_reach_edge
        )
        self.assertEqual(
            MACHINE_APPROACH_TRANSIT_HEIGHT, MACHINE_FOLD_CLEARANCE_Z
        )

    def test_anchor_refinement_is_bounded_below_global_safety_limit(self):
        self.assertGreater(DEFAULT_MAX_ANCHOR_ALIGNMENT, 0.10)
        self.assertLessEqual(DEFAULT_MAX_ANCHOR_ALIGNMENT, 0.15)
        self.assertLessEqual(
            MAX_MACHINE_FIXTURE_REFINEMENT,
            DEFAULT_MAX_ANCHOR_ALIGNMENT,
        )

    def test_machine_tag_refinement_keeps_surveyed_fixture_height(self):
        resolved = resolve_machine_target_position(
            Point3(0.86, -0.01, 0.86),
            (0.85, 0.0, 0.92),
        )

        self.assertEqual(resolved, (0.86, -0.01, 0.92))

    def test_machine_tag_refinement_rejects_a_large_disagreement(self):
        configured = (0.85, 0.0, 0.92)

        with self.assertRaisesRegex(ValueError, "differs from calibration"):
            resolve_machine_target_position(
                Point3(0.954, 0.018, 0.86),
                configured,
            )

    def test_arm_replan_settle_requires_all_joints_stationary(self):
        self.assertTrue(
            arm_joint_sample_is_stable(
                (0.0, 0.1, -0.2),
                (0.001, 0.099, -0.199),
                tolerance=0.002,
            )
        )
        self.assertFalse(
            arm_joint_sample_is_stable(
                (0.0, 0.1, -0.2),
                (0.001, 0.104, -0.199),
                tolerance=0.002,
            )
        )

    def test_arm_replan_settle_rejects_malformed_samples(self):
        with self.assertRaisesRegex(ValueError, "equally sized"):
            arm_joint_sample_is_stable((0.0,), (0.0, 0.1))
        with self.assertRaisesRegex(ValueError, "must be positive"):
            arm_joint_sample_is_stable((0.0,), (0.0,), tolerance=0.0)

    def test_machine_part_center_uses_all_three_rgbd_coordinates(self):
        resolved = resolve_observed_machine_part_center(
            Point3(0.814, 0.010, 0.920),
            (0.820, 0.0, 0.971),
            maximum_vertical_distance=0.08,
        )

        self.assertEqual(resolved, (0.814, 0.010, 0.920))

    def test_machine_part_center_rejects_height_outside_work_volume(self):
        with self.assertRaisesRegex(
            ValueError, "differs from its safe work volume"
        ):
            resolve_observed_machine_part_center(
                Point3(0.814, 0.010, 0.870),
                (0.820, 0.0, 0.971),
                maximum_vertical_distance=0.08,
            )

    def test_machine_part_center_rejects_invalid_vertical_limit(self):
        with self.assertRaisesRegex(ValueError, "must be positive"):
            resolve_observed_machine_part_center(
                Point3(0.814, 0.010, 0.920),
                (0.820, 0.0, 0.971),
                maximum_vertical_distance=0.0,
            )

    def test_machine_release_requires_confirmed_fixture_clamp(self):
        self.assertTrue(
            released_part_retention_is_valid(
                "machine_3", support_contact=False, fixture_clamped=True
            )
        )
        self.assertFalse(
            released_part_retention_is_valid(
                "machine_3", support_contact=True, fixture_clamped=False
            )
        )

    def test_unclamped_destination_requires_fresh_support(self):
        self.assertTrue(
            released_part_retention_is_valid(
                "finished_bin", support_contact=True, fixture_clamped=None
            )
        )
        self.assertFalse(
            released_part_retention_is_valid(
                "finished_bin", support_contact=False, fixture_clamped=None
            )
        )


    def test_supported_release_clears_both_v_jaw_faces(self):
        projected_half_height = 0.5 * (
            FINGERTIP_V_FACE_LENGTH * math.cos(FINGERTIP_V_FACE_PITCH)
            + FINGERTIP_V_FACE_THICKNESS * math.sin(FINGERTIP_V_FACE_PITCH)
        )
        lowest_pad_point_after_release = (
            0.005
            + SUPPORTED_RELEASE_SEPARATION_DISTANCE
            - projected_half_height
        )

        self.assertGreaterEqual(
            lowest_pad_point_after_release,
            0.5 * 0.12 + SUPPORTED_RELEASE_CLEARANCE,
        )
        self.assertGreater(SUPPORTED_RELEASE_SEPARATION_DISTANCE, 0.070)

    def test_supported_release_uses_measured_grasp_offset(self):
        shallow_grasp = supported_release_separation_distance(0.0)
        nominal_grasp = supported_release_separation_distance(0.005)

        self.assertAlmostEqual(shallow_grasp - nominal_grasp, 0.005)

    def test_supported_release_rejects_invalid_grasp_offset(self):
        with self.assertRaisesRegex(ValueError, "cannot be negative"):
            supported_release_separation_distance(-0.001)
        with self.assertRaisesRegex(ValueError, "must be finite"):
            supported_release_separation_distance(math.nan)

    def test_supported_part_scene_uses_measured_contact_stop(self):
        center = supported_part_center_from_tcp(
            Point3(0.720, -0.200, 0.272), 0.005
        )

        self.assertEqual(center, (0.720, -0.200, 0.267))
        self.assertNotEqual(center[2], 0.315)

    def test_supported_part_center_rejects_invalid_grasp_offset(self):
        with self.assertRaisesRegex(ValueError, "cannot be negative"):
            supported_part_center_from_tcp(
                Point3(0.720, -0.200, 0.272), -0.001
            )
        with self.assertRaisesRegex(ValueError, "must be finite"):
            supported_part_center_from_tcp(
                Point3(0.720, -0.200, 0.272), math.nan
            )


if __name__ == "__main__":
    unittest.main()
