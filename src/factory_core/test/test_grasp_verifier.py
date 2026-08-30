import unittest

from ros_gz_interfaces.msg import Contact, Contacts

from factory_core.gazebo_truth_observer import (
    FrameTransform,
    Point3,
    Quaternion,
    check_attachment_distance,
    check_finger_contact_times,
    contact_pairs_for_part,
    check_position,
    check_upright,
    contacts_include_part,
    gripper_sides_touching_part,
    point_correction_in_parent,
    resolve_frame_origin,
    resolve_frame_rotation,
    transform_point,
)


class GraspVerifierTests(unittest.TestCase):
    def test_contact_filter_requires_the_requested_workpiece(self):
        message = Contacts()
        contact = Contact()
        contact.collision1.name = (
            "factory_robot::gripper_left_finger_tip_link::finger_pad_collision"
        )
        contact.collision2.name = "raw_part_2::part::collision"
        message.contacts = [contact]

        self.assertTrue(contacts_include_part(message, "raw_part_2"))
        self.assertFalse(contacts_include_part(message, "raw_part_1"))

    def test_contact_filter_rejects_empty_part_id(self):
        with self.assertRaisesRegex(ValueError, "part_id"):
            contacts_include_part(Contacts(), "")

    def test_contact_pair_diagnostic_keeps_only_requested_part(self):
        message = Contacts()
        requested = Contact()
        requested.collision1.name = "raw_part_2::part::collision"
        requested.collision2.name = "factory_robot::left_inner::collision"
        other = Contact()
        other.collision1.name = "raw_part_1::part::collision"
        other.collision2.name = "raw_bin::table::collision"
        message.contacts = [requested, other]

        self.assertEqual(
            contact_pairs_for_part(message, "raw_part_2"),
            ((requested.collision1.name, requested.collision2.name),),
        )

    def test_workpiece_contact_classifies_both_gripper_sides(self):
        message = Contacts()
        contacts = []
        for side in ("left", "right"):
            contact = Contact()
            contact.collision1.name = (
                f"factory_robot::gripper_{side}_finger_tip_link::"
                "finger_pad_collision"
            )
            contact.collision2.name = "raw_part_2::part::collision"
            contacts.append(contact)
        message.contacts = contacts

        self.assertEqual(
            gripper_sides_touching_part(message, "raw_part_2"),
            {"left", "right"},
        )

    def test_workpiece_contact_classifies_finger_body_collisions(self):
        message = Contacts()
        contacts = []
        for side in ("left", "right"):
            contact = Contact()
            contact.collision1.name = (
                f"factory_robot::gripper_{side}_finger_tip_link::"
                "finger_body_collision"
            )
            contact.collision2.name = "raw_part_2::part::collision"
            contacts.append(contact)
        message.contacts = contacts

        self.assertEqual(
            gripper_sides_touching_part(message, "raw_part_2"),
            {"left", "right"},
        )

    def test_workpiece_contact_ignores_table_and_other_parts(self):
        message = Contacts()
        contact = Contact()
        contact.collision1.name = "raw_bin::table::collision"
        contact.collision2.name = "raw_part_1::part::collision"
        message.contacts = [contact]

        self.assertEqual(
            gripper_sides_touching_part(message, "raw_part_2"), set()
        )

    def test_contact_confirmation_rejects_samples_before_close(self):
        check = check_finger_contact_times(
            9.9,
            10.1,
            now=10.2,
            max_age=1.0,
            received_after=10.0,
        )

        self.assertFalse(check.accepted)
        self.assertFalse(check.left)
        self.assertTrue(check.right)

    def test_contact_confirmation_accepts_fresh_post_close_samples(self):
        check = check_finger_contact_times(
            10.1,
            10.15,
            now=10.2,
            max_age=1.0,
            received_after=10.0,
        )

        self.assertTrue(check.accepted)

    def test_grasp_position_inside_independent_tolerances(self):
        check = check_position(
            Point3(1.03, 1.04, 0.57),
            Point3(1.00, 1.00, 0.50),
            max_horizontal_error=0.051,
            max_vertical_error=0.071,
        )

        self.assertTrue(check.accepted)
        self.assertEqual(round(check.horizontal_error, 3), 0.050)
        self.assertEqual(round(check.vertical_error, 3), 0.070)

    def test_grasp_position_rejects_horizontal_miss(self):
        check = check_position(
            Point3(1.08, 1.00, 0.54),
            Point3(1.00, 1.00, 0.50),
            max_horizontal_error=0.065,
            max_vertical_error=0.080,
        )

        self.assertFalse(check.accepted)
        self.assertIn("rejected", check.describe())

    def test_grasp_position_rejects_vertical_miss(self):
        check = check_position(
            Point3(1.02, 1.01, 0.60),
            Point3(1.00, 1.00, 0.50),
            max_horizontal_error=0.065,
            max_vertical_error=0.080,
        )

        self.assertFalse(check.accepted)

    def test_resolves_model_and_link_transform_chain(self):
        transforms = (
            FrameTransform(
                parent_frame="raw_part_2",
                child_frame="raw_part_2::part",
                translation=Point3(0.1, 0.0, 0.2),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
            FrameTransform(
                parent_frame="multi_machine_factory",
                child_frame="raw_part_2",
                translation=Point3(-4.0, -2.7, 0.5),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
        )

        position = resolve_frame_origin(
            transforms,
            "raw_part_2::part",
            root_frames=("multi_machine_factory",),
        )

        self.assertEqual(position, Point3(-3.9, -2.7, 0.7))

    def test_resolves_rotated_child_offset(self):
        transforms = (
            FrameTransform(
                parent_frame="world",
                child_frame="part",
                translation=Point3(1.0, 2.0, 3.0),
                rotation=Quaternion(
                    0.0,
                    0.0,
                    0.7071067811865476,
                    0.7071067811865476,
                ),
            ),
        )

        position = resolve_frame_origin(
            transforms,
            "part",
            root_frames=("world",),
        )
        self.assertIsNotNone(position)
        self.assertAlmostEqual(position.x, 1.0)
        self.assertAlmostEqual(position.y, 2.0)
        self.assertAlmostEqual(position.z, 3.0)

    def test_transforms_base_relative_slot_into_factory_truth(self):
        robot_pose = FrameTransform(
            parent_frame="multi_machine_factory",
            child_frame="factory_robot",
            translation=Point3(4.0, -3.0, 0.02),
            rotation=Quaternion(
                0.0,
                0.0,
                0.7071067811865476,
                0.7071067811865476,
            ),
        )

        transformed = transform_point(robot_pose, Point3(0.2, 0.0, 0.60))

        self.assertAlmostEqual(transformed.x, 4.0)
        self.assertAlmostEqual(transformed.y, -2.8)
        self.assertAlmostEqual(transformed.z, 0.62)

    def test_converts_world_grasp_error_into_robot_base_axes(self):
        correction = point_correction_in_parent(
            actual=Point3(1.0, 1.0, 0.4),
            desired=Point3(0.0, 1.0, 0.5),
            parent_rotation=Quaternion(
                0.0,
                0.0,
                0.7071067811865476,
                0.7071067811865476,
            ),
        )

        self.assertAlmostEqual(correction.x, 0.0)
        self.assertAlmostEqual(correction.y, 1.0)
        self.assertAlmostEqual(correction.z, 0.1)

    def test_upright_check_rejects_a_slanted_cylinder(self):
        upright = check_upright(
            Quaternion(0.0, 0.0, 0.0, 1.0), max_tilt_degrees=8.0
        )
        tilted = check_upright(
            Quaternion(0.0, 0.258819, 0.0, 0.965926),
            max_tilt_degrees=8.0,
        )

        self.assertTrue(upright.accepted)
        self.assertFalse(tilted.accepted)
        self.assertAlmostEqual(tilted.tilt_degrees, 30.0, places=3)

    def test_resolves_orientation_through_transform_chain(self):
        transforms = (
            FrameTransform(
                parent_frame="world",
                child_frame="part_model",
                translation=Point3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.70710678, 0.70710678),
            ),
            FrameTransform(
                parent_frame="part_model",
                child_frame="part_model::part",
                translation=Point3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
        )

        rotation = resolve_frame_rotation(
            transforms, "part_model::part", root_frames=("world",)
        )

        self.assertIsNotNone(rotation)
        axis = check_upright(rotation, max_tilt_degrees=1.0)
        self.assertTrue(axis.accepted)

    def test_attachment_distance_detects_lost_constraint(self):
        stable = check_attachment_distance(0.061, 0.050, 0.020)
        lost = check_attachment_distance(0.390, 0.050, 0.020)

        self.assertTrue(stable.accepted)
        self.assertFalse(lost.accepted)
        self.assertAlmostEqual(stable.distance_error, 0.011)
        self.assertGreater(lost.distance_error, stable.distance_error)
