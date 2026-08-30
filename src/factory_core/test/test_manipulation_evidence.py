import threading
import time
import unittest

from factory_core.manipulation_evidence import (
    FingerContactCheck,
    ManipulationEvidence,
    collision_belongs_to_part,
    contact_is_current,
    grasp_hold_is_valid,
    loaded_hold_reseat_is_valid,
    proof_lift_reseat_is_valid,
    support_collision_matches_station,
)


def evidence_with_contacts(left=(), right=()):
    """Build the tactile cache without creating ROS subscriptions."""
    evidence = ManipulationEvidence.__new__(ManipulationEvidence)
    evidence._part_ids = ("raw_part_1", "raw_part_2", "raw_part_3")
    evidence._maximum_sample_age = 1.0
    evidence._lock = threading.Lock()
    now = time.monotonic()
    evidence._finger_contact_times = {
        "left": {part_id: now for part_id in left},
        "right": {part_id: now for part_id in right},
    }
    evidence._support_contact_times = {}
    evidence._contact_pairs = {
        part_id: {} for part_id in evidence._part_ids
    }
    return evidence


class ManipulationEvidenceTests(unittest.TestCase):
    def test_anonymous_grasp_discovers_one_bilaterally_contacted_entity(self):
        selection = evidence_with_contacts(
            left=("raw_part_2",), right=("raw_part_2",)
        ).recent_any_two_finger_contact()
        self.assertTrue(selection.accepted)
        self.assertEqual(selection.physical_part_id, "raw_part_2")

    def test_contacts_on_different_parts_do_not_create_false_identity(self):
        selection = evidence_with_contacts(
            left=("raw_part_1",), right=("raw_part_2",)
        ).recent_any_two_finger_contact()
        self.assertFalse(selection.accepted)
        self.assertFalse(selection.ambiguous)

    def test_two_bilateral_matches_are_rejected_as_ambiguous(self):
        selection = evidence_with_contacts(
            left=("raw_part_1", "raw_part_2"),
            right=("raw_part_1", "raw_part_2"),
        ).recent_any_two_finger_contact()
        self.assertFalse(selection.accepted)
        self.assertTrue(selection.ambiguous)

    def test_bilateral_contact_and_preserved_aperture_prove_hold(self):
        self.assertTrue(
            grasp_hold_is_valid(
                FingerContactCheck(True, True),
                0.031,
                0.030,
                minimum_total_closure=0.020,
                maximum_position_change=0.008,
            )
        )

    def test_one_finger_contact_cannot_prove_hold(self):
        self.assertFalse(
            grasp_hold_is_valid(
                FingerContactCheck(True, False),
                0.031,
                0.030,
                minimum_total_closure=0.020,
                maximum_position_change=0.008,
            )
        )

    def test_aperture_drift_rejects_a_slipping_part(self):
        self.assertFalse(
            grasp_hold_is_valid(
                FingerContactCheck(True, True),
                0.021,
                0.031,
                minimum_total_closure=0.020,
                maximum_position_change=0.008,
            )
        )

    def test_missing_joint_state_fails_closed(self):
        self.assertFalse(
            grasp_hold_is_valid(
                FingerContactCheck(True, True),
                None,
                0.031,
                minimum_total_closure=0.020,
                maximum_position_change=0.008,
            )
        )

    def test_old_contact_cannot_authorize_a_new_close(self):
        self.assertFalse(
            contact_is_current(
                4.0,
                now=5.0,
                maximum_age=2.0,
                received_after=4.5,
            )
        )

    def test_collision_identity_is_exact_not_substring_only(self):
        self.assertTrue(
            collision_belongs_to_part(
                "raw_part_2::part::collision", "raw_part_2"
            )
        )
        self.assertFalse(
            collision_belongs_to_part(
                "raw_part_20::part::collision", "raw_part_2"
            )
        )

    def test_machine_support_requires_selected_fixture_geometry(self):
        fixture = "machine_2::body::fixture_base_collision"
        spindle = "machine_2::body::spindle_head_collision"
        self.assertTrue(support_collision_matches_station(fixture, "machine_2"))
        self.assertFalse(support_collision_matches_station(fixture, "machine_1"))
        self.assertFalse(support_collision_matches_station(spindle, "machine_2"))
        self.assertFalse(
            support_collision_matches_station(
                "machine_2::body::work_table_collision", "machine_2"
            )
        )

    def test_finished_support_cannot_be_satisfied_by_raw_table(self):
        self.assertTrue(
            support_collision_matches_station(
                "finished_bin::body::table_collision", "finished_bin"
            )
        )
        self.assertFalse(
            support_collision_matches_station(
                "raw_bin::body::table_collision", "finished_bin"
            )
        )


    def test_recent_support_contact_requires_current_selected_station(self):
        evidence = evidence_with_contacts()
        received_after = time.monotonic() - 0.1
        evidence._support_contact_times[("raw_part_2", "machine_3")] = (
            time.monotonic()
        )

        self.assertTrue(
            evidence.recent_support_contact(
                "raw_part_2", "machine_3", received_after=received_after
            )
        )
        self.assertFalse(
            evidence.recent_support_contact(
                "raw_part_2", "machine_2", received_after=received_after
            )
        )
        self.assertFalse(
            evidence.recent_support_contact(
                "raw_part_2",
                "machine_3",
                received_after=time.monotonic() + 0.1,
            )
        )

    def test_proof_lift_can_rebase_a_bilateral_compliant_reseat(self):
        self.assertTrue(
            proof_lift_reseat_is_valid(
                FingerContactCheck(True, True),
                0.062,
                minimum_total_closure=0.016,
                maximum_total_closure=0.072,
            )
        )

    def test_proof_lift_reseat_rejects_a_mechanical_stop(self):
        self.assertFalse(
            proof_lift_reseat_is_valid(
                FingerContactCheck(True, True),
                0.080,
                minimum_total_closure=0.016,
                maximum_total_closure=0.072,
            )
        )

    def test_proof_lift_reseat_still_requires_bilateral_contact(self):
        self.assertFalse(
            proof_lift_reseat_is_valid(
                FingerContactCheck(True, False),
                0.062,
                minimum_total_closure=0.016,
                maximum_total_closure=0.072,
            )
        )

    def test_proof_lift_reseat_rejects_an_invalid_range(self):
        with self.assertRaisesRegex(ValueError, "must exceed"):
            proof_lift_reseat_is_valid(
                FingerContactCheck(True, True),
                0.062,
                minimum_total_closure=0.072,
                maximum_total_closure=0.072,
            )

    def test_loaded_hold_can_reseat_once_with_bilateral_contact(self):
        self.assertTrue(
            loaded_hold_reseat_is_valid(
                FingerContactCheck(True, True),
                0.029,
                0.021,
                minimum_total_closure=0.016,
                maximum_total_closure=0.072,
                maximum_reseat_change=0.012,
            )
        )

    def test_loaded_hold_reseat_rejects_a_large_aperture_jump(self):
        self.assertFalse(
            loaded_hold_reseat_is_valid(
                FingerContactCheck(True, True),
                0.050,
                0.021,
                minimum_total_closure=0.016,
                maximum_total_closure=0.072,
                maximum_reseat_change=0.012,
            )
        )

    def test_loaded_hold_reseat_requires_bilateral_identity(self):
        self.assertFalse(
            loaded_hold_reseat_is_valid(
                FingerContactCheck(True, False),
                0.029,
                0.021,
                minimum_total_closure=0.016,
                maximum_total_closure=0.072,
                maximum_reseat_change=0.012,
            )
        )

    def test_loaded_hold_reseat_requires_an_existing_reference(self):
        self.assertFalse(
            loaded_hold_reseat_is_valid(
                FingerContactCheck(True, True),
                0.029,
                None,
                minimum_total_closure=0.016,
                maximum_total_closure=0.072,
                maximum_reseat_change=0.012,
            )
        )

if __name__ == "__main__":
    unittest.main()
