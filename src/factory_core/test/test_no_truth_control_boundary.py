from pathlib import Path
import unittest


REPOSITORY = Path(__file__).resolve().parents[3]
CORE_PACKAGE = REPOSITORY / "src" / "factory_core"
RUNTIME = CORE_PACKAGE / "factory_core"
BRINGUP_LAUNCH = REPOSITORY / "src" / "factory_bringup" / "launch"


class NoTruthControlBoundaryTest(unittest.TestCase):
    def test_manipulation_runtime_has_no_gazebo_truth_or_fake_grasp_api(self):
        source = (RUNTIME / "manipulate_part_server.py").read_text()

        forbidden = (
            "GazeboTruthObserver",
            "gazebo_truth_observer",
            "request_attach",
            "request_detach",
            "set_entity_pose",
            "DetachableJoint",
        )
        for token in forbidden:
            self.assertNotIn(token, source)

    def test_truth_helpers_are_not_exposed_as_runtime_executables(self):
        setup_source = (CORE_PACKAGE / "setup.py").read_text()
        launch_source = "\n".join(
            path.read_text() for path in sorted(BRINGUP_LAUNCH.glob("*.py"))
        )

        for token in ("gazebo_truth_observer", "gazebo_truth_odometry"):
            self.assertNotIn(token, setup_source)
            self.assertNotIn(token, launch_source)

    def test_action_contract_names_missing_physical_evidence_explicitly(self):
        action = (
            REPOSITORY
            / "src"
            / "factory_interfaces"
            / "action"
            / "ManipulatePart.action"
        ).read_text()

        self.assertIn("PHYSICAL_EVIDENCE_FAILED=5", action)
        self.assertNotIn("ATTACHMENT_FAILED", action)

    def test_planning_scene_api_states_that_it_cannot_hold_gazebo_parts(self):
        source = (RUNTIME / "planning_scene_client.py").read_text()

        self.assertIn("attach_carried_workpiece_geometry", source)
        self.assertIn("cannot create force or a Gazebo constraint", source)
        self.assertNotIn("def attach_workpiece(", source)


if __name__ == "__main__":
    unittest.main()

