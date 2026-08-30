import unittest

from sensor_msgs.msg import JointState

from factory_core.ik_reachability import (
    candidate_screening_poses,
    joint_state_with_arm_seed,
)
from factory_core.manipulation_config import CartesianPose
from factory_core.motion_client import ARM_JOINTS, JointTarget


class IkReachabilityTests(unittest.TestCase):
    def test_arm_seed_preserves_non_arm_joint_measurements(self):
        measured = JointState()
        measured.name = ["gripper_finger_joint", *ARM_JOINTS]
        measured.position = [0.42, *([9.0] * len(ARM_JOINTS))]
        seed = JointTarget("candidate_seed", (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))

        result = joint_state_with_arm_seed(measured, seed)

        self.assertEqual(result.name, measured.name)
        self.assertEqual(result.position[0], 0.42)
        self.assertEqual(tuple(result.position[1:]), seed.positions)
        self.assertEqual(measured.position[1], 9.0)

    def test_missing_arm_joint_is_rejected(self):
        measured = JointState()
        measured.name = list(ARM_JOINTS[:-1])
        measured.position = [0.0] * len(measured.name)
        seed = JointTarget("candidate_seed", (0.0,) * len(ARM_JOINTS))

        with self.assertRaisesRegex(ValueError, "every arm joint"):
            joint_state_with_arm_seed(measured, seed)

    def test_candidate_screening_requires_approach_and_grasp_ik(self):
        target = CartesianPose(
            frame_id="base_link",
            position=(1.0, 2.0, 3.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )

        poses = candidate_screening_poses(
            target,
            grasp_offset=(0.0, 0.0, 0.005),
            approach_offset=(-0.1, 0.0, 0.0),
        )

        self.assertEqual([name for name, _pose in poses], ["approach", "grasp"])
        self.assertEqual(poses[0][1].position, (0.9, 2.0, 3.005))
        self.assertEqual(poses[1][1].position, (1.0, 2.0, 3.005))


if __name__ == "__main__":
    unittest.main()
