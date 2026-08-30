from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node


ARM_JOINTS = (
    "arm_shoulder_pan_joint",
    "arm_shoulder_lift_joint",
    "arm_elbow_joint",
    "arm_wrist_1_joint",
    "arm_wrist_2_joint",
    "arm_wrist_3_joint",
)
# Named free-space postures remain below the UR joint limits. Manipulation
# contact and insertion use the lower Cartesian profiles instead.
JOINT_VELOCITY_SCALE = 0.85
JOINT_ACCELERATION_SCALE = 0.70


@dataclass(frozen=True)
class JointTarget:
    """Readable named arm target used by deterministic task primitives."""

    name: str
    positions: tuple[float, ...]

    def validate(self) -> None:
        if len(self.positions) != len(ARM_JOINTS):
            raise ValueError(f"{self.name} must define {len(ARM_JOINTS)} joints")


STOWED = JointTarget(
    "stowed",
    (0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0),
)
# A compact, collision-checked posture that unfolds the elbow above the deck
# before Cartesian station motion. It also gives IK a central wrist seed.
MANIPULATION_READY = JointTarget(
    "manipulation_ready",
    (0.35, -1.35, 1.45, -1.70, -1.5708, 0.25),
)
# Collision-checked high waypoint shared by the two geometrically identical
# bin stations. A pose target has multiple elbow / wrist IK branches; this
# recorded branch is the one whose low Cartesian continuation stays outside
# the mobile-base envelope.
BIN_STATION_TRANSIT = JointTarget(
    "bin_station_transit",
    (-2.683542, -1.432689, -2.285305, -2.565191, -1.112748, -1.570797),
)
# Collision-checked pre-grasp configuration captured after the raised raw-bin
# fixture was introduced. Keeping this station-specific taught waypoint avoids
# asking IK to choose a different elbow branch on every production cycle.
RAW_BIN_APPROACH = JointTarget(
    "raw_bin_approach",
    (-0.459838, 0.030302, 1.925045, -1.966176, 1.062605, -1.557617),
)
MOTION_TEST = JointTarget(
    "motion_test",
    (0.35, -1.35, 1.45, -1.70, -1.5708, 0.25),
)


class MoveGroupClient:
    """Small ROS action adapter; planning remains owned by MoveIt."""

    def __init__(self, node: Node, action_name: str = "/move_action") -> None:
        self._client = ActionClient(node, MoveGroup, action_name)

    def wait_until_ready(self, timeout_sec: float = 20.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send_joint_target(self, target: JointTarget):
        target.validate()
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 8.0
        goal.request.max_velocity_scaling_factor = JOINT_VELOCITY_SCALE
        goal.request.max_acceleration_scaling_factor = JOINT_ACCELERATION_SCALE
        goal.request.pipeline_id = "ompl"
        goal.request.goal_constraints = [self._joint_constraints(target.positions)]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        return self._client.send_goal_async(goal)

    @staticmethod
    def _joint_constraints(positions: Iterable[float]) -> Constraints:
        constraints = Constraints(name="deterministic_joint_target")
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=float(position),
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for name, position in zip(ARM_JOINTS, positions, strict=True)
        ]
        return constraints


def moveit_succeeded(error_code: MoveItErrorCodes) -> bool:
    return error_code.val == MoveItErrorCodes.SUCCESS
