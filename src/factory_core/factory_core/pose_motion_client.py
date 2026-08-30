"""Pose-level MoveIt adapter for grasp and place primitives."""

from dataclasses import dataclass
from typing import Literal

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


DEFAULT_POSITION_TOLERANCE = 0.003
# Empty-gripper Cartesian travel is still bounded below the joint-space
# profile. The higher velocity shortens longer approach segments, while the
# validated acceleration margin is retained because Cartesian acceleration
# maps nonlinearly into the arm joints near the CNC reach boundary.
EMPTY_LINEAR_VELOCITY_SCALE = 0.65
EMPTY_LINEAR_ACCELERATION_SCALE = 0.20
# Empty Cartesian motion inside the CNC is close to the UR5e reach boundary.
# Keep the general empty speed but leave acceleration margin for the nonlinear
# elbow mapping observed across valid mobile-base docking poses.
FIXTURE_EMPTY_LINEAR_VELOCITY_SCALE = 0.65
FIXTURE_EMPTY_LINEAR_ACCELERATION_SCALE = 0.15
# Proof lifts remain the slowest profile because they run before the optional
# transport constraint is enabled. Bilateral physical contact is already
# established, so 16% / 6% remains a low-energy verification motion.
PROOF_LINEAR_VELOCITY_SCALE = 0.16
PROOF_LINEAR_ACCELERATION_SCALE = 0.06
# General loaded motion covers collision-checked bin approach and placement.
# It remains slower than empty travel and post-proof transport, but need not
# inherit the tighter acceleration limit required at the CNC reach boundary.
LOADED_LINEAR_VELOCITY_SCALE = 0.40
LOADED_LINEAR_ACCELERATION_SCALE = 0.16
# The CNC insertion sits close to the arm's reach boundary. Small changes in
# visual docking and the preceding IK solution amplify Cartesian acceleration
# at the elbow, so fixture entry owns a separate measured-safe envelope.
FIXTURE_INSERTION_LINEAR_VELOCITY_SCALE = 0.40
FIXTURE_INSERTION_LINEAR_ACCELERATION_SCALE = 0.13
# Pilz rejected 0.65 / 0.45 on the measured loaded CNC-exit state and paid
# for a second planning attempt. This profile preserves more margin while
# remaining faster than the former process-linear profile. A collision-checked
# The explicit PTP candidate covers dock poses where LIN has no valid solution.
# Long diagonal transport produces a larger tangential load than vertical
# proof-lift or short fixture insertion. Keep it faster than process motion,
# but bound acceleration so the upright cylinder cannot roll between pads.
LOADED_TRANSPORT_LINEAR_VELOCITY_SCALE = 0.45
LOADED_TRANSPORT_LINEAR_ACCELERATION_SCALE = 0.18
# Crossing the CNC door plane is more constrained than open-space transport.
# Keep both speed and acceleration below the long-carry envelope; PTP is
# deliberately not used while the gripper is inside the physical aperture.
LOADED_EGRESS_LINEAR_VELOCITY_SCALE = 0.40
LOADED_EGRESS_LINEAR_ACCELERATION_SCALE = 0.14
# The proof motion above remains deliberately slow and friction-only. These
# faster profiles are used only after the workpiece passed bilateral contact,
# the friction-only proof lift, and measured-aperture hold checks. Every loaded
# segment revalidates tactile/joint evidence and remains collision checked.
LOADED_PTP_VELOCITY_SCALE = 0.70
LOADED_PTP_ACCELERATION_SCALE = 0.50
LOADED_OMPL_VELOCITY_SCALE = 0.65
LOADED_OMPL_ACCELERATION_SCALE = 0.50
EMPTY_PTP_VELOCITY_SCALE = 0.80
EMPTY_PTP_ACCELERATION_SCALE = 0.65
EMPTY_OMPL_VELOCITY_SCALE = 0.70
EMPTY_OMPL_ACCELERATION_SCALE = 0.55

MotionPlanner = Literal[
    "ompl",
    "loaded_ompl",
    "ptp",
    "loaded_ptp",
    "lin",
    "proof_lin",
    "loaded_lin",
    "fixture_empty_lin",
    "fixture_loaded_lin",
    "loaded_transport_lin",
    "loaded_egress_lin",
]


@dataclass(frozen=True)
class PoseTarget:
    """A named gripper TCP pose in a known TF frame."""

    name: str
    frame_id: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]

    def validate(self) -> None:
        if not self.name or not self.frame_id:
            raise ValueError("pose target needs a name and frame")
        if len(self.position) != 3 or len(self.orientation) != 4:
            raise ValueError("pose target needs xyz and xyzw values")


class PoseMoveGroupClient:
    """Plan collision-checked free-space or process-linear TCP motion."""

    def __init__(
        self,
        node: Node,
        action_name: str = "/move_action",
        position_tolerance: float = DEFAULT_POSITION_TOLERANCE,
    ) -> None:
        if position_tolerance <= 0.0:
            raise ValueError("position_tolerance must be positive")
        self._client = ActionClient(node, MoveGroup, action_name)
        self._position_tolerance = position_tolerance

    def wait_until_ready(self, timeout_sec: float = 20.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send_pose_target(
        self,
        target: PoseTarget,
        *,
        planner: MotionPlanner = "ompl",
        position_tolerance: float | None = None,
    ):
        target.validate()
        tolerance = self._position_tolerance
        if position_tolerance is not None:
            if position_tolerance <= 0.0:
                raise ValueError("position_tolerance must be positive")
            tolerance = position_tolerance

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 12
        goal.request.allowed_planning_time = 15.0
        if planner not in {
            "ompl",
            "loaded_ompl",
            "ptp",
            "loaded_ptp",
            "lin",
            "proof_lin",
            "loaded_lin",
            "fixture_empty_lin",
            "fixture_loaded_lin",
            "loaded_transport_lin",
            "loaded_egress_lin",
        }:
            raise ValueError(f"unsupported motion planner: {planner}")
        if planner in {
            "lin",
            "proof_lin",
            "loaded_lin",
            "fixture_empty_lin",
            "fixture_loaded_lin",
            "loaded_transport_lin",
            "loaded_egress_lin",
        }:
            # A friction-only grasp needs a gentler acceleration envelope than
            # an empty gripper. The transport variant is allowed only after
            # the unassisted proof lift has established a stable grasp.
            if planner == "loaded_lin":
                velocity = LOADED_LINEAR_VELOCITY_SCALE
                acceleration = LOADED_LINEAR_ACCELERATION_SCALE
            elif planner == "fixture_empty_lin":
                velocity = FIXTURE_EMPTY_LINEAR_VELOCITY_SCALE
                acceleration = FIXTURE_EMPTY_LINEAR_ACCELERATION_SCALE
            elif planner == "fixture_loaded_lin":
                velocity = FIXTURE_INSERTION_LINEAR_VELOCITY_SCALE
                acceleration = FIXTURE_INSERTION_LINEAR_ACCELERATION_SCALE
            elif planner == "proof_lin":
                velocity = PROOF_LINEAR_VELOCITY_SCALE
                acceleration = PROOF_LINEAR_ACCELERATION_SCALE
            elif planner == "loaded_transport_lin":
                velocity = LOADED_TRANSPORT_LINEAR_VELOCITY_SCALE
                acceleration = LOADED_TRANSPORT_LINEAR_ACCELERATION_SCALE
            elif planner == "loaded_egress_lin":
                velocity = LOADED_EGRESS_LINEAR_VELOCITY_SCALE
                acceleration = LOADED_EGRESS_LINEAR_ACCELERATION_SCALE
            else:
                velocity = EMPTY_LINEAR_VELOCITY_SCALE
                acceleration = EMPTY_LINEAR_ACCELERATION_SCALE
            goal.request.max_velocity_scaling_factor = velocity
            goal.request.max_acceleration_scaling_factor = acceleration
            goal.request.pipeline_id = "pilz_industrial_motion_planner"
            goal.request.planner_id = "LIN"
        elif planner in {"ptp", "loaded_ptp"}:
            loaded = planner == "loaded_ptp"
            goal.request.max_velocity_scaling_factor = (
                LOADED_PTP_VELOCITY_SCALE
                if loaded
                else EMPTY_PTP_VELOCITY_SCALE
            )
            goal.request.max_acceleration_scaling_factor = (
                LOADED_PTP_ACCELERATION_SCALE
                if loaded
                else EMPTY_PTP_ACCELERATION_SCALE
            )
            goal.request.pipeline_id = "pilz_industrial_motion_planner"
            goal.request.planner_id = "PTP"
        else:
            loaded = planner == "loaded_ompl"
            goal.request.max_velocity_scaling_factor = (
                LOADED_OMPL_VELOCITY_SCALE
                if loaded
                else EMPTY_OMPL_VELOCITY_SCALE
            )
            goal.request.max_acceleration_scaling_factor = (
                LOADED_OMPL_ACCELERATION_SCALE
                if loaded
                else EMPTY_OMPL_ACCELERATION_SCALE
            )
            goal.request.pipeline_id = "ompl"
            goal.request.planner_id = "RRTConnectkConfigDefault"
        goal.request.goal_constraints = [self._constraints(target, tolerance)]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        return self._client.send_goal_async(goal)

    def _constraints(
        self, target: PoseTarget, position_tolerance: float
    ) -> Constraints:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = target.position
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = target.orientation

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tolerance]

        position = PositionConstraint()
        position.header.frame_id = target.frame_id
        position.link_name = "gripper_tcp"
        position.weight = 1.0
        position.constraint_region = BoundingVolume(
            primitives=[sphere], primitive_poses=[pose]
        )

        orientation = OrientationConstraint()
        orientation.header.frame_id = target.frame_id
        orientation.link_name = "gripper_tcp"
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = 0.05
        orientation.absolute_y_axis_tolerance = 0.05
        orientation.absolute_z_axis_tolerance = 0.05
        orientation.weight = 1.0

        return Constraints(
            name=target.name,
            position_constraints=[position],
            orientation_constraints=[orientation],
        )
