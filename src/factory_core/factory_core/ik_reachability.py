"""Collision-aware IK screening for sensor-selected manipulation targets."""

from __future__ import annotations

import threading
import time

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .manipulation_config import CartesianPose
from .motion_client import ARM_JOINTS, JointTarget


def candidate_screening_poses(
    target: CartesianPose,
    grasp_offset: tuple[float, float, float],
    approach_offset: tuple[float, float, float],
) -> tuple[tuple[str, CartesianPose], ...]:
    """Return every pose that must be executable before selecting a part."""
    grasp = target.translated(grasp_offset)
    approach = grasp.translated(approach_offset)
    return (("approach", approach), ("grasp", grasp))


def joint_state_with_arm_seed(
    measured: JointState, seed: JointTarget
) -> JointState:
    """Copy a full robot state and replace only the six arm positions."""
    seed.validate()
    source_names = set(measured.name)
    positions = dict(zip(measured.name, measured.position))
    positions.update(zip(ARM_JOINTS, seed.positions))
    if any(name not in source_names for name in ARM_JOINTS):
        raise ValueError("joint state does not contain every arm joint")

    result = JointState()
    result.header = measured.header
    result.name = list(measured.name)
    result.position = [positions[name] for name in result.name]
    # Velocity and effort lengths must continue to match the source names.
    result.velocity = list(measured.velocity)
    result.effort = list(measured.effort)
    return result


class CollisionAwareIk:
    """Use MoveIt's own collision model to reject unreachable candidates."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._lock = threading.Lock()
        self._joint_state: JointState | None = None
        self._subscription = node.create_subscription(
            JointState, "/joint_states", self._remember_joint_state, 10
        )
        self._client = node.create_client(GetPositionIK, "/compute_ik")

    def wait_until_ready(self, timeout_sec: float) -> bool:
        """Require both the service and one measured full robot state."""
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            return False
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                if self._joint_state is not None:
                    return True
            time.sleep(0.02)
        return False

    def solve_async(
        self,
        pose: CartesianPose,
        seed: JointTarget,
        *,
        timeout_sec: float = 0.75,
    ):
        """Request one collision-aware solution from a known wrist branch."""
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        with self._lock:
            measured = self._joint_state
        if measured is None:
            raise RuntimeError("no measured joint state is available for IK")

        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "arm"
        ik.ik_link_name = "gripper_tcp"
        robot_state = RobotState()
        robot_state.joint_state = joint_state_with_arm_seed(measured, seed)
        robot_state.is_diff = True
        ik.robot_state = robot_state
        ik.pose_stamped.header.frame_id = pose.frame_id
        ik.pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
        ik.pose_stamped.pose.position.x = pose.position[0]
        ik.pose_stamped.pose.position.y = pose.position[1]
        ik.pose_stamped.pose.position.z = pose.position[2]
        ik.pose_stamped.pose.orientation.x = pose.orientation[0]
        ik.pose_stamped.pose.orientation.y = pose.orientation[1]
        ik.pose_stamped.pose.orientation.z = pose.orientation[2]
        ik.pose_stamped.pose.orientation.w = pose.orientation[3]
        ik.avoid_collisions = True
        ik.timeout.sec = int(timeout_sec)
        ik.timeout.nanosec = int((timeout_sec % 1.0) * 1e9)
        return self._client.call_async(request)

    def _remember_joint_state(self, message: JointState) -> None:
        with self._lock:
            self._joint_state = message
