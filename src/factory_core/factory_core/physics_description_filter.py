"""Validate and republish the robot description used by Gazebo.

The first manipulation prototype needed a second URDF that replaced Robotiq
mesh collisions at runtime.  The loop-free parallel gripper now uses the same
convex geometry in MoveIt and Gazebo, so this boundary only validates that the
physical gripper is present before exposing the description to the spawner.
"""

from xml.etree import ElementTree

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


REQUIRED_GRIPPER_LINKS = (
    "gripper_parallel_base_link",
    "gripper_left_finger_tip_link",
    "gripper_right_finger_tip_link",
)
REQUIRED_GRIPPER_JOINTS = (
    "gripper_left_finger_joint",
    "gripper_right_finger_joint",
)


def _require_named_element(root, element_type: str, name: str) -> None:
    element = next(
        (
            item
            for item in root.findall(element_type)
            if item.get("name") == name
        ),
        None,
    )
    if element is None:
        raise ValueError(f"missing robot {element_type}: {name}")


def make_physics_description(robot_description: str) -> str:
    """Return one shared planning/physics URDF after structural validation."""
    root = ElementTree.fromstring(robot_description)
    for link_name in REQUIRED_GRIPPER_LINKS:
        _require_named_element(root, "link", link_name)
    for joint_name in REQUIRED_GRIPPER_JOINTS:
        _require_named_element(root, "joint", joint_name)
    return robot_description


class PhysicsDescriptionFilter(Node):
    """Publish the validated description on Gazebo's dedicated input topic."""

    def __init__(self) -> None:
        super().__init__("physics_description_filter")
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            String, "/gazebo_robot_description", qos
        )
        self._subscription = self.create_subscription(
            String, "/robot_description", self._on_description, qos
        )
        self._published_description: str | None = None

    def _on_description(self, message: String) -> None:
        if message.data == self._published_description:
            return
        try:
            physics_description = make_physics_description(message.data)
        except (ElementTree.ParseError, ValueError) as error:
            self.get_logger().error(f"Invalid robot description: {error}")
            return

        self._publisher.publish(String(data=physics_description))
        self._published_description = message.data
        self.get_logger().info(
            "Published shared MoveIt/Gazebo parallel-gripper description"
        )


def main() -> None:
    rclpy.init()
    node = PhysicsDescriptionFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
