"""Publish a continuous odometry frame from Gazebo model-pose truth.

The physical differential base, arm and workpieces share one articulated
simulation. This adapter removes the known world spawn pose from Gazebo truth
and publishes only motion relative to startup. AMCL still owns
``map -> odom`` and therefore still has to localize from the laser scan.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .base_kinematics import PlanarPose, local_motion, normalize_angle
from .planar_velocity import quaternion_yaw


def relative_pose(origin: PlanarPose, current: PlanarPose) -> PlanarPose:
    """Express a world pose in an odometry frame fixed at ``origin``."""

    motion = local_motion(origin, current)
    return PlanarPose(motion.forward, motion.lateral, motion.yaw)


def body_velocity(
    previous: PlanarPose, current: PlanarPose, elapsed: float
) -> tuple[float, float, float]:
    """Estimate child-frame planar velocity from two odometry poses."""

    if elapsed <= 0.0:
        return 0.0, 0.0, 0.0
    velocity_x = (current.x - previous.x) / elapsed
    velocity_y = (current.y - previous.y) / elapsed
    cosine = math.cos(current.yaw)
    sine = math.sin(current.yaw)
    return (
        cosine * velocity_x + sine * velocity_y,
        -sine * velocity_x + cosine * velocity_y,
        normalize_angle(current.yaw - previous.yaw) / elapsed,
    )


class SimOdometry(Node):
    """Convert absolute Gazebo pose odometry into ROS's local odom frame."""

    def __init__(self) -> None:
        super().__init__("sim_odometry")
        self.declare_parameter("input_topic", "/ground_truth/odom")
        self.declare_parameter("output_topic", "/sim_base/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._origin: PlanarPose | None = None
        self._previous_pose: PlanarPose | None = None
        self._previous_stamp_ns: int | None = None
        self._publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 10
        )
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._on_truth,
            10,
        )

    def _on_truth(self, truth: Odometry) -> None:
        world = _planar_pose(truth)
        if self._origin is None:
            self._origin = world
        odom_pose = relative_pose(self._origin, world)
        stamp_ns = _stamp_nanoseconds(truth)

        velocity = (0.0, 0.0, 0.0)
        if self._previous_pose is not None and self._previous_stamp_ns is not None:
            elapsed = (stamp_ns - self._previous_stamp_ns) / 1_000_000_000.0
            if 0.001 <= elapsed <= 0.5:
                velocity = body_velocity(self._previous_pose, odom_pose, elapsed)

        message = Odometry()
        message.header.stamp = truth.header.stamp
        message.header.frame_id = self._odom_frame
        message.child_frame_id = self._base_frame
        message.pose.pose.position.x = odom_pose.x
        message.pose.pose.position.y = odom_pose.y
        message.pose.pose.orientation.z = math.sin(odom_pose.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(odom_pose.yaw / 2.0)
        message.twist.twist.linear.x = velocity[0]
        message.twist.twist.linear.y = velocity[1]
        message.twist.twist.angular.z = velocity[2]
        self._publisher.publish(message)

        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = odom_pose.x
        transform.transform.translation.y = odom_pose.y
        transform.transform.rotation = message.pose.pose.orientation
        self._broadcaster.sendTransform(transform)

        self._previous_pose = odom_pose
        self._previous_stamp_ns = stamp_ns


def _planar_pose(message: Odometry) -> PlanarPose:
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return PlanarPose(
        position.x,
        position.y,
        quaternion_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        ),
    )


def _stamp_nanoseconds(message: Odometry) -> int:
    return (
        message.header.stamp.sec * 1_000_000_000
        + message.header.stamp.nanosec
    )


def main() -> None:
    rclpy.init()
    node = SimOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
