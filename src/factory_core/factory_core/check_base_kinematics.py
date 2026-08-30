"""Drive a short calibration pattern and compare wheel odometry with Gazebo truth."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .base_kinematics import PlanarPose, local_motion, motion_error


class BaseKinematicsChecker(Node):
    def __init__(self) -> None:
        super().__init__("base_kinematics_checker")
        self.declare_parameter("forward_speed", 0.20)
        self.declare_parameter("forward_duration", 2.0)
        self.declare_parameter("turn_speed", 0.50)
        self.declare_parameter("turn_duration", 2.0)
        self.declare_parameter("position_tolerance", 0.08)
        self.declare_parameter("yaw_tolerance", 0.12)
        self.declare_parameter("tilt_tolerance", 0.10)
        self.declare_parameter("odom_topic", "/base_controller/odom")

        self.controller_pose: PlanarPose | None = None
        self.truth_pose: PlanarPose | None = None
        self.maximum_truth_tilt = 0.0
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_controller_odom,
            10,
        )
        self.create_subscription(
            Odometry, "/ground_truth/odom", self._on_truth_odom, 10
        )
        self.command_publisher = self.create_publisher(
            TwistStamped, "/base_controller/cmd_vel", 10
        )

    def wait_for_measurements(self, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.controller_pose is not None and self.truth_pose is not None:
                return
        raise RuntimeError("Timed out waiting for controller and ground-truth odometry")

    def drive(self, linear: float, angular: float, duration: float) -> None:
        # Use the node clock so a slow simulation still receives exactly the
        # requested motion duration. The launch test enables simulated time.
        deadline_ns = self.get_clock().now().nanoseconds + int(duration * 1e9)
        while self.get_clock().now().nanoseconds < deadline_ns:
            self._publish_command(linear, angular)
            rclpy.spin_once(self, timeout_sec=0.05)
        # Hold zero for simulation time too. A few wall-clock spins can be
        # shorter than one physics update when Gazebo is heavily loaded.
        settle_deadline_ns = self.get_clock().now().nanoseconds + int(1.0e9)
        while self.get_clock().now().nanoseconds < settle_deadline_ns:
            self._publish_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._publish_command(0.0, 0.0)

    def snapshot(self) -> tuple[PlanarPose, PlanarPose]:
        if self.controller_pose is None or self.truth_pose is None:
            raise RuntimeError("Odometry is not initialized")
        return self.controller_pose, self.truth_pose

    def _publish_command(self, linear: float, angular: float) -> None:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_footprint"
        command.twist.linear.x = linear
        command.twist.angular.z = angular
        self.command_publisher.publish(command)

    def _on_controller_odom(self, message: Odometry) -> None:
        self.controller_pose = _planar_pose(message)

    def _on_truth_odom(self, message: Odometry) -> None:
        self.truth_pose = _planar_pose(message)
        orientation = message.pose.pose.orientation
        # World-Z component of the robot's local Z axis. This catches a base
        # that rolls over even when its projected planar yaw still looks sane.
        upright_z = 1.0 - 2.0 * (orientation.x**2 + orientation.y**2)
        tilt = math.acos(max(-1.0, min(1.0, upright_z)))
        self.maximum_truth_tilt = max(self.maximum_truth_tilt, tilt)


def _planar_pose(message: Odometry) -> PlanarPose:
    orientation = message.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )
    position = message.pose.pose.position
    return PlanarPose(position.x, position.y, yaw)


def _format_motion(label, motion) -> str:
    return (
        f"{label}: forward={motion.forward:+.3f} m "
        f"lateral={motion.lateral:+.3f} m yaw={motion.yaw:+.3f} rad"
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseKinematicsChecker()
    try:
        node.wait_for_measurements()
        odom_start, truth_start = node.snapshot()
        node.drive(
            float(node.get_parameter("forward_speed").value),
            0.0,
            float(node.get_parameter("forward_duration").value),
        )
        odom_forward_end, truth_forward_end = node.snapshot()
        odom_forward = local_motion(odom_start, odom_forward_end)
        truth_forward = local_motion(truth_start, truth_forward_end)
        forward_error = motion_error(truth_forward, odom_forward)

        odom_turn_start, truth_turn_start = node.snapshot()
        node.drive(
            0.0,
            float(node.get_parameter("turn_speed").value),
            float(node.get_parameter("turn_duration").value),
        )
        odom_turn_end, truth_turn_end = node.snapshot()
        odom_turn = local_motion(odom_turn_start, odom_turn_end)
        truth_turn = local_motion(truth_turn_start, truth_turn_end)
        turn_error = motion_error(truth_turn, odom_turn)

        # A differential base must be symmetric. Testing only one direction
        # allowed a contact-model regression to pass while Nav2 could not make
        # a clockwise turn at the first station.
        odom_reverse_start, truth_reverse_start = node.snapshot()
        node.drive(
            0.0,
            -float(node.get_parameter("turn_speed").value),
            float(node.get_parameter("turn_duration").value),
        )
        odom_reverse_end, truth_reverse_end = node.snapshot()
        odom_reverse = local_motion(odom_reverse_start, odom_reverse_end)
        truth_reverse = local_motion(truth_reverse_start, truth_reverse_end)
        reverse_error = motion_error(truth_reverse, odom_reverse)

        for line in (
            _format_motion("forward truth", truth_forward),
            _format_motion("forward odom ", odom_forward),
            _format_motion("forward error", forward_error),
            _format_motion("turn truth", truth_turn),
            _format_motion("turn odom ", odom_turn),
            _format_motion("turn error", turn_error),
            _format_motion("reverse turn truth", truth_reverse),
            _format_motion("reverse turn odom ", odom_reverse),
            _format_motion("reverse turn error", reverse_error),
            f"maximum truth tilt: {node.maximum_truth_tilt:.3f} rad",
        ):
            node.get_logger().info(line)

        position_tolerance = float(node.get_parameter("position_tolerance").value)
        yaw_tolerance = float(node.get_parameter("yaw_tolerance").value)
        tilt_tolerance = float(node.get_parameter("tilt_tolerance").value)
        failed = (
            truth_forward.forward <= 0.10
            or abs(forward_error.forward) > position_tolerance
            or abs(forward_error.lateral) > position_tolerance
            or abs(truth_turn.yaw) <= 0.40
            or abs(turn_error.yaw) > yaw_tolerance
            or abs(truth_reverse.yaw) <= 0.40
            or truth_turn.yaw * truth_reverse.yaw >= 0.0
            or abs(reverse_error.yaw) > yaw_tolerance
            or node.maximum_truth_tilt > tilt_tolerance
        )
        if failed:
            raise RuntimeError(
                "Base kinematics disagree with Gazebo truth; calibrate wheel signs, "
                "radius, separation, or contact parameters"
            )
        node.get_logger().info("Base kinematics match Gazebo truth within tolerance")
    finally:
        node.drive(0.0, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
