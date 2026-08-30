from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class InitialPosePublisher(Node):
    """Publish the known Gazebo spawn pose so AMCL can activate deterministically."""

    def __init__(self) -> None:
        super().__init__("factory_initial_pose")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", -1.2)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("publish_delay", 2.0)
        self._published = 0
        self._started_at = self.get_clock().now()
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", qos
        )
        self._timer = self.create_timer(0.5, self._tick)

    def _tick(self) -> None:
        elapsed = (self.get_clock().now() - self._started_at).nanoseconds / 1e9
        if elapsed < float(self.get_parameter("publish_delay").value):
            return
        self._publisher.publish(self._message())
        self._published += 1
        if self._published >= 40:
            self._timer.cancel()
            self.get_logger().info("Published Gazebo spawn pose to AMCL")

    def _message(self) -> PoseWithCovarianceStamped:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = float(self.get_parameter("initial_x").value)
        message.pose.pose.position.y = float(self.get_parameter("initial_y").value)
        yaw = float(self.get_parameter("initial_yaw").value)
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
