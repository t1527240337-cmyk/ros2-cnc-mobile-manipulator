"""Arbitrate and adapt mobile-base velocity commands for Gazebo."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class VelocitySourceArbiter:
    """Give a short, renewable priority lease to precision docking commands."""

    def __init__(self, hold_sec: float) -> None:
        if hold_sec <= 0.0:
            raise ValueError("hold_sec must be positive")
        self._hold_sec = hold_sec
        self._precision_deadline = 0.0

    def register_precision_command(self, now: float) -> None:
        """Renew the lease whenever the visual servo publishes a command."""
        self._precision_deadline = now + self._hold_sec

    def accepts_navigation_command(self, now: float) -> bool:
        """Return false while a recent precision command owns the base."""
        return now >= self._precision_deadline


class TwistStamper(Node):
    """Expose one differential-base command publisher and arbitrate inputs.

    Nav2 normally owns the base through ``input_topic``.  During the final
    centimetres of station alignment, the visual servo renews a short lease on
    ``precision_input_topic``.  Normal commands (including Collision Monitor's
    repeated zero command) cannot overwrite that lease, while loss of the
    visual-servo process automatically returns control to Nav2 after the lease
    expires.
    """

    def __init__(self) -> None:
        super().__init__("cmd_vel_stamper")
        self.declare_parameter("input_topic", "/cmd_vel_safe")
        self.declare_parameter("precision_input_topic", "/cmd_vel_docking")
        self.declare_parameter("precision_hold_sec", 0.20)
        self.declare_parameter("output_topic", "/base_controller/cmd_vel")
        self.declare_parameter("frame_id", "base_footprint")

        input_topic = str(self.get_parameter("input_topic").value)
        precision_input_topic = str(
            self.get_parameter("precision_input_topic").value
        )
        output_topic = str(self.get_parameter("output_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._arbiter = VelocitySourceArbiter(
            float(self.get_parameter("precision_hold_sec").value)
        )

        self._publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._on_navigation, 10)
        self.create_subscription(
            Twist, precision_input_topic, self._on_precision, 10
        )

        self.get_logger().info(
            "Velocity arbiter ready: "
            f"navigation={input_topic}, precision={precision_input_topic}, "
            f"differential_base={output_topic}"
        )

    def _on_navigation(self, command: Twist) -> None:
        if not self._arbiter.accepts_navigation_command(time.monotonic()):
            return
        self._publish(command)

    def _on_precision(self, command: Twist) -> None:
        self._arbiter.register_precision_command(time.monotonic())
        self._publish(command)

    def _publish(self, command: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._frame_id
        stamped.twist = command
        self._publisher.publish(stamped)


def main() -> None:
    rclpy.init()
    node = TwistStamper()
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
