"""Bridge an unstamped velocity topic without changing its command."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TwistRelay(Node):
    """Connect a producer to Nav2's configured velocity pipeline."""

    def __init__(self) -> None:
        super().__init__("twist_relay")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_nav")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        if input_topic == output_topic:
            raise ValueError("Twist relay input and output topics must differ")

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._relay, 10)
        self.get_logger().info(
            f"Velocity relay ready: {input_topic} -> {output_topic}"
        )

    def _relay(self, command: Twist) -> None:
        self._publisher.publish(command)


def main() -> None:
    rclpy.init()
    node = TwistRelay()
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
