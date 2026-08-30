"""Clear stale gripper joints while retaining raw-tray fixture clamps."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty


PART_IDS = ("raw_part_1", "raw_part_2", "raw_part_3")


def startup_detach_topics(
    part_ids: tuple[str, ...] = PART_IDS,
) -> tuple[str, ...]:
    """Return only stale gripper joints that are safe to clear at startup.

    Gazebo creates both detachable-joint adapters in the attached state.  Raw
    fixture joints intentionally remain attached as tray locators until a
    verified two-finger grasp asks the manipulation server to release one.
    """
    return tuple(
        f"/factory/gripper/{part_id}/detach" for part_id in part_ids
    )


class PartAttachmentInitializer(Node):
    """Clear startup gripper ownership without releasing source fixtures."""

    def __init__(self) -> None:
        super().__init__("part_attachment_initializer")
        self.declare_parameter("publish_delay", 0.5)
        self.declare_parameter("publish_count", 20)
        self.done = False
        self._published = 0
        self._started_at = self.get_clock().now()
        self._publishers = [
            self.create_publisher(Empty, topic, 1)
            for topic in startup_detach_topics()
        ]
        self._timer = self.create_timer(0.25, self._tick)

    def _tick(self) -> None:
        elapsed = (self.get_clock().now() - self._started_at).nanoseconds / 1e9
        if elapsed < float(self.get_parameter("publish_delay").value):
            return
        for publisher in self._publishers:
            publisher.publish(Empty())
        self._published += 1
        if self._published >= int(self.get_parameter("publish_count").value):
            self.done = True
            self._timer.cancel()
            self.get_logger().info(
                "Cleared startup gripper joints; raw tray fixtures remain clamped"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PartAttachmentInitializer()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
