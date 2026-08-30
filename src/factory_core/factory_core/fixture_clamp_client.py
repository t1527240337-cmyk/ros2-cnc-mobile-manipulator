"""ROS adapter for Gazebo's simulated CNC fixture clamps."""

from __future__ import annotations

from rclpy.node import Node
from std_msgs.msg import Empty, String


class FixtureClampClient:
    """Command one physical workpiece clamp and observe its confirmed state."""

    def __init__(self, node: Node, part_ids: tuple[str, ...]) -> None:
        self._attached: dict[str, bool | None] = {
            part_id: None for part_id in part_ids
        }
        self._attach_publishers = {}
        self._detach_publishers = {}
        self._subscriptions = []

        for part_id in part_ids:
            prefix = f"/factory/fixture/{part_id}"
            self._attach_publishers[part_id] = node.create_publisher(
                Empty, f"{prefix}/attach", 1
            )
            self._detach_publishers[part_id] = node.create_publisher(
                Empty, f"{prefix}/detach", 1
            )
            self._subscriptions.append(
                node.create_subscription(
                    String,
                    f"{prefix}/attached",
                    lambda message, current=part_id: self._remember_state(
                        current, message
                    ),
                    1,
                )
            )

    def request_clamp(self, part_id: str) -> None:
        self._publisher_for(self._attach_publishers, part_id).publish(Empty())

    def request_release(self, part_id: str) -> None:
        self._publisher_for(self._detach_publishers, part_id).publish(Empty())

    def is_clamped(self, part_id: str) -> bool | None:
        self._require_part(part_id)
        return self._attached[part_id]

    def _remember_state(self, part_id: str, message: String) -> None:
        self._attached[part_id] = message.data == "attached"

    def _publisher_for(self, publishers: dict, part_id: str):
        self._require_part(part_id)
        return publishers[part_id]

    def _require_part(self, part_id: str) -> None:
        if part_id not in self._attached:
            raise ValueError(f"unknown fixture workpiece: {part_id}")
