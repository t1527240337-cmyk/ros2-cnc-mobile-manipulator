from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from .station_config import load_station_definitions


@dataclass(frozen=True)
class StationPose:
    name: str
    x: float
    y: float
    yaw: float


def load_station_poses(config_path: str | Path) -> dict[str, StationPose]:
    """Load and validate staging poses from the factory's single config file."""
    definitions = load_station_definitions(config_path)
    return {
        name: StationPose(
            name=name,
            x=station.staging_pose.x,
            y=station.staging_pose.y,
            yaw=station.staging_pose.yaw,
        )
        for name, station in definitions.items()
    }


class Nav2Client:
    """Typed adapter around Nav2's NavigateToPose action."""

    def __init__(self, node: Node, action_name: str = "/navigate_to_pose") -> None:
        self._node = node
        self._client = ActionClient(node, NavigateToPose, action_name)

    def wait_until_ready(self, timeout_sec: float = 20.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send_station(self, station: StationPose):
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(station)
        return self._client.send_goal_async(goal)

    def _pose(self, station: StationPose) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = station.x
        pose.pose.position.y = station.y
        pose.pose.orientation.z = math.sin(station.yaw / 2.0)
        pose.pose.orientation.w = math.cos(station.yaw / 2.0)
        return pose
