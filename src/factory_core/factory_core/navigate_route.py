"""Navigate a configured factory transit route with Nav2."""

from __future__ import annotations

import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .route_config import FactoryRoute, load_factory_routes


def _default_route_config() -> str:
    share = Path(get_package_share_directory("factory_core"))
    return str(share / "config" / "routes.yaml")


def _default_behavior_tree() -> str:
    share = Path(get_package_share_directory("factory_core"))
    return str(
        share / "behavior_trees" / "navigate_through_factory_route.xml"
    )


class RouteNavigationClient:
    """Small typed adapter around Nav2's NavigateThroughPoses action."""

    def __init__(self, node: Node, behavior_tree: str) -> None:
        self._node = node
        self._behavior_tree = behavior_tree
        self._client = ActionClient(
            node, NavigateThroughPoses, "/navigate_through_poses"
        )

    def wait_until_ready(self, timeout_sec: float = 20.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send(self, route: FactoryRoute):
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._pose(waypoint) for waypoint in route.waypoints]
        goal.behavior_tree = self._behavior_tree
        return self._client.send_goal_async(goal)

    def _pose(self, waypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("factory_navigate_route")
    node.declare_parameter("route", "raw_bin_to_finished_bin")
    node.declare_parameter("route_config", _default_route_config())
    node.declare_parameter("behavior_tree", _default_behavior_tree())
    node.declare_parameter("navigation_timeout", 360.0)
    route_name = str(node.get_parameter("route").value)
    route_config = str(node.get_parameter("route_config").value)
    behavior_tree = str(node.get_parameter("behavior_tree").value)
    navigation_timeout = float(node.get_parameter("navigation_timeout").value)
    client = RouteNavigationClient(node, behavior_tree)

    try:
        route = load_factory_routes(route_config).get(route_name)
        if route is None:
            raise ValueError(f"Unknown route: {route_name}")
        if not client.wait_until_ready():
            raise RuntimeError(
                "Nav2 action /navigate_through_poses is not available"
            )

        node.get_logger().info(
            f"Navigating route {route.name} through "
            f"{len(route.waypoints)} waypoint(s)"
        )
        goal_handle = _wait(node, client.send(route), 15.0)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Nav2 rejected route {route.name}")

        wrapped_result = _wait(
            node, goal_handle.get_result_async(), navigation_timeout
        )
        if wrapped_result is None:
            _wait(node, goal_handle.cancel_goal_async(), 5.0)
            raise RuntimeError(f"Navigation route {route.name} timed out")

        result = wrapped_result.result
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != NavigateThroughPoses.Result.NONE
        ):
            raise RuntimeError(
                f"Route navigation failed: status={wrapped_result.status} "
                f"code={result.error_code} detail={result.error_msg}"
            )
        node.get_logger().info(f"Completed route {route.name}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _wait(node: Node, future, timeout_sec: float):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    return future.result() if future.done() else None


if __name__ == "__main__":
    main()
