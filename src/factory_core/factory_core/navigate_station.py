from __future__ import annotations

from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node

from .navigation_client import Nav2Client, load_station_poses


def _default_station_config() -> str:
    return str(Path(get_package_share_directory("factory_core")) / "config/stations.yaml")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("factory_navigate_station")
    node.declare_parameter("station", "raw_bin")
    node.declare_parameter("station_config", _default_station_config())
    node.declare_parameter("navigation_timeout", 240.0)
    station_name = str(node.get_parameter("station").value)
    station_config = str(node.get_parameter("station_config").value)
    navigation_timeout = float(node.get_parameter("navigation_timeout").value)
    client = Nav2Client(node)
    try:
        station = load_station_poses(station_config).get(station_name)
        if station is None:
            raise ValueError(f"Unknown station: {station_name}")
        if not client.wait_until_ready():
            raise RuntimeError("Nav2 action /navigate_to_pose is not available")
        node.get_logger().info(f"Navigating to {station.name} staging pose")
        goal_handle = _wait(node, client.send_station(station), 15.0)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Nav2 rejected station {station.name}")
        wrapped_result = _wait(
            node, goal_handle.get_result_async(), navigation_timeout
        )
        if wrapped_result is None:
            cancel_result = _wait(node, goal_handle.cancel_goal_async(), 5.0)
            if cancel_result is None:
                node.get_logger().warning("Nav2 cancel request timed out")
            raise RuntimeError(f"Navigation to {station.name} timed out")
        result = wrapped_result.result
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != NavigateToPose.Result.NONE
        ):
            raise RuntimeError(
                f"Navigation failed: status={wrapped_result.status} "
                f"code={result.error_code} detail={result.error_msg}"
            )
        node.get_logger().info(f"Reached {station.name} staging pose")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _wait(node: Node, future, timeout_sec: float):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    return future.result() if future.done() else None


if __name__ == "__main__":
    main()
