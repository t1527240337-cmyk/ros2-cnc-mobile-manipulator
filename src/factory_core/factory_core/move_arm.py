from __future__ import annotations

import rclpy
from rclpy.node import Node

from .motion_client import MOTION_TEST, MoveGroupClient, STOWED, moveit_succeeded


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("factory_move_arm")
    client = MoveGroupClient(node)
    try:
        if not client.wait_until_ready():
            raise RuntimeError("MoveIt action /move_action is not available")
        for target in (MOTION_TEST, STOWED):
            node.get_logger().info(f"Planning and executing arm target: {target.name}")
            goal_handle = _wait(node, client.send_joint_target(target), 15.0)
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError(f"MoveIt rejected target {target.name}")
            wrapped_result = _wait(node, goal_handle.get_result_async(), 45.0)
            if wrapped_result is None:
                raise RuntimeError(f"MoveIt execution timed out at {target.name}")
            if not moveit_succeeded(wrapped_result.result.error_code):
                code = wrapped_result.result.error_code.val
                raise RuntimeError(f"MoveIt failed at {target.name} with code {code}")
            node.get_logger().info(f"Arm reached {target.name} through MoveIt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _wait(node: Node, future, timeout_sec: float):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    return future.result() if future.done() else None
