"""Exercise pose-level MoveIt planning for the gripper TCP."""

import rclpy
from rclpy.node import Node

from .motion_client import MoveGroupClient, STOWED, moveit_succeeded
from .pose_motion_client import PoseMoveGroupClient, PoseTarget


POSE_TEST = PoseTarget(
    name="tcp_pose_test",
    frame_id="arm_mount",
    position=(0.49, 0.24, 0.45),
    orientation=(-0.003, 1.0, 0.018, 0.001),
)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("factory_move_pose")
    pose_client = PoseMoveGroupClient(node)
    joint_client = MoveGroupClient(node)
    try:
        if not pose_client.wait_until_ready():
            raise RuntimeError("MoveIt action /move_action is not available")
        _execute(node, pose_client.send_pose_target(POSE_TEST), POSE_TEST.name)
        _execute(node, joint_client.send_joint_target(STOWED), STOWED.name)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _execute(node: Node, goal_future, target_name: str) -> None:
    node.get_logger().info(f"Planning and executing target: {target_name}")
    goal_handle = _wait(node, goal_future, 15.0)
    if goal_handle is None or not goal_handle.accepted:
        raise RuntimeError(f"MoveIt rejected target {target_name}")
    wrapped_result = _wait(node, goal_handle.get_result_async(), 60.0)
    if wrapped_result is None:
        raise RuntimeError(f"MoveIt execution timed out at {target_name}")
    if not moveit_succeeded(wrapped_result.result.error_code):
        code = wrapped_result.result.error_code.val
        raise RuntimeError(f"MoveIt failed at {target_name} with code {code}")
    node.get_logger().info(f"Reached {target_name} through MoveIt")


def _wait(node: Node, future, timeout_sec: float):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    return future.result() if future.done() else None


if __name__ == "__main__":
    main()
