"""Leave a station safely before starting normal navigation."""

import time

from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes
from nav2_msgs.action import UndockRobot
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .base_clearance import BaseClearanceController
from .motion_client import MoveGroupClient, STOWED, moveit_succeeded


class UndockStationClient:
    """Small action adapter; Nav2 remains responsible for base control."""

    def __init__(self, node: Node) -> None:
        self._action = ActionClient(node, UndockRobot, "/undock_robot")

    def wait_until_ready(self, timeout_sec: float = 30.0) -> bool:
        return self._action.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(self, dock_type: str, max_undocking_time: float):
        goal = UndockRobot.Goal()
        goal.dock_type = dock_type
        goal.max_undocking_time = max_undocking_time
        return self._action.send_goal_async(goal)


# The bin plugin's 1.0 m staging offset and 0.582 m calibrated work offset
# put its staging pose 0.418 m behind the manipulation pose. Nav2 accepts the
# undock within 0.05 m of staging, leaving at least 0.368 m. Keep a rounded
# 0.35 m guarantee so configuration and controller tolerances retain margin.
_NAV2_GUARANTEED_CLEARANCE = {
    "factory_bin_station": 0.35,
}


def remaining_clearance_after_nav2_undock(
    dock_type: str, requested_distance: float
) -> float:
    """Return only clearance not already guaranteed by Nav2's staging pose."""
    if requested_distance < 0.0:
        raise ValueError("requested clearance distance must be non-negative")
    guaranteed = _NAV2_GUARANTEED_CLEARANCE.get(dock_type, 0.0)
    return max(0.0, requested_distance - guaranteed)


def uses_nav2_undocking(dock_type: str) -> bool:
    """Return whether the station should use Nav2's staging-pose retreat.

    CNC manipulation ends at a visually refined work pose, not at the dock
    plugin's nominal pose. Reconstructing a staging pose from that final pose
    can make the Nav2 undock controller oscillate near the open machine. The
    measured straight retreat below is the safer and deterministic CNC exit.
    """
    return dock_type != "factory_station"


def _wait(node: Node, future, timeout_sec: float):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    return future.result() if future.done() else None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("factory_undock_station")
    node.declare_parameter("dock_type", "factory_station")
    node.declare_parameter("stow_arm", False)
    # The action timeout is separate from the post-undock 0.30 m retreat.
    # Budget for DART running below real time under the full perception stack.
    node.declare_parameter("undocking_timeout", 75.0)
    node.declare_parameter("clearance_distance", 0.30)
    node.declare_parameter("clearance_speed", 0.18)
    # DART may run below wall-clock speed while perception and MoveIt are
    # active. A fully loaded stack has measured 0.290 m after 45 wall-clock
    # seconds, so retain the 0.300 m safety distance and budget enough time
    # for that final centimetre instead of weakening the clearance contract.
    node.declare_parameter("clearance_timeout", 60.0)

    dock_type = str(node.get_parameter("dock_type").value)
    stow_arm = bool(node.get_parameter("stow_arm").value)
    undocking_timeout = float(node.get_parameter("undocking_timeout").value)
    clearance_distance = float(node.get_parameter("clearance_distance").value)
    clearance_speed = float(node.get_parameter("clearance_speed").value)
    clearance_timeout = float(node.get_parameter("clearance_timeout").value)

    client = UndockStationClient(node)
    clearance = BaseClearanceController(node)
    arm = MoveGroupClient(node)
    remaining_clearance = clearance_distance
    try:
        if uses_nav2_undocking(dock_type):
            _leave_nav2_dock(node, client, dock_type, undocking_timeout)
            remaining_clearance = remaining_clearance_after_nav2_undock(
                dock_type, clearance_distance
            )
            node.get_logger().info(
                "Nav2 reached the staging pose; "
                f"{remaining_clearance:.3f} m additional clearance remains"
            )
        else:
            node.get_logger().info(
                "Leaving the visually refined CNC work pose with a measured "
                "straight retreat"
            )
        travelled = clearance.retreat(
            remaining_clearance,
            clearance_speed,
            clearance_timeout,
        )
        node.get_logger().info(
            f"Station cleared after {travelled:.3f} m measured extra retreat"
        )
        if stow_arm:
            _stow_after_undocking(node, arm)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



def _leave_nav2_dock(
    node: Node,
    client: UndockStationClient,
    dock_type: str,
    undocking_timeout: float,
) -> None:
    """Release a dock through Nav2 and reach its configured staging pose."""
    if not client.wait_until_ready():
        raise RuntimeError("Nav2 action /undock_robot is not available")

    node.get_logger().info(f"Undocking from {dock_type}")
    goal_handle = _wait(
        node,
        client.send_goal(dock_type, undocking_timeout),
        timeout_sec=15.0,
    )
    if goal_handle is None or not goal_handle.accepted:
        raise RuntimeError("Docking server rejected the undock request")

    wrapped_result = _wait(
        node,
        goal_handle.get_result_async(),
        timeout_sec=undocking_timeout + 10.0,
    )
    if wrapped_result is None:
        _wait(node, goal_handle.cancel_goal_async(), timeout_sec=5.0)
        raise RuntimeError("Undocking timed out")

    result = wrapped_result.result
    if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
        raise RuntimeError(
            f"Undocking failed: status={wrapped_result.status} "
            f"code={result.error_code} detail={result.error_msg}"
        )


def _stow_after_undocking(node: Node, arm: MoveGroupClient) -> None:
    """Fold the empty arm only after the chassis has cleared the station."""
    if not arm.wait_until_ready(timeout_sec=20.0):
        raise RuntimeError("MoveIt action /move_action is not available")
    node.get_logger().info("Stowing the empty arm after undocking")
    last_error_code: int | None = None
    for attempt in range(1, 3):
        if attempt > 1:
            node.get_logger().warning(
                "MoveIt state lagged the controller after undocking; "
                "replanning stow from the latest measured state"
            )
            time.sleep(0.75)

        goal_handle = _wait(
            node,
            arm.send_joint_target(STOWED),
            timeout_sec=15.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveIt rejected the post-undock stow request")
        wrapped_result = _wait(
            node,
            goal_handle.get_result_async(),
            timeout_sec=75.0,
        )
        if wrapped_result is None:
            _wait(node, goal_handle.cancel_goal_async(), timeout_sec=5.0)
            raise RuntimeError("Post-undock arm stow timed out")
        if (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and moveit_succeeded(wrapped_result.result.error_code)
        ):
            node.get_logger().info(
                "Arm stowed with the station safely behind the chassis"
            )
            return

        last_error_code = int(wrapped_result.result.error_code.val)
        if last_error_code != MoveItErrorCodes.CONTROL_FAILED:
            break

    raise RuntimeError(
        f"Post-undock arm stow failed with code {last_error_code}"
    )


if __name__ == "__main__":
    main()
