"""Coordinate Nav2 staging and camera alignment for factory stations."""

from __future__ import annotations

from pathlib import Path
import time
from typing import Callable

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import DockRobot, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String

from .navigation_client import Nav2Client, StationPose
from .station_config import StationDefinition, load_station_definitions
from .visual_station_alignment import (
    AlignmentError,
    TagAlignmentTarget,
    VisualStationAligner,
)


DOCKING_STATES = {
    DockRobot.Feedback.NAV_TO_STAGING_POSE: "navigating to staging pose",
    DockRobot.Feedback.INITIAL_PERCEPTION: "waiting for station marker",
    DockRobot.Feedback.CONTROLLING: "performing visual alignment",
    DockRobot.Feedback.WAIT_FOR_CHARGE: "waiting for charging current",
    DockRobot.Feedback.RETRY: "retrying docking",
}

PRECISE_GOAL_CHECKER = "goal_checker"
BIN_STAGING_GOAL_CHECKER = "bin_staging_goal_checker"
BIN_STATIONS = frozenset({"raw_bin", "finished_bin"})


def uses_direct_machine_alignment(station_name: str) -> bool:
    """Let Nav2 stop nearby, then use one continuous camera servo at CNCs."""
    return station_name.startswith("machine_")


def goal_checker_for_station(station_name: str) -> str:
    """Choose coarse bin staging without weakening CNC or charge docking."""
    if station_name in BIN_STATIONS:
        return BIN_STAGING_GOAL_CHECKER
    return PRECISE_GOAL_CHECKER


# The calibrated tag target keeps the mobile base outside the CNC enclosure
# while preserving the UR5e work envelope. The marker is offset from the
# workstation centre, so its residual is not the same as base truth error.
MACHINE_TAG_TARGET = TagAlignmentTarget(x=0.564, y=-0.359, yaw=1.555)
# Both bin tags are mounted 0.55 m to the robot's right at the taught pose.
# Their vertical orientation makes tag yaw singular, so bin refinement uses
# this calibrated translation and deliberately ignores tag heading.
RAW_BIN_TAG_TARGET = TagAlignmentTarget(x=0.570, y=-0.550, yaw=0.0)
FINISHED_BIN_TAG_TARGET = TagAlignmentTarget(x=0.650, y=-0.550, yaw=0.0)
# Each machine-axis residual is bounded at 10 mm. This keeps the radial base
# error below 15 mm and leaves margin for Tag-to-fixture lever-arm error inside
# the independent 30 mm CNC loading-datum gate.
MACHINE_POSITION_TOLERANCE = 0.010
# Lateral marker error maps directly to base cross-track error, so it keeps a
# tighter bound than the biased longitudinal marker measurement.
MACHINE_LATERAL_TOLERANCE = 0.010
BIN_LATERAL_TOLERANCE = 0.030
# Bins use the same 30 mm / 3 degree physical contract. One bounded odometry
# side step captures large cross-track errors; continuous camera servoing must
# close the remaining error rather than repeating blind diagonal moves.
BIN_POSITION_TOLERANCE = 0.030
BIN_HEADING_TOLERANCE = 0.0524
BIN_USE_ODOMETRY_CORRECTIONS = True


def _default_station_config() -> str:
    share = Path(get_package_share_directory("factory_core"))
    return str(share / "config/stations.yaml")


class DockStationClient:
    """Coordinate target selection with Nav2 without issuing base commands."""

    def __init__(
        self,
        node: Node,
        callback_wait: Callable[[], None] | None = None,
    ) -> None:
        self._node = node
        self._action = ActionClient(node, DockRobot, "/dock_robot")
        self._navigation = Nav2Client(node)
        self._lifecycle = node.create_client(
            GetState,
            "/docking_server/get_state",
        )
        self._callback_wait = callback_wait or (
            lambda: rclpy.spin_once(node, timeout_sec=0.05)
        )
        target_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._target_publisher = node.create_publisher(
            Int32, "/perception/target_tag_id", target_qos
        )
        self._goal_checker_publisher = node.create_publisher(
            String, "/goal_checker_selector", target_qos
        )
        self._last_feedback_state: int | None = None
        self._visual_aligner = VisualStationAligner(
            node, callback_wait=callback_wait
        )

    def wait_until_ready(self, timeout_sec: float = 30.0) -> bool:
        return self._action.wait_for_server(timeout_sec=timeout_sec)

    def wait_until_navigation_ready(self, timeout_sec: float = 30.0) -> bool:
        return self._navigation.wait_until_ready(timeout_sec=timeout_sec)

    def wait_until_active(self, timeout_sec: float = 30.0) -> bool:
        """Wait for the lifecycle state, not merely Action discovery."""

        deadline = time.monotonic() + timeout_sec
        if not self._lifecycle.wait_for_service(timeout_sec=timeout_sec):
            return False

        while time.monotonic() < deadline:
            future = self._lifecycle.call_async(GetState.Request())
            while time.monotonic() < deadline and not future.done():
                self._callback_wait()
            if not future.done():
                return False
            try:
                response = future.result()
            except Exception:
                response = None
            if (
                response is not None
                and response.current_state.id
                == State.PRIMARY_STATE_ACTIVE
            ):
                return True
            self._callback_wait()
        return False

    def select_goal_checker(self, station_name: str) -> None:
        goal_checker = goal_checker_for_station(station_name)
        self._goal_checker_publisher.publish(String(data=goal_checker))
        self._node.get_logger().info(
            f"Using {goal_checker} for {station_name} staging"
        )

    def send_staging_goal(self, station: StationDefinition):
        """Navigate to a configured staging pose without starting DockRobot."""
        self.select_goal_checker(station.name)
        pose = station.staging_pose
        return self._navigation.send_station(
            StationPose(
                name=station.name,
                x=pose.x,
                y=pose.y,
                yaw=pose.yaw,
            )
        )

    def select_target(self, tag_id: int) -> None:
        self._target_publisher.publish(Int32(data=tag_id))

    def clear_target(self) -> None:
        self._target_publisher.publish(Int32(data=-1))

    def send_goal(
        self, station_name: str, navigate_to_staging: bool, staging_timeout: float
    ):
        self.select_goal_checker(station_name)
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = station_name
        goal.navigate_to_staging_pose = navigate_to_staging
        goal.max_staging_time = staging_timeout
        return self._action.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )

    def align_machine_tag(self, tag_frame: str, timeout_sec: float):
        return self._visual_aligner.align(
            tag_frame,
            MACHINE_TAG_TARGET,
            timeout_sec=timeout_sec,
            position_tolerance=MACHINE_POSITION_TOLERANCE,
            lateral_tolerance=MACHINE_LATERAL_TOLERANCE,
            # The CNC marker is mounted outside the arm corridor, so the
            # bounded differential-drive side step can safely close the
            # coarse Nav2 cross-track residual before arm-frame planning.
            longitudinal_only=False,
            control_lateral=True,
            use_odometry_corrections=True,
        )

    def align_bin_pose(self, tag_frame: str, timeout_sec: float):
        raw_bin = tag_frame == "raw_bin_tag"
        target = (
            RAW_BIN_TAG_TARGET if raw_bin else FINISHED_BIN_TAG_TARGET
        )
        return self._visual_aligner.align(
            tag_frame,
            target,
            timeout_sec=timeout_sec,
            position_tolerance=BIN_POSITION_TOLERANCE,
            heading_tolerance=BIN_HEADING_TOLERANCE,
            heading_from_tag_normal=True,
            # A single bounded differential-drive side step captures a large
            # Nav2 cross-track residual. Camera feedback then closes reach,
            # lateral error and heading without another open-loop side step.
            lateral_tolerance=BIN_LATERAL_TOLERANCE,
            control_lateral=True,
            use_odometry_corrections=BIN_USE_ODOMETRY_CORRECTIONS,
        )

    def _on_feedback(self, message) -> None:
        state = int(message.feedback.state)
        if state == self._last_feedback_state:
            return
        self._last_feedback_state = state
        description = DOCKING_STATES.get(state, f"state {state}")
        self._node.get_logger().info(f"Docking: {description}")


def _run_machine_alignment(
    node: Node,
    client: DockStationClient,
    station: StationDefinition,
    navigate_to_staging: bool,
    staging_timeout: float,
) -> AlignmentError:
    """Navigate once and hand the final centimetres directly to vision."""
    if navigate_to_staging:
        if not client.wait_until_navigation_ready():
            raise RuntimeError("Nav2 action /navigate_to_pose is not available")
        node.get_logger().info(
            f"Navigating to {station.name} staging pose before visual alignment"
        )
        goal_handle = _wait(
            node,
            client.send_staging_goal(station),
            timeout_sec=15.0,
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Nav2 rejected station {station.name}")

        wrapped_result = _wait(
            node,
            goal_handle.get_result_async(),
            staging_timeout,
        )
        if wrapped_result is None:
            _wait(node, goal_handle.cancel_goal_async(), 5.0)
            raise RuntimeError(
                f"Navigation to {station.name} staging pose timed out"
            )
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

    node.get_logger().info(
        "Starting continuous AprilTag alignment; DockRobot local approach "
        "is intentionally skipped for CNC stations"
    )
    return client.align_machine_tag(
        f"{station.name}_tag",
        timeout_sec=75.0,
    )


def _run_docking_server(
    node: Node,
    client: DockStationClient,
    station: StationDefinition,
    navigate_to_staging: bool,
    staging_timeout: float,
    docking_timeout: float,
) -> None:
    """Execute Nav2 DockRobot for bins and the charging station."""
    if not client.wait_until_ready():
        raise RuntimeError("Nav2 action /dock_robot is not available")
    goal_handle = _wait(
        node,
        client.send_goal(
            station.name,
            navigate_to_staging,
            staging_timeout,
        ),
        timeout_sec=15.0,
    )
    if goal_handle is None or not goal_handle.accepted:
        raise RuntimeError(f"Docking server rejected {station.name}")

    wrapped_result = _wait(
        node,
        goal_handle.get_result_async(),
        docking_timeout,
    )
    if wrapped_result is None:
        _wait(node, goal_handle.cancel_goal_async(), 5.0)
        raise RuntimeError(f"Docking at {station.name} timed out")
    result = wrapped_result.result
    if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
        raise RuntimeError(
            f"Docking failed: status={wrapped_result.status} "
            f"code={result.error_code} detail={result.error_msg}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("factory_dock_station")
    node.declare_parameter("station", "raw_bin")
    node.declare_parameter("station_config", _default_station_config())
    node.declare_parameter("navigate_to_staging", True)
    node.declare_parameter("staging_timeout", 180.0)
    node.declare_parameter("docking_timeout", 240.0)
    node.declare_parameter("refine_bin_pose", True)

    station_name = str(node.get_parameter("station").value)
    station_config = str(node.get_parameter("station_config").value)
    navigate_to_staging = bool(node.get_parameter("navigate_to_staging").value)
    staging_timeout = float(node.get_parameter("staging_timeout").value)
    docking_timeout = float(node.get_parameter("docking_timeout").value)
    refine_bin_pose = bool(
        node.get_parameter("refine_bin_pose").value
    )
    client = DockStationClient(node)

    try:
        stations = load_station_definitions(station_config)
        if station_name not in stations:
            raise ValueError(f"Unknown station: {station_name}")
        station = stations[station_name]

        client.select_target(station.tag_id)
        rclpy.spin_once(node, timeout_sec=0.25)
        node.get_logger().info(
            f"Docking at {station.name} using AprilTag {station.tag_id}"
        )
        if uses_direct_machine_alignment(station.name):
            error = _run_machine_alignment(
                node,
                client,
                station,
                navigate_to_staging,
                staging_timeout,
            )
        else:
            _run_docking_server(
                node,
                client,
                station,
                navigate_to_staging,
                staging_timeout,
                docking_timeout,
            )
            if station_name in BIN_STATIONS and refine_bin_pose:
                node.get_logger().info(
                    "Nav2 docking complete; applying optional bin calibration"
                )
                error = client.align_bin_pose(
                    f"{station_name}_tag", timeout_sec=180.0
                )
            else:
                error = None
        if error is not None:
            node.get_logger().info(
                "Final AprilTag alignment complete: "
                f"dx={error.longitudinal:.3f} m, dy={error.lateral:.3f} m, "
                f"dyaw={error.heading:.3f} rad"
            )
        node.get_logger().info(f"Docked at {station.name}")
    finally:
        client.clear_target()
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _wait(node: Node, future, timeout_sec: float):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    return future.result() if future.done() else None
