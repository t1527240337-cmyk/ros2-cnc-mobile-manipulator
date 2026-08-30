"""Provision an unordered raw-material workspace for physical simulation."""

from __future__ import annotations

import math
import shutil
import subprocess
import time

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from .unordered_layout import LayoutPoint, WorkspaceBounds, sample_unordered_layout


PART_IDS = tuple(f"raw_part_{index}" for index in range(1, 7))
UNORDERED_BOUNDS = WorkspaceBounds(x=(-4.24, -4.04), y=(-3.00, -2.40))
MINIMUM_CENTER_DISTANCE = 0.13


class RawBinRandomizer(Node):
    """Place workpieces anywhere in one bounded, collision-safe workspace."""

    def __init__(self) -> None:
        super().__init__("raw_bin_randomizer")
        self.declare_parameter("seed", 7)
        self.declare_parameter("active_part_count", len(PART_IDS))
        self.declare_parameter("randomize_positions", True)
        self.declare_parameter(
            "set_pose_service", "/world/multi_machine_factory/set_pose"
        )
        self._seed = int(self.get_parameter("seed").value)
        self._service_name = str(
            self.get_parameter("set_pose_service").value
        )
        self._gz_executable = shutil.which("gz")
        self._active_part_count = int(
            self.get_parameter("active_part_count").value
        )
        if not 1 <= self._active_part_count <= len(PART_IDS):
            raise ValueError(
                f"active_part_count must be between 1 and {len(PART_IDS)}"
            )
        self._randomize_positions = bool(
            self.get_parameter("randomize_positions").value
        )

        if self._gz_executable is None:
            raise RuntimeError("Gazebo command-line tools are unavailable")
        self._detach_publishers = {
            part_id: self.create_publisher(
                Empty, f"/factory/fixture/{part_id}/detach", 1
            )
            for part_id in PART_IDS
        }
        self._attach_publishers = {
            part_id: self.create_publisher(
                Empty, f"/factory/fixture/{part_id}/attach", 1
            )
            for part_id in PART_IDS
        }
        self._fixture_states: dict[str, bool | None] = {
            part_id: None for part_id in PART_IDS
        }
        self._state_subscriptions = [
            self.create_subscription(
                String,
                f"/factory/fixture/{part_id}/attached",
                lambda message, current=part_id: self._remember_fixture_state(
                    current, message
                ),
                10,
            )
            for part_id in PART_IDS
        ]

    def run(self) -> None:
        self._command_until_fixture_state(
            {part_id: False for part_id in PART_IDS},
            self._detach_publishers,
            action="release before provisioning",
        )
        layout = self._layout()
        for index, part_id in enumerate(PART_IDS):
            active = index < self._active_part_count
            pose = (
                self._pose(layout[index])
                if active
                else self._parking_pose(index)
            )
            self._set_pose(part_id, pose)
            if active:
                self.get_logger().info(
                    f"{part_id} provisioned at "
                    f"({pose.position.x:.3f}, {pose.position.y:.3f})"
                )

        expected_states = {
            part_id: index < self._active_part_count
            for index, part_id in enumerate(PART_IDS)
        }
        final_publishers = {
            part_id: (
                self._attach_publishers[part_id]
                if expected_states[part_id]
                else self._detach_publishers[part_id]
            )
            for part_id in PART_IDS
        }
        self._command_until_fixture_state(
            expected_states,
            final_publishers,
            action="apply the provisioned fixture state",
        )
        self.get_logger().info(
            "Unordered raw-material workspace is ready with "
            f"{self._active_part_count} active part(s); seed={self._seed}, "
            f"minimum_center_distance={MINIMUM_CENTER_DISTANCE:.3f} m"
        )

    def _layout(self) -> tuple[LayoutPoint, ...]:
        if self._randomize_positions:
            return sample_unordered_layout(
                seed=self._seed,
                count=self._active_part_count,
                bounds=UNORDERED_BOUNDS,
                minimum_center_distance=MINIMUM_CENTER_DISTANCE,
            )
        return sample_unordered_layout(
            seed=0,
            count=self._active_part_count,
            bounds=UNORDERED_BOUNDS,
            minimum_center_distance=MINIMUM_CENTER_DISTANCE,
        )

    def _set_pose(self, part_id: str, pose: Pose) -> None:
        """Call Gazebo's native UserCommands service.

        Jazzy's standard ros_gz bridge does not expose ``gz.msgs.Pose`` as a
        ROS service. Keeping this simulator-only adapter here prevents Gazebo
        transport details from leaking into perception or task execution.
        """

        request = (
            f'name: "{part_id}" '
            f"position {{ x: {pose.position.x} y: {pose.position.y} "
            f"z: {pose.position.z} }} "
            f"orientation {{ x: {pose.orientation.x} y: {pose.orientation.y} "
            f"z: {pose.orientation.z} w: {pose.orientation.w} }}"
        )
        completed = subprocess.run(
            [
                self._gz_executable,
                "service",
                "-s",
                self._service_name,
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                request,
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=7.0,
        )
        if completed.returncode != 0 or "true" not in completed.stdout.lower():
            detail = completed.stderr.strip() or completed.stdout.strip()
            raise RuntimeError(f"could not move {part_id}: {detail}")

    @staticmethod
    def _pose(point: LayoutPoint) -> Pose:
        pose = Pose()
        pose.position.x = point.x
        pose.position.y = point.y
        pose.position.z = 0.614
        pose.orientation.z = math.sin(point.yaw / 2.0)
        pose.orientation.w = math.cos(point.yaw / 2.0)
        return pose

    @staticmethod
    def _parking_pose(index: int) -> Pose:
        """Keep inactive pool entities outside the physical workspace."""

        pose = Pose()
        pose.position.x = -6.0
        pose.position.y = -4.5 - 0.2 * index
        pose.position.z = -1.0
        pose.orientation.w = 1.0
        return pose

    def _remember_fixture_state(
        self, part_id: str, message: String
    ) -> None:
        self._fixture_states[part_id] = message.data == "attached"

    def _command_until_fixture_state(
        self,
        expected_states: dict[str, bool],
        publishers: dict[str, object],
        *,
        action: str,
    ) -> None:
        """Repeat a simulator command until every fixture confirms it."""
        deadline = time.monotonic() + 12.0
        next_command = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            unresolved = {
                part_id: (self._fixture_states[part_id], expected)
                for part_id, expected in expected_states.items()
                if self._fixture_states[part_id] != expected
            }
            if not unresolved:
                return
            now = time.monotonic()
            if now >= next_command:
                for part_id in unresolved:
                    publishers[part_id].publish(Empty())
                next_command = now + 0.25
            time.sleep(0.02)

        detail = ", ".join(
            f"{part_id}=observed:{observed}/expected:{expected}"
            for part_id, (observed, expected) in unresolved.items()
        )
        raise RuntimeError(
            f"could not {action}; fixture confirmations: {detail}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RawBinRandomizer()
    try:
        node.run()
    except Exception as error:
        node.get_logger().error(str(error))
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
