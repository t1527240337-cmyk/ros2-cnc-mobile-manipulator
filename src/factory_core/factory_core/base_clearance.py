"""Measured station-clearance motion after Nav2 releases a dock."""

from __future__ import annotations

import time
from typing import Callable

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from .planar_velocity import planar_distance


class BaseClearanceController:
    """Back away from a station by an odometry-measured distance.

    Nav2's undock action establishes that physical dock contact has ended.
    This controller establishes the separate factory invariant that the full
    loaded footprint has cleared the workstation before path following starts.
    Commands enter the same /cmd_vel pipeline as the docking server.
    """

    def __init__(
        self,
        node: Node,
        command_topic: str = "/cmd_vel",
        odom_topic: str = "/odometry/filtered",
        callback_wait: Callable[[], None] | None = None,
    ) -> None:
        self._node = node
        self._callback_wait = callback_wait
        self._position: tuple[float, float] | None = None
        self._publisher = node.create_publisher(Twist, command_topic, 10)
        self._subscription = node.create_subscription(
            Odometry, odom_topic, self._on_odometry, 10
        )

    def retreat(
        self,
        distance: float,
        speed: float,
        timeout_sec: float,
    ) -> float:
        """Retreat in body -X and return the measured planar displacement."""
        if distance < 0.0:
            raise ValueError("clearance distance must be non-negative")
        if speed <= 0.0:
            raise ValueError("clearance speed must be positive")
        if timeout_sec <= 0.0:
            raise ValueError("clearance timeout must be positive")
        if distance == 0.0:
            return 0.0

        self._wait_for_odometry(min(5.0, timeout_sec))
        if self._position is None:
            raise RuntimeError("base odometry is unavailable for clearance retreat")

        start = self._position
        deadline = time.monotonic() + timeout_sec
        command = Twist()
        command.linear.x = -speed
        travelled = 0.0

        try:
            while time.monotonic() < deadline:
                self._publisher.publish(command)
                self._wait_for_callbacks()
                if self._position is None:
                    continue
                travelled = planar_distance(start, self._position)
                if travelled >= distance:
                    return travelled
        finally:
            self._stop()

        raise RuntimeError(
            f"station clearance timed out after {travelled:.3f} m "
            f"(required {distance:.3f} m)"
        )

    def _wait_for_odometry(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while self._position is None and time.monotonic() < deadline:
            self._wait_for_callbacks()

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._position = (float(position.x), float(position.y))

    def _stop(self) -> None:
        self._publisher.publish(Twist())
        self._wait_for_callbacks()
        self._publisher.publish(Twist())

    def _wait_for_callbacks(self) -> None:
        if self._callback_wait is not None:
            self._callback_wait()
            return
        rclpy.spin_once(self._node, timeout_sec=0.05)
