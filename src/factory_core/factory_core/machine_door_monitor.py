"""Measured CNC-door state used as a manipulation safety interlock."""

from __future__ import annotations

from dataclasses import dataclass
from functools import partial
import threading
import time

from rclpy.node import Node
from sensor_msgs.msg import JointState


DOOR_JOINT_NAME = "sliding_door_joint"
OPEN_POSITION_METERS = 1.18
OPEN_POSITION_TOLERANCE = 0.05


@dataclass(frozen=True)
class DoorPositionSample:
    """One measured door position and its local receive time."""

    position: float
    received_at: float


def joint_position(
    message: JointState, joint_name: str = DOOR_JOINT_NAME
) -> float | None:
    """Extract one named position without relying on array order."""
    try:
        index = message.name.index(joint_name)
    except ValueError:
        return None
    if index >= len(message.position):
        return None
    return float(message.position[index])


class MachineDoorMonitor:
    """Collect independent Gazebo feedback for each CNC sliding door."""

    def __init__(self, node: Node, machine_ids: tuple[str, ...]) -> None:
        self._samples: dict[str, DoorPositionSample] = {}
        self._lock = threading.Lock()
        self._subscriptions = [
            node.create_subscription(
                JointState,
                f"/{machine_id}/door_joint_state",
                partial(self._remember, machine_id),
                10,
            )
            for machine_id in machine_ids
        ]

    def latest(self, machine_id: str) -> DoorPositionSample | None:
        """Return a snapshot safe to read from an action worker thread."""
        with self._lock:
            return self._samples.get(machine_id)

    @staticmethod
    def is_open(sample: DoorPositionSample) -> bool:
        """Use the simulated limit-switch tolerance, not command intent."""
        return sample.position >= (
            OPEN_POSITION_METERS - OPEN_POSITION_TOLERANCE
        )

    def _remember(self, machine_id: str, message: JointState) -> None:
        position = joint_position(message)
        if position is None:
            return
        sample = DoorPositionSample(
            position=position,
            received_at=time.monotonic(),
        )
        with self._lock:
            self._samples[machine_id] = sample
