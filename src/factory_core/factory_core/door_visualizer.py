from __future__ import annotations

from functools import partial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from factory_interfaces.msg import MachineState


class DoorVisualizer(Node):
    """Mirrors validated semantic door state to Gazebo's prismatic joints."""

    OPEN_POSITION_METERS = 1.18

    def __init__(self) -> None:
        super().__init__("machine_door_visualizer")
        # The name is intentionally distinct from rclpy.Node._publishers.
        self._door_publishers = {}
        for index in range(1, 4):
            machine_id = f"machine_{index}"
            self._door_publishers[machine_id] = self.create_publisher(
                Float64, f"/{machine_id}/door_position_cmd", 10
            )
            self.create_subscription(
                MachineState,
                f"/{machine_id}/state",
                partial(self._publish_position, machine_id),
                10,
            )

    def _publish_position(self, machine_id: str, state: MachineState) -> None:
        command = Float64()
        command.data = self.OPEN_POSITION_METERS if state.door_open else 0.0
        self._door_publishers[machine_id].publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DoorVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
