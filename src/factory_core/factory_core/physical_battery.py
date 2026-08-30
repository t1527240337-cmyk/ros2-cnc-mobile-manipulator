"""Physical battery model driven by Gazebo robot motion and dock contact."""

from __future__ import annotations

import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import BatteryState, JointState

# Sensor estimates can spike while Gazebo inserts the model or resets time.
# Bound energy inputs by the simulated robot's configured capabilities so a
# single bad sample cannot consume most of the battery.
MAX_LINEAR_SPEED_MPS = 1.0
MAX_ANGULAR_SPEED_RAD_S = 2.0
MAX_ARM_JOINT_SPEED_RAD_S = 3.5

ARM_JOINTS = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "finger_joint",
}
ARM_JOINTS.update(
    {
        f"arm_{name}"
        for name in tuple(ARM_JOINTS)
        if name != "finger_joint"
    }
)
CHARGING_CURRENT_AMPS = 5.0
DISCHARGING_CURRENT_AMPS = -1.0


def charging_contacts_present(message: Contacts) -> bool:
    """Require a collision pair between the robot and dock contact pads."""

    for contact in message.contacts:
        names = (contact.collision1.name, contact.collision2.name)
        robot_contact = any(
            "charging_contact_collision" in name and "factory_robot" in name
            for name in names
        )
        dock_contact = any(
            "charging_contact_collision" in name and "charge_dock" in name
            for name in names
        )
        if robot_contact and dock_contact:
            return True
    return False


def bounded_speed(value: float, maximum: float) -> float:
    """Return a finite, non-negative speed limited to robot capability."""
    if maximum <= 0.0:
        raise ValueError("maximum speed must be positive")
    if not math.isfinite(value):
        return 0.0
    return min(abs(value), maximum)


class BatteryModel:
    """Integrate deterministic charge and motion-dependent discharge rates."""

    def __init__(
        self,
        initial_percentage: float,
        *,
        charge_rate: float,
        idle_drain_rate: float,
        linear_drain_rate: float,
        angular_drain_rate: float,
        arm_drain_rate: float,
    ) -> None:
        if not 0.0 <= initial_percentage <= 1.0:
            raise ValueError("initial_percentage must be in [0, 1]")
        rates = (
            charge_rate,
            idle_drain_rate,
            linear_drain_rate,
            angular_drain_rate,
            arm_drain_rate,
        )
        if any(rate < 0.0 for rate in rates) or charge_rate == 0.0:
            raise ValueError(
                "charge_rate must be positive and drain rates non-negative"
            )

        self.percentage = initial_percentage
        self.charging = False
        self._charge_rate = charge_rate
        self._idle_drain_rate = idle_drain_rate
        self._linear_drain_rate = linear_drain_rate
        self._angular_drain_rate = angular_drain_rate
        self._arm_drain_rate = arm_drain_rate

    def advance(
        self,
        elapsed: float,
        *,
        charging_contact: bool,
        linear_speed: float,
        angular_speed: float,
        arm_speed_sum: float,
    ) -> float:
        """Advance charge by elapsed wall-clock-equivalent simulation time."""
        if elapsed < 0.0:
            raise ValueError("elapsed must be non-negative")

        self.charging = charging_contact
        if charging_contact:
            change = self._charge_rate * elapsed
        else:
            drain_rate = (
                self._idle_drain_rate
                + self._linear_drain_rate * abs(linear_speed)
                + self._angular_drain_rate * abs(angular_speed)
                + self._arm_drain_rate * abs(arm_speed_sum)
            )
            change = -drain_rate * elapsed

        self.percentage = min(1.0, max(0.0, self.percentage + change))
        return self.percentage


class PhysicalBatteryNode(Node):
    """Publish physical battery state without owning task or Agent policy."""

    def __init__(self) -> None:
        super().__init__("physical_battery")
        self.declare_parameter("initial_percentage", 0.42)
        self.declare_parameter("charge_rate", 0.02)
        self.declare_parameter("idle_drain_rate", 0.00002)
        self.declare_parameter("linear_drain_rate", 0.00040)
        self.declare_parameter("angular_drain_rate", 0.00010)
        self.declare_parameter("arm_drain_rate", 0.00003)
        self._battery = BatteryModel(
            float(self.get_parameter("initial_percentage").value),
            charge_rate=float(self.get_parameter("charge_rate").value),
            idle_drain_rate=float(
                self.get_parameter("idle_drain_rate").value
            ),
            linear_drain_rate=float(
                self.get_parameter("linear_drain_rate").value
            ),
            angular_drain_rate=float(
                self.get_parameter("angular_drain_rate").value
            ),
            arm_drain_rate=float(
                self.get_parameter("arm_drain_rate").value
            ),
        )
        self._charging_contact_received_at: float | None = None
        self._linear_speed = 0.0
        self._angular_speed = 0.0
        self._arm_speed_sum = 0.0
        self._last_tick_ns: int | None = None
        self._last_contact = False

        self._publisher = self.create_publisher(
            BatteryState, "/factory/physical_battery_state", 10
        )
        self.create_subscription(
            Odometry, "/base_controller/odom", self._on_odometry, 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self.create_subscription(
            Contacts,
            "/factory/charging/contacts",
            self._on_charging_contacts,
            10,
        )
        self.create_timer(0.20, self._update)

    def _on_odometry(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        self._linear_speed = bounded_speed(
            math.hypot(linear.x, linear.y),
            MAX_LINEAR_SPEED_MPS,
        )
        self._angular_speed = bounded_speed(
            message.twist.twist.angular.z,
            MAX_ANGULAR_SPEED_RAD_S,
        )

    def _on_charging_contacts(self, message: Contacts) -> None:
        self._charging_contact_received_at = (
            time.monotonic() if charging_contacts_present(message) else None
        )

    def _on_joint_state(self, message: JointState) -> None:
        velocities = zip(message.name, message.velocity)
        self._arm_speed_sum = sum(
            bounded_speed(velocity, MAX_ARM_JOINT_SPEED_RAD_S)
            for name, velocity in velocities
            if name in ARM_JOINTS
        )

    def _update(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_tick_ns is None:
            self._last_tick_ns = now_ns
            return
        elapsed = (now_ns - self._last_tick_ns) / 1e9
        self._last_tick_ns = now_ns
        if elapsed <= 0.0:
            return
        elapsed = min(elapsed, 1.0)

        contact = (
            self._charging_contact_received_at is not None
            and time.monotonic() - self._charging_contact_received_at <= 0.5
        )
        self._battery.advance(
            elapsed,
            charging_contact=contact,
            linear_speed=self._linear_speed,
            angular_speed=self._angular_speed,
            arm_speed_sum=self._arm_speed_sum,
        )
        if contact != self._last_contact:
            state = "established" if contact else "released"
            self.get_logger().info(f"Charging contact {state}")
            self._last_contact = contact
        self._publisher.publish(self._message())

    def _message(self) -> BatteryState:
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.percentage = self._battery.percentage
        message.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if self._battery.charging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        message.current = (
            CHARGING_CURRENT_AMPS
            if self._battery.charging
            else DISCHARGING_CURRENT_AMPS
        )
        message.present = True
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhysicalBatteryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
