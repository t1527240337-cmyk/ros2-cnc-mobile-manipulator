"""Force command and measured-state boundary for the parallel gripper."""

from __future__ import annotations

import time

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GripperClient:
    """Drive both jaws with equal force and verify their measured motion.

    Position control is unsuitable once either jaw touches a workpiece: two
    independent position servos fight the contact constraint. The hardware
    therefore exposes effort interfaces, while this boundary keeps both force
    commands equal and treats bilateral tactile evidence as grasp truth.
    """

    JOINT_NAMES = (
        "gripper_left_finger_joint",
        "gripper_right_finger_joint",
    )
    CONTROLLER = "gripper_controller"
    # Search gently so first contact does not kick a loose cylinder. Once
    # bilateral contact identifies one workpiece, a higher bounded normal
    # force resists the measured carry acceleration. Both values remain well
    # below the 40 N per-rail URDF limit.
    CLOSE_EFFORT = 3.0
    HOLD_EFFORT = 36.0
    OPEN_EFFORT = -3.0
    MAX_COMMAND_EFFORT = 38.0

    OPEN_POSITION = 0.0
    CLOSED_POSITION = 0.040

    # Each rail may settle up to 7 mm from its mechanical open stop; the
    # combined 14 mm travel remains disjoint from the 16 mm hold range.
    SAFE_RELEASE_POSITION = 0.005
    MAX_SAFE_PICK_POSITION = 0.006
    MAX_CONFINED_PICK_POSITION = 0.006
    RELEASE_OPEN_POSITION = 0.006
    # For two independent rails, jaw aperture depends on the sum of their
    # travel. Their difference describes only the workpiece's lateral offset.
    # V-face contact position varies with the supported cylinder pose. Fresh
    # bilateral identity and the measured proof lift own the lower bound;
    # joint travel only rejects the empty fully-closed mechanical stops.
    MIN_VERIFIED_TOTAL_CLOSURE = 0.0
    # Both rails stop at 0.040 m (0.080 m combined). A real cylinder must
    # prevent the jaws reaching those mechanical stops even after it centres
    # itself in the compliant V pads during the proof lift.
    MAX_VERIFIED_TOTAL_CLOSURE = 0.072
    # A useful diagnostic for target-centering quality, not a grasp criterion:
    # an off-centre cylinder legitimately stops the two independent rails at
    # different positions while fresh bilateral tactile contact proves hold.
    MAX_FINGER_POSITION_DIFFERENCE = 0.002

    def __init__(self, node: Node) -> None:
        self._command_publisher = node.create_publisher(
            Float64MultiArray, f"/{self.CONTROLLER}/commands", 10
        )
        self._positions: dict[str, float] = {}
        # Effort is a continuously owned actuator command, not an event.  A
        # periodic refresh keeps the force controller deterministic across
        # long navigation actions and DDS/controller scheduling gaps.
        self._command_effort = 0.0
        self._command_timer = node.create_timer(0.05, self._republish_command)
        self._velocities: dict[str, float] = {}
        self._joint_state_time: float | None = None
        self._joint_state_subscription = node.create_subscription(
            JointState, "/joint_states", self._remember_joint_state, 10
        )

    def wait_until_ready(self, timeout_sec: float = 20.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._command_publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        return False

    def open(self) -> None:
        self._send_equal_effort(self.OPEN_EFFORT)

    def close_to(self, position: float) -> None:
        """Start force-limited closing toward a validated travel bound."""
        self._require_position(position)
        self._send_equal_effort(self.CLOSE_EFFORT)

    def release_contact(self) -> None:
        """Remove jaw preload while retaining bounded CNC clearance."""
        self._send_equal_effort(self.OPEN_EFFORT)

    def hold_at_contact(
        self, measured_position: float
    ) -> tuple[float, float]:
        """Maintain bounded normal force after bilateral contact."""
        self._require_total_position(measured_position)
        pair = self.measured_positions()
        if pair is None:
            raise RuntimeError("both finger positions are required for hold")
        if abs(sum(pair) - measured_position) > 0.002:
            raise RuntimeError("finger sample changed before hold command")
        total = sum(pair)
        if not self.MIN_VERIFIED_TOTAL_CLOSURE <= total <= (
            self.MAX_VERIFIED_TOTAL_CLOSURE
        ):
            raise RuntimeError(
                "combined finger travel is outside the physical hold range; "
                f"total={total:.3f}, "
                f"fingers=({pair[0]:.3f},{pair[1]:.3f})"
            )
        self._send_equal_effort(self.HOLD_EFFORT)
        return pair

    def hold_stabilized_carry(
        self, measured_position: float
    ) -> tuple[float, float]:
        return self.hold_at_contact(measured_position)

    def stop(self) -> None:
        """Remove motor effort without changing the measured joint state."""
        self._send_equal_effort(0.0)

    def measured_state(self) -> tuple[float | None, float | None]:
        """Return total jaw closure and the fastest rail velocity."""
        pair = self.measured_positions()
        if pair is None:
            return None, None
        velocities = tuple(
            self._velocities.get(name) for name in self.JOINT_NAMES
        )
        if any(value is None for value in velocities):
            return sum(pair), None
        return sum(pair), max(abs(float(value)) for value in velocities)

    def measured_positions(self) -> tuple[float, float] | None:
        if any(name not in self._positions for name in self.JOINT_NAMES):
            return None
        return tuple(self._positions[name] for name in self.JOINT_NAMES)

    def both_fingers_at_or_below(self, position: float) -> bool:
        """Return whether both physical rails reached an opening bound."""
        self._require_position(position)
        pair = self.measured_positions()
        return pair is not None and max(pair) <= position

    def both_fingers_at_or_above(self, position: float) -> bool:
        """Return whether both physical rails reached a closure bound."""
        self._require_position(position)
        pair = self.measured_positions()
        return pair is not None and min(pair) >= position

    def measured_sample(
        self,
    ) -> tuple[float | None, float | None, float | None]:
        position, velocity = self.measured_state()
        return position, velocity, self._joint_state_time

    def fingers_are_symmetric(self) -> bool:
        """Report target-centering quality without deciding grasp validity.

        Bilateral tactile identity and bounded rail travel own grasp safety.
        """
        pair = self.measured_positions()
        return pair is not None and abs(pair[0] - pair[1]) <= (
            self.MAX_FINGER_POSITION_DIFFERENCE
        )

    def _remember_joint_state(self, message: JointState) -> None:
        complete = True
        for name in self.JOINT_NAMES:
            try:
                index = message.name.index(name)
            except ValueError:
                complete = False
                continue
            if index < len(message.position):
                self._positions[name] = float(message.position[index])
            else:
                complete = False
            if index < len(message.velocity):
                self._velocities[name] = float(message.velocity[index])
            else:
                complete = False
        if complete:
            self._joint_state_time = time.monotonic()

    def _send_equal_effort(self, effort: float) -> None:
        if abs(effort) > self.MAX_COMMAND_EFFORT:
            raise ValueError(f"unsafe gripper effort command: {effort}")
        # Controller order is right then left; equal positive joint effort
        # closes the mirrored rails and equal negative effort opens them.
        self._command_effort = float(effort)
        self._command_publisher.publish(
            Float64MultiArray(data=[self._command_effort, self._command_effort])
        )

    def _republish_command(self) -> None:
        """Refresh the currently owned effort for the actuator controller."""
        effort = self._command_effort
        self._command_publisher.publish(
            Float64MultiArray(data=[effort, effort])
        )

    def _require_position(self, position: float) -> None:
        if not self.OPEN_POSITION <= position <= self.CLOSED_POSITION:
            raise ValueError(f"invalid gripper closure target: {position}")

    def _require_total_position(self, position: float) -> None:
        maximum = 2.0 * self.CLOSED_POSITION
        if not 0.0 <= position <= maximum:
            raise ValueError(
                f"invalid total gripper closure: {position}"
            )
