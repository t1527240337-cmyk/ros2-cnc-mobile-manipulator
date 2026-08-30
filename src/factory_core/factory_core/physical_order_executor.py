"""Atomic ROS 2 worker for physically verified factory capabilities."""

from __future__ import annotations

from pathlib import Path
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from factory_interfaces.action import ExecutePhysicalStep, ManipulatePart
from factory_interfaces.msg import MachineState
from factory_interfaces.srv import ControlTask, MachineCommand, PartTransfer
from moveit_msgs.msg import MoveItErrorCodes
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from .base_clearance import BaseClearanceController
from .base_kinematics import is_planar_motion_settled
from .dock_station import (
    DockStationClient,
    uses_direct_machine_alignment,
)
from .finished_slot_perception import (
    FinishedSlotPerception,
    unreserved_slot_preferences,
)
from .motion_client import MoveGroupClient, STOWED, moveit_succeeded
from .physical_step_server import PhysicalStepServer
from .physical_order_plan import (
    EnergyDecision,
    MachineOperation,
    Manipulation,
    PhysicalStep,
    StepKind,
    TransferEvent,
    energy_decision,
)
from .station_config import load_station_definitions
from .undock_station import (
    UndockStationClient,
    remaining_clearance_after_nav2_undock,
    uses_nav2_undocking,
)


class PhysicalOrderError(RuntimeError):
    """A bounded physical workflow failure suitable for an action result."""


class PhysicalMachineFault(PhysicalOrderError):
    """A workpiece is locked inside a CNC that requires an operator."""

    def __init__(
        self,
        machine_id: str,
        part_id: str,
        fault_code: int,
    ) -> None:
        self.machine_id = machine_id
        self.part_id = part_id
        self.fault_code = fault_code
        super().__init__(
            f"{machine_id} faulted with code {fault_code}; "
            f"{part_id or 'a workpiece'} is trapped in the fixture and "
            "requires manual intervention"
        )


class PhysicalOrderIncomplete(PhysicalOrderError):
    """Healthy work completed, but one or more trapped parts remain."""


class PhysicalOrderCancelled(RuntimeError):
    """Operator cancellation observed at a safe execution boundary."""


def _default_station_config() -> str:
    share = Path(get_package_share_directory("factory_core"))
    return str(share / "config" / "stations.yaml")


class PhysicalOrderExecutor(Node):
    """Execute one validated physical step requested by BehaviorTree.CPP.

    The behavior tree owns task order and failure flow. This worker composes
    Nav2, docking, MoveIt and PLC APIs without publishing base or joint
    commands directly. Semantic inventory changes only after physical success.
    """

    def __init__(self) -> None:
        super().__init__("physical_order_executor")
        self._declare_parameters()
        station_config = str(self.get_parameter("station_config").value)
        self._stations = load_station_definitions(station_config)
        self._machine_ids = tuple(
            name for name in self._stations if name.startswith("machine_")
        )
        self._use_finished_slot_perception = bool(
            self.get_parameter("use_finished_slot_perception").value
        )
        if not self._use_finished_slot_perception:
            raise ValueError(
                "finished-bin placement requires RGB-D occupancy perception"
            )
        self._finished_slot_perception_timeout = float(
            self.get_parameter("finished_slot_perception_timeout").value
        )
        self._finished_slot_order = tuple(
            int(slot_id)
            for slot_id in self.get_parameter("finished_slot_order").value
        )
        if self._finished_slot_perception_timeout <= 0.0:
            raise ValueError("finished-slot perception timeout must be positive")
        if not self._finished_slot_order or any(
            slot_id < 1 for slot_id in self._finished_slot_order
        ):
            raise ValueError("finished_slot_order must contain positive ids")
        if len(set(self._finished_slot_order)) != len(self._finished_slot_order):
            raise ValueError("finished_slot_order cannot contain duplicates")

        self._callbacks = ReentrantCallbackGroup()
        self._state_lock = threading.RLock()
        self._goal_reserved = False
        self._active_order_id = ""
        self._pause_requested = False
        self._cancel_requested = False
        self._machine_states: dict[str, MachineState] = {}
        self._machine_state_received_at: dict[str, float] = {}
        self._battery_percentage: float | None = None
        self._base_motion_sample: tuple[float, float, float, float] | None = None
        # A successful physical PLACE reserves its destination until this
        # executor restarts with a fresh Gazebo world. Perception still proves
        # a candidate empty; this memory prevents a settled or rolled part
        # from becoming eligible again after it leaves a narrow depth ROI.
        self._placed_finished_slots: set[int] = set()
        self._pending_finished_slot: int | None = None
        # Logical IDs belong to orders; simulator IDs are learned only from
        # bilateral contact and translated at simulator-facing boundaries.
        self._part_identity: dict[str, str] = {}
        # A controlled retreat from a CNC establishes a local, observable
        # re-entry corridor. It remains valid only until another station is
        # visited, and avoids asking Nav2 to plan from its own inflated wall.
        self._departed_station_id: str | None = None


        callback_wait = lambda: time.sleep(0.05)
        self._dock = DockStationClient(self, callback_wait=callback_wait)
        self._undock = UndockStationClient(self)
        self._clearance = BaseClearanceController(
            self, callback_wait=callback_wait
        )
        self._finished_slots = FinishedSlotPerception(self)
        self._arm = MoveGroupClient(self)
        self._manipulation = ActionClient(
            self,
            ManipulatePart,
            "/manipulate_part",
            callback_group=self._callbacks,
        )
        self._machine_command_client = self.create_client(
            MachineCommand,
            "/factory/machine_command",
            callback_group=self._callbacks,
        )
        self._part_transfer_client = self.create_client(
            PartTransfer,
            "/factory/part_transfer",
            callback_group=self._callbacks,
        )
        for machine_id in self._machine_ids:
            self.create_subscription(
                MachineState,
                f"/{machine_id}/state",
                self._remember_machine_state,
                10,
                callback_group=self._callbacks,
            )
        self.create_subscription(
            BatteryState,
            "/battery_state",
            self._remember_battery,
            10,
            callback_group=self._callbacks,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("base_odometry_topic").value),
            self._remember_base_motion,
            10,
            callback_group=self._callbacks,
        )
        self.create_service(
            ControlTask,
            "/factory/control_task",
            self._control_task,
            callback_group=self._callbacks,
        )
        self._physical_step_server = PhysicalStepServer(
            self, self._callbacks
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("station_config", _default_station_config())
        self.declare_parameter("dependency_timeout", 90.0)
        self.declare_parameter("staging_timeout", 180.0)
        self.declare_parameter("docking_timeout", 300.0)
        self.declare_parameter("undocking_timeout", 75.0)
        self.declare_parameter("clearance_distance", 0.30)
        self.declare_parameter("raw_perception_settle_time", 0.5)
        self.declare_parameter("use_finished_slot_perception", True)
        self.declare_parameter("finished_slot_perception_timeout", 8.0)
        self.declare_parameter("finished_slot_order", [2, 1, 4, 3])
        self.declare_parameter("base_odometry_topic", "/odometry/filtered")
        self.declare_parameter("base_settle_timeout", 8.0)
        self.declare_parameter("base_settle_samples", 3)
        self.declare_parameter("base_linear_tolerance", 0.005)
        self.declare_parameter("base_angular_tolerance", 0.010)
        self.declare_parameter("clearance_speed", 0.18)
        self.declare_parameter("clearance_timeout", 60.0)
        self.declare_parameter("manipulation_timeout", 600.0)
        self.declare_parameter("low_battery_threshold", 0.25)
        self.declare_parameter("charge_target", 0.80)
        self.declare_parameter("charge_timeout", 300.0)
        self.declare_parameter("machine_done_timeout", 120.0)
        self.declare_parameter("machine_state_max_age", 2.0)


    def _execute_step(self, step: PhysicalStep, goal_handle) -> None:
        if step.kind == StepKind.DOCK:
            self._dock_at(step.station_id, goal_handle)
        elif step.kind == StepKind.UNDOCK:
            self._leave_station(step.station_id, step.stow_arm, goal_handle)
        elif step.kind == StepKind.MANIPULATE:
            self._manipulate_part(step, goal_handle)
        elif step.kind == StepKind.MACHINE_COMMAND:
            self._send_machine_command(step, goal_handle)
        elif step.kind == StepKind.WAIT_MACHINE_DONE:
            self._wait_for_machine_done(
                step.machine_id,
                goal_handle,
                expected_part_id=self._require_physical_part_id(step.part_id),
            )
        elif step.kind == StepKind.COMMIT_TRANSFER:
            self._commit_part_transfer(step, goal_handle)
        else:
            raise PhysicalOrderError(f"unsupported step: {step.kind}")

    def _dock_at(self, station_id: str, goal_handle) -> None:
        station = self._stations[station_id]
        self._dock.select_target(station.tag_id)
        time.sleep(0.25)
        try:
            if uses_direct_machine_alignment(station_id):
                self._dock_at_machine(station_id, goal_handle)
            elif station_id in {"raw_bin", "finished_bin"}:
                self._dock_at_bin(station_id, goal_handle)
            else:
                self._dock_with_nav2(station_id, goal_handle)
        finally:
            self._dock.clear_target()
            time.sleep(0.10)
        self._wait_for_base_settle(goal_handle)
        if station_id == "raw_bin":
            # Docking and bin detection share the mast camera. Let chassis
            # control settle before the manipulation server starts its own
            # fresh three-frame stability window.
            settle_time = float(
                self.get_parameter("raw_perception_settle_time").value
            )
            deadline = time.monotonic() + settle_time
            self.get_logger().info(
                f"Waiting {settle_time:.1f} s for raw-bin RGB-D handoff"
            )
            while time.monotonic() < deadline:
                self._raise_if_cancelled(goal_handle)
                time.sleep(0.05)

    def _dock_at_machine(self, station_id: str, goal_handle) -> None:
        """Navigate once, then hand the final CNC correction to AprilTag."""
        if self._departed_station_id == station_id:
            self.get_logger().info(
                f"Re-entering {station_id} from its measured retreat corridor; "
                "skipping redundant global replanning"
            )
        else:
            self._navigate_to_staging(station_id, goal_handle)
        self._wait_for_base_settle(goal_handle)
        self.get_logger().info(
            f"Reached {station_id} local approach; starting one visual alignment"
        )
        self._dock.align_machine_tag(
            f"{station_id}_tag", timeout_sec=75.0
        )
        self._departed_station_id = None

    def _dock_at_bin(self, station_id: str, goal_handle) -> None:
        """Use Nav2 for transit and vision only for the final bin residual."""
        self._navigate_to_staging(station_id, goal_handle)
        self._wait_for_base_settle(goal_handle)

        self.get_logger().info(
            f"Reached {station_id} staging; refining the bin pose once"
        )
        self._dock.align_bin_pose(
            f"{station_id}_tag", timeout_sec=180.0
        )
        self._departed_station_id = None
        if station_id == "finished_bin" and self._use_finished_slot_perception:
            self._preselect_finished_bin_slot(goal_handle)

    def _preselect_finished_bin_slot(self, goal_handle) -> None:
        """Choose from fresh depth frames captured during final alignment."""

        with self._state_lock:
            reserved_slots = frozenset(self._placed_finished_slots)
        preferred_slots = unreserved_slot_preferences(
            self._finished_slot_order,
            reserved_slots,
        )
        if not preferred_slots:
            raise PhysicalOrderError(
                "no unreserved finished-bin destination remains"
            )
        try:
            slot_id = self._finished_slots.wait_for_empty(
                preferred_slots,
                timeout_sec=self._finished_slot_perception_timeout,
                include_recent_history=True,
            )
        except RuntimeError as error:
            raise PhysicalOrderError(str(error)) from error
        self._raise_if_cancelled(goal_handle)
        with self._state_lock:
            self._pending_finished_slot = slot_id
        self.get_logger().info(
            f"Finished-bin RGB-D selected empty slot {slot_id}; "
            "captured from final visual-alignment history"
        )

    def _navigate_to_staging(self, station_id: str, goal_handle) -> None:
        """Reach one surveyed staging pose and verify the Nav2 result."""
        station = self._stations[station_id]
        nav_handle = self._wait_future(
            self._dock.send_staging_goal(station),
            15.0,
            goal_handle=goal_handle,
        )
        if nav_handle is None or not nav_handle.accepted:
            raise PhysicalOrderError(
                f"navigation rejected staging pose {station_id}"
            )

        wrapped = self._wait_future(
            nav_handle.get_result_async(),
            float(self.get_parameter("staging_timeout").value),
            goal_handle=goal_handle,
            cancel_callback=nav_handle.cancel_goal_async,
        )
        if wrapped is None:
            nav_handle.cancel_goal_async()
            raise PhysicalOrderError(
                f"navigation to staging pose {station_id} timed out"
            )
        result = wrapped.result
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != NavigateToPose.Result.NONE
        ):
            raise PhysicalOrderError(
                f"navigation to staging pose {station_id} failed: "
                f"code={result.error_code} detail={result.error_msg}"
            )

    def _dock_with_nav2(self, station_id: str, goal_handle) -> None:
        """Reserve the Nav2 docking lifecycle for the charging station."""
        goal_future = self._dock.send_goal(
            station_id,
            navigate_to_staging=True,
            staging_timeout=float(
                self.get_parameter("staging_timeout").value
            ),
        )
        dock_handle = self._wait_future(
            goal_future, 15.0, goal_handle=goal_handle
        )
        if dock_handle is None or not dock_handle.accepted:
            raise PhysicalOrderError(
                f"docking server rejected {station_id}"
            )
        wrapped = self._wait_future(
            dock_handle.get_result_async(),
            float(self.get_parameter("docking_timeout").value),
            goal_handle=goal_handle,
            cancel_callback=dock_handle.cancel_goal_async,
        )
        if wrapped is None:
            dock_handle.cancel_goal_async()
            raise PhysicalOrderError(f"docking at {station_id} timed out")
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            raise PhysicalOrderError(
                f"docking at {station_id} failed: "
                f"code={result.error_code} detail={result.error_msg}"
            )

    def _wait_for_base_settle(self, goal_handle) -> None:
        """Require fresh near-zero odometry before planning in ``base_link``."""
        timeout = float(self.get_parameter("base_settle_timeout").value)
        required_samples = int(
            self.get_parameter("base_settle_samples").value
        )
        linear_tolerance = float(
            self.get_parameter("base_linear_tolerance").value
        )
        angular_tolerance = float(
            self.get_parameter("base_angular_tolerance").value
        )
        if timeout <= 0.0 or required_samples <= 0:
            raise PhysicalOrderError(
                "base settle timeout and sample count must be positive"
            )

        deadline = time.monotonic() + timeout
        stable_samples = 0
        last_sample_time = -1.0
        latest_motion: tuple[float, float, float] | None = None
        while time.monotonic() < deadline:
            self._raise_if_cancelled(goal_handle)
            with self._state_lock:
                sample = self._base_motion_sample
            if sample is not None and sample[0] != last_sample_time:
                last_sample_time = sample[0]
                latest_motion = sample[1:]
                if is_planar_motion_settled(
                    *latest_motion,
                    linear_tolerance=linear_tolerance,
                    angular_tolerance=angular_tolerance,
                ):
                    stable_samples += 1
                    if stable_samples >= required_samples:
                        speed = (
                            latest_motion[0] ** 2 + latest_motion[1] ** 2
                        ) ** 0.5
                        self.get_logger().info(
                            "Mobile base settled before arm-frame planning: "
                            f"linear={speed:.4f} m/s, "
                            f"angular={abs(latest_motion[2]):.4f} rad/s"
                        )
                        return
                else:
                    stable_samples = 0
            time.sleep(0.02)

        detail = "no fresh odometry"
        if latest_motion is not None:
            detail = (
                f"vx={latest_motion[0]:.4f}, vy={latest_motion[1]:.4f}, "
                f"wz={latest_motion[2]:.4f}"
            )
        raise PhysicalOrderError(f"mobile base did not settle: {detail}")

    def _leave_station(
        self, station_id: str, stow_arm: bool, goal_handle
    ) -> None:
        dock_type = self._dock_type(station_id)
        timeout = float(self.get_parameter("undocking_timeout").value)
        requested_clearance = float(
            self.get_parameter("clearance_distance").value
        )
        remaining_clearance = requested_clearance
        if (
            station_id == "charge_dock"
            and uses_nav2_undocking(dock_type)
        ):
            undock_handle = self._wait_future(
                self._undock.send_goal(dock_type, timeout),
                15.0,
                goal_handle=goal_handle,
            )
            if undock_handle is None or not undock_handle.accepted:
                raise PhysicalOrderError(
                    f"undocking server rejected {station_id}"
                )
            wrapped = self._wait_future(
                undock_handle.get_result_async(),
                timeout + 10.0,
                goal_handle=goal_handle,
                cancel_callback=undock_handle.cancel_goal_async,
            )
            if wrapped is None:
                undock_handle.cancel_goal_async()
                raise PhysicalOrderError(f"undocking from {station_id} timed out")
            result = wrapped.result
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
                raise PhysicalOrderError(
                    f"undocking from {station_id} failed: "
                    f"code={result.error_code} detail={result.error_msg}"
                )
            remaining_clearance = remaining_clearance_after_nav2_undock(
                dock_type, requested_clearance
            )
            self.get_logger().info(
                f"Nav2 staging for {station_id} leaves "
                f"{remaining_clearance:.3f} m additional clearance"
            )

        travelled = self._clearance.retreat(
            remaining_clearance,
            float(self.get_parameter("clearance_speed").value),
            float(self.get_parameter("clearance_timeout").value),
        )
        self.get_logger().info(
            f"Cleared {station_id} by {travelled:.3f} m extra retreat"
        )
        if stow_arm:
            self._stow_arm(goal_handle)
        self._departed_station_id = station_id

    def _manipulate_part(self, step: PhysicalStep, goal_handle) -> None:
        operation = {
            Manipulation.PICK: ManipulatePart.Goal.PICK,
            Manipulation.PLACE: ManipulatePart.Goal.PLACE,
        }[step.manipulation]
        placement_slot_id = 0
        selects_finished_slot = (
            self._use_finished_slot_perception
            and step.station_id == "finished_bin"
            and step.manipulation == Manipulation.PLACE
        )
        if selects_finished_slot:
            with self._state_lock:
                pending_slot = self._pending_finished_slot
            if pending_slot is None:
                raise PhysicalOrderError(
                    "finished-bin slot was not selected during final alignment"
                )
            placement_slot_id = pending_slot
        goal = ManipulatePart.Goal()
        goal.operation = operation
        goal.station_id = step.station_id
        goal.part_id = step.part_id
        goal.placement_slot_id = placement_slot_id
        child_handle = self._wait_future(
            self._manipulation.send_goal_async(goal),
            15.0,
            goal_handle=goal_handle,
        )
        if child_handle is None or not child_handle.accepted:
            raise PhysicalOrderError(
                f"manipulation rejected at {step.station_id}"
            )
        wrapped = self._wait_future(
            child_handle.get_result_async(),
            float(self.get_parameter("manipulation_timeout").value),
            goal_handle=goal_handle,
            cancel_callback=child_handle.cancel_goal_async,
        )
        if wrapped is None:
            child_handle.cancel_goal_async()
            raise PhysicalOrderError(
                f"manipulation timed out at {step.station_id}"
            )
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            raise PhysicalOrderError(
                f"manipulation failed at {step.station_id}: "
                f"code={result.error_code} detail={result.message}"
            )
        if (
            step.station_id == "raw_bin"
            and step.manipulation == Manipulation.PICK
        ):
            physical_part_id = result.physical_part_id.strip()
            if not physical_part_id:
                raise PhysicalOrderError(
                    "raw-bin grasp succeeded without a tactile physical identity"
                )
            existing = self._part_identity.get(step.part_id)
            if existing is not None and existing != physical_part_id:
                raise PhysicalOrderError(
                    f"logical part {step.part_id} changed identity from "
                    f"{existing} to {physical_part_id}"
                )
            if physical_part_id in self._part_identity.values() and existing is None:
                raise PhysicalOrderError(
                    f"physical part {physical_part_id} is already reserved"
                )
            self._part_identity[step.part_id] = physical_part_id
            self.get_logger().info(
                f"Bound logical {step.part_id} to tactile identity "
                f"{physical_part_id}"
            )
        if selects_finished_slot:
            with self._state_lock:
                self._placed_finished_slots.add(placement_slot_id)
                self._pending_finished_slot = None
            self.get_logger().info(
                "Reserved finished-bin slot "
                f"{placement_slot_id} after physical placement"
            )

    def _send_machine_command(self, step: PhysicalStep, goal_handle) -> None:
        command = {
            MachineOperation.OPEN_DOOR: MachineCommand.Request.OPEN_DOOR,
            MachineOperation.CLOSE_DOOR: MachineCommand.Request.CLOSE_DOOR,
            MachineOperation.START: MachineCommand.Request.START,
            MachineOperation.CONFIRM_LOAD: MachineCommand.Request.CONFIRM_LOAD,
            MachineOperation.CONFIRM_UNLOAD: MachineCommand.Request.CONFIRM_UNLOAD,
        }[step.machine_operation]
        request = MachineCommand.Request()
        request.machine_id = step.machine_id
        request.command = command
        request.part_id = (
            self._require_physical_part_id(step.part_id)
            if step.part_id else ""
        )
        response = self._wait_future(
            self._machine_command_client.call_async(request),
            15.0,
            goal_handle=goal_handle,
        )
        if response is None or not response.accepted:
            detail = "service timeout" if response is None else response.message
            raise PhysicalOrderError(
                f"{step.machine_id} rejected {step.machine_operation.value}: {detail}"
            )

    def _commit_part_transfer(self, step: PhysicalStep, goal_handle) -> None:
        event = {
            TransferEvent.PICK_FROM_RAW: PartTransfer.Request.PICK_FROM_RAW,
            TransferEvent.PLACE_IN_FINISHED: PartTransfer.Request.PLACE_IN_FINISHED,
        }[step.transfer_event]
        request = PartTransfer.Request()
        request.event = event
        request.part_id = self._require_physical_part_id(step.part_id)
        response = self._wait_future(
            self._part_transfer_client.call_async(request),
            15.0,
            goal_handle=goal_handle,
        )
        if response is None or not response.accepted:
            detail = "service timeout" if response is None else response.message
            raise PhysicalOrderError(
                f"inventory rejected {step.transfer_event.value}: {detail}"
            )

    def _require_physical_part_id(self, logical_part_id: str) -> str:
        """Resolve an order ID without exposing simulator identity to BT."""

        physical_part_id = self._part_identity.get(logical_part_id)
        if physical_part_id is None:
            raise PhysicalOrderError(
                f"logical part {logical_part_id} has no tactile identity"
            )
        return physical_part_id

    def _wait_for_machine_done(
        self,
        machine_id: str,
        goal_handle,
        *,
        expected_part_id: str = "",
    ) -> None:
        deadline = time.monotonic() + float(
            self.get_parameter("machine_done_timeout").value
        )
        while time.monotonic() < deadline:
            self._raise_if_cancelled(goal_handle)
            with self._state_lock:
                state = self._machine_states.get(machine_id)
                received = self._machine_state_received_at.get(machine_id)
            max_age = float(self.get_parameter("machine_state_max_age").value)
            if received is None or time.monotonic() - received > max_age:
                state = None
            if state is not None:
                if state.state == MachineState.FAULT:
                    raise PhysicalMachineFault(
                        machine_id,
                        state.part_id,
                        int(state.fault_code),
                    )
                if state.state == MachineState.DONE and state.door_open:
                    if expected_part_id and state.part_id != expected_part_id:
                        raise PhysicalOrderError(
                            f"{machine_id} contains {state.part_id or 'unknown part'}, "
                            f"not expected {expected_part_id}"
                        )
                    return
            time.sleep(0.10)
        raise PhysicalOrderError(
            f"{machine_id} did not reach unload-ready DONE state"
        )

    def _ensure_cycle_energy(
        self,
        goal_handle,
        *,
        auto_recharge: bool,
        completed: int,
        total: int,
    ) -> bool:
        """Recharge only between complete empty-gripper transfer cycles."""
        low_threshold = float(
            self.get_parameter("low_battery_threshold").value
        )
        with self._state_lock:
            battery = self._battery_percentage
        if battery is None:
            raise PhysicalOrderError("physical battery state is unavailable")

        decision = energy_decision(
            battery,
            auto_recharge=auto_recharge,
            low_threshold=low_threshold,
        )
        if decision == EnergyDecision.CONTINUE:
            return False
        if decision == EnergyDecision.BLOCK:
            raise PhysicalOrderError(
                f"battery is {battery * 100.0:.1f}% and automatic "
                "recharge is disabled"
            )

        self._publish_feedback(
            goal_handle,
            phase="recharge",
            machine_id="",
            completed=completed,
            total=total,
            detail=(
                f"battery {battery * 100.0:.1f}% below "
                f"{low_threshold * 100.0:.1f}%; docking to charge"
            ),
        )
        # Energy is evaluated only after the previous part has been placed.
        # Fold the empty arm before crossing the factory to the charger.
        self._stow_arm(goal_handle)
        self._dock_at("charge_dock", goal_handle)
        self._wait_until_charged(goal_handle, completed, total)
        self._leave_station("charge_dock", False, goal_handle)
        with self._state_lock:
            resumed_battery = self._battery_percentage
        self._publish_feedback(
            goal_handle,
            phase="recharge_complete",
            machine_id="",
            completed=completed,
            total=total,
            detail=(
                f"charged to {(resumed_battery or 0.0) * 100.0:.1f}%; "
                "resuming production at the next cycle boundary"
            ),
        )
        return True

    def _wait_until_charged(
        self, goal_handle, completed: int, total: int
    ) -> None:
        target = float(self.get_parameter("charge_target").value)
        timeout = float(self.get_parameter("charge_timeout").value)
        if not 0.0 < target <= 1.0 or timeout <= 0.0:
            raise PhysicalOrderError("invalid physical charging policy")

        deadline = time.monotonic() + timeout
        next_feedback = 0.0
        while time.monotonic() < deadline:
            self._raise_if_cancelled(goal_handle)
            with self._state_lock:
                battery = self._battery_percentage
            if battery is not None and battery >= target:
                self.get_logger().info(
                    f"Physical battery reached {battery * 100.0:.1f}%"
                )
                return
            if time.monotonic() >= next_feedback:
                self._publish_feedback(
                    goal_handle,
                    phase="charging",
                    machine_id="",
                    completed=completed,
                    total=total,
                    detail=(
                        f"charging: {(battery or 0.0) * 100.0:.1f}% / "
                        f"{target * 100.0:.1f}%"
                    ),
                )
                next_feedback = time.monotonic() + 1.0
            time.sleep(0.10)
        raise PhysicalOrderError(
            f"battery did not reach {target * 100.0:.1f}% before timeout"
        )

    def _select_idle_machine(
        self, allowed, goal_handle, completed: int, total: int
    ) -> str:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            self._raise_if_cancelled(goal_handle)
            with self._state_lock:
                states = dict(self._machine_states)
                received = dict(self._machine_state_received_at)
            now = time.monotonic()
            max_age = float(self.get_parameter("machine_state_max_age").value)
            states = {
                machine_id: state
                for machine_id, state in states.items()
                if now - received.get(machine_id, float("-inf")) <= max_age
            }
            skipped_faults: list[str] = []
            for machine_id in allowed:
                state = states.get(machine_id)
                if state is not None and state.state == MachineState.FAULT:
                    skipped_faults.append(machine_id)
                    continue
                if state is not None and state.state == MachineState.IDLE:
                    if skipped_faults:
                        self._publish_feedback(
                            goal_handle,
                            phase="machine_reassigned",
                            machine_id=machine_id,
                            completed=completed,
                            total=total,
                            detail=(
                                f"skipped faulted {', '.join(skipped_faults)}; "
                                f"assigned work to {machine_id}"
                            ),
                        )
                    return machine_id

            observed = [states.get(machine_id) for machine_id in allowed]
            if observed and all(state is not None for state in observed):
                if all(state.state == MachineState.FAULT for state in observed):
                    trapped = [state for state in observed if state.part_id]
                    if trapped:
                        names = ", ".join(state.machine_id for state in trapped)
                        raise PhysicalOrderError(
                            f"all allowed machines faulted; {names} contain "
                            "trapped workpieces requiring manual intervention"
                        )
                    raise PhysicalOrderError(
                        "all allowed machines faulted before loading"
                    )
            self._publish_feedback(
                goal_handle,
                phase="wait_idle_machine",
                machine_id="",
                completed=completed,
                total=total,
                detail="waiting for an allowed IDLE machine",
            )
            time.sleep(0.25)
        raise PhysicalOrderError("no allowed machine became IDLE")

    def _stow_arm(self, goal_handle) -> None:
        last_error = None
        for attempt in range(2):
            arm_handle = self._wait_future(
                self._arm.send_joint_target(STOWED),
                15.0,
                goal_handle=goal_handle,
            )
            if arm_handle is None or not arm_handle.accepted:
                raise PhysicalOrderError("MoveIt rejected post-undock stow")
            wrapped = self._wait_future(
                arm_handle.get_result_async(),
                75.0,
                goal_handle=goal_handle,
                cancel_callback=arm_handle.cancel_goal_async,
            )
            if wrapped is None:
                arm_handle.cancel_goal_async()
                raise PhysicalOrderError("post-undock stow timed out")
            if (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and moveit_succeeded(wrapped.result.error_code)
            ):
                return
            last_error = int(wrapped.result.error_code.val)
            if last_error != MoveItErrorCodes.CONTROL_FAILED:
                break
            time.sleep(0.75)
        raise PhysicalOrderError(
            f"post-undock arm stow failed with code {last_error}"
        )

    def _require_dependencies(self) -> None:
        timeout = float(self.get_parameter("dependency_timeout").value)
        dependencies = (
            (self._dock.wait_until_navigation_ready(timeout),
             "/navigate_to_pose"),
            (self._dock.wait_until_ready(timeout), "/dock_robot"),
            (self._dock.wait_until_active(timeout),
             "/docking_server lifecycle ACTIVE"),
            (self._undock.wait_until_ready(timeout), "/undock_robot"),
            (self._manipulation.wait_for_server(timeout), "/manipulate_part"),
            (self._machine_command_client.wait_for_service(timeout),
             "/factory/machine_command"),
            (self._part_transfer_client.wait_for_service(timeout),
             "/factory/part_transfer"),
            (self._arm.wait_until_ready(timeout), "/move_action"),
        )
        missing = [name for ready, name in dependencies if not ready]
        if missing:
            raise PhysicalOrderError(
                f"physical dependencies unavailable: {', '.join(missing)}"
            )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._state_lock:
                battery_ready = self._battery_percentage is not None
            if battery_ready:
                return
            time.sleep(0.05)
        raise PhysicalOrderError(
            "physical dependency unavailable: /battery_state"
        )

    @staticmethod
    def _dock_type(station_id: str) -> str:
        if station_id.startswith("machine_"):
            return "factory_station"
        if station_id in {"raw_bin", "finished_bin"}:
            return "factory_bin_station"
        if station_id == "charge_dock":
            return "charging_station"
        raise PhysicalOrderError(f"unknown dock type for {station_id}")

    def _wait_future(
        self,
        future,
        timeout_sec: float,
        *,
        goal_handle,
        cancel_callback=None,
    ):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            if self._is_cancel_requested(goal_handle):
                if cancel_callback is not None:
                    cancel_callback()
                raise PhysicalOrderCancelled(
                    "order canceled at a safe physical boundary"
                )
            time.sleep(0.05)
        return None

    def _wait_at_safe_boundary(
        self, goal_handle, completed: int, total: int
    ) -> None:
        next_feedback = 0.0
        while True:
            self._raise_if_cancelled(goal_handle)
            with self._state_lock:
                paused = self._pause_requested
            if not paused:
                return
            if time.monotonic() >= next_feedback:
                self._publish_feedback(
                    goal_handle,
                    phase="paused",
                    machine_id="",
                    completed=completed,
                    total=total,
                    detail="paused at a physical step boundary",
                )
                next_feedback = time.monotonic() + 1.0
            time.sleep(0.10)

    def _raise_if_cancelled(self, goal_handle) -> None:
        if self._is_cancel_requested(goal_handle):
            raise PhysicalOrderCancelled(
                "order canceled at a safe physical boundary"
            )

    def _is_cancel_requested(self, goal_handle) -> bool:
        with self._state_lock:
            external = self._cancel_requested
        return bool(goal_handle.is_cancel_requested or external)

    def _control_task(self, request, response):
        with self._state_lock:
            if not self._active_order_id:
                response.task_state = "idle"
                response.message = "no active physical order"
                return response
            if request.task_id and request.task_id != self._active_order_id:
                response.task_state = "rejected"
                response.message = "task id does not match the active order"
                return response
            if request.command == ControlTask.Request.PAUSE:
                self._pause_requested = True
                response.task_state = "pausing"
            elif request.command == ControlTask.Request.RESUME:
                self._pause_requested = False
                response.task_state = "running"
            elif request.command == ControlTask.Request.CANCEL:
                self._cancel_requested = True
                response.task_state = "canceling"
            else:
                response.task_state = "rejected"
                response.message = "unknown task command"
                return response
            response.accepted = True
            response.message = "accepted at the next safe boundary"
        return response

    def _remember_base_motion(self, message: Odometry) -> None:
        twist = message.twist.twist
        sample = (
            time.monotonic(),
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        )
        with self._state_lock:
            self._base_motion_sample = sample

    def _remember_machine_state(self, message: MachineState) -> None:
        with self._state_lock:
            self._machine_states[message.machine_id] = message
            self._machine_state_received_at[message.machine_id] = time.monotonic()

    def _remember_battery(self, message: BatteryState) -> None:
        if 0.0 <= message.percentage <= 1.0:
            with self._state_lock:
                self._battery_percentage = float(message.percentage)

    def _begin_order(self, order_id: str) -> None:
        with self._state_lock:
            self._active_order_id = order_id
            self._pause_requested = False
            self._cancel_requested = False

    def _publish_feedback(
        self,
        goal_handle,
        *,
        phase: str,
        machine_id: str,
        completed: int,
        total: int,
        detail: str,
    ) -> None:
        feedback = ExecutePhysicalStep.Feedback()
        feedback.phase = phase
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhysicalOrderExecutor()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
