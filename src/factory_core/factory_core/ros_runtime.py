from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from factory_interfaces.action import ExecuteOrder
from factory_interfaces.msg import MachineState as MachineStateMsg
from factory_interfaces.srv import (
    ControlTask,
    GetFactoryState,
    MachineCommand,
    PartTransfer,
)

from .checkpoint import CheckpointStore
from .domain import FactoryState, HeldPartKind, ProductionOrder
from .scheduler import DecisionKind, SimulationEngine
from .fixture_clamp_client import FixtureClampClient


class FactoryRuntime(Node):
    """Semantic factory cell controller.

    It owns machine and order state. The Agent may request high-level changes,
    but every transition is checked here before it reaches a hardware adapter.
    """

    def __init__(self) -> None:
        super().__init__("factory_runtime")
        self._declare_parameters()

        cycle_seconds = float(self.get_parameter("machine_cycle_seconds").value)
        self.state = FactoryState.default(cycle_seconds=cycle_seconds)
        self.state.raw_part_count = int(self.get_parameter("raw_part_count").value)
        self.state.battery = float(self.get_parameter("initial_battery").value)

        checkpoint_path = str(self.get_parameter("checkpoint_path").value)
        self._checkpoint = CheckpointStore(checkpoint_path)
        self._active_order: ProductionOrder | None = None
        self._task_paused = False
        self._external_cancel = False
        self._lock = threading.RLock()
        self._callbacks = ReentrantCallbackGroup()
        self._physical_battery_current: float | None = None
        self._last_autonomous_tick = time.monotonic()
        self._fixture_clamp = FixtureClampClient(
            self, tuple(f"raw_part_{index}" for index in range(1, 7))
        )


        self._machine_publishers = {
            machine_id: self.create_publisher(
                MachineStateMsg, f"/{machine_id}/state", 10
            )
            for machine_id in self.state.machines
        }
        self._battery_publisher = self.create_publisher(
            BatteryState, "/battery_state", 10
        )
        self.create_subscription(
            BatteryState,
            "/factory/physical_battery_state",
            self._on_physical_battery,
            10,
        )
        self.create_timer(0.5, self._publish_state, callback_group=self._callbacks)
        self.create_service(
            GetFactoryState,
            "/factory/get_state",
            self._get_state,
            callback_group=self._callbacks,
        )
        self.create_service(
            MachineCommand,
            "/factory/machine_command",
            self._machine_command,
            callback_group=self._callbacks,
        )
        self.create_service(
            PartTransfer,
            "/factory/part_transfer",
            self._part_transfer,
            callback_group=self._callbacks,
        )
        self._action = None
        if bool(self.get_parameter("enable_order_execution").value):
            self.create_service(
                ControlTask,
                "/factory/control_task",
                self._control_task,
                callback_group=self._callbacks,
            )
            self._action = ActionServer(
                self,
                ExecuteOrder,
                "/factory/execute_order",
                execute_callback=self._execute,
                goal_callback=self._goal,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self._callbacks,
            )
            mode = "semantic order execution enabled"
        else:
            mode = "physical executor owns order execution"
        self.get_logger().info(
            f"Factory semantic runtime ready with three machines; {mode}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("raw_part_count", 6)
        self.declare_parameter("initial_battery", 1.0)
        self.declare_parameter("machine_cycle_seconds", 12.0)
        self.declare_parameter("step_period", 0.15)
        self.declare_parameter("checkpoint_path", "/tmp/factory_robot/checkpoint.json")
        self.declare_parameter("enable_order_execution", True)

    def _goal(self, goal: ExecuteOrder.Goal) -> GoalResponse:
        with self._lock:
            if self._active_order is not None:
                self.get_logger().warning("Rejected order: another order is active")
                return GoalResponse.REJECT
        if goal.quantity < 1:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        order = self._order_from_goal(goal_handle.request)
        result = ExecuteOrder.Result()
        try:
            self._begin_order(order)
            engine = SimulationEngine(self.state, order)
            step_period = float(self.get_parameter("step_period").value)

            while rclpy.ok():
                terminal = self._handle_control_requests(goal_handle, order, result)
                if terminal:
                    return result
                if self._task_paused:
                    # Pause dispatching at an atomic checkpoint, but let an
                    # autonomous CNC finish its already-started cycle.
                    with self._lock:
                        self.state.tick(
                            SimulationEngine.ACTION_SECONDS[DecisionKind.WAIT]
                        )
                        self._checkpoint.save(self.state, order)
                    self._publish_feedback(
                        goal_handle, order, "paused", "", "dispatch paused; machines continue"
                    )
                    time.sleep(step_period)
                    continue

                with self._lock:
                    decision = engine.step()
                    self._checkpoint.save(self.state, order)
                self._publish_feedback(
                    goal_handle,
                    order,
                    decision.kind.value,
                    decision.machine_id,
                    decision.detail,
                )

                if decision.kind == DecisionKind.COMPLETE:
                    goal_handle.succeed()
                    return self._fill_result(result, True, order.completed, 0, "order completed")
                if decision.kind == DecisionKind.BLOCKED:
                    goal_handle.abort()
                    return self._fill_result(result, False, order.completed, 10, decision.detail)
                time.sleep(step_period)

            goal_handle.abort()
            return self._fill_result(result, False, order.completed, 11, "ROS shutdown")
        except ValueError as exc:
            self.get_logger().error(str(exc))
            goal_handle.abort()
            return self._fill_result(result, False, order.completed, 1, str(exc))
        finally:
            self._finish_order()

    @staticmethod
    def _order_from_goal(goal: ExecuteOrder.Goal) -> ProductionOrder:
        return ProductionOrder(
            order_id=goal.order_id,
            quantity=int(goal.quantity),
            allowed_machine_ids=list(goal.allowed_machine_ids),
            auto_recharge=bool(goal.auto_recharge),
        )

    def _begin_order(self, order: ProductionOrder) -> None:
        with self._lock:
            order.allowed_machine_ids = order.allowed_machine_ids or list(self.state.machines)
            order.validate(set(self.state.machines), self.state.raw_part_count)
            self._active_order = order
            self._task_paused = False
            self._external_cancel = False

    def _finish_order(self) -> None:
        with self._lock:
            self._active_order = None
            self._task_paused = False
            self._external_cancel = False

    def _handle_control_requests(self, goal_handle, order, result) -> bool:
        if goal_handle.is_cancel_requested:
            with self._lock:
                self._checkpoint.save(self.state, order)
            goal_handle.canceled()
            self._fill_result(result, False, order.completed, 2, "canceled; checkpoint saved")
            return True

        with self._lock:
            externally_canceled = self._external_cancel
        if externally_canceled:
            with self._lock:
                self._checkpoint.save(self.state, order)
            goal_handle.abort()
            self._fill_result(
                result, False, order.completed, 3, "canceled by operator; checkpoint saved"
            )
            return True
        return False

    @staticmethod
    def _fill_result(result, success, completed, error_code, message):
        result.success = success
        result.completed = completed
        result.error_code = error_code
        result.message = message
        return result

    def _publish_feedback(self, goal_handle, order, phase, machine_id, detail) -> None:
        feedback = ExecuteOrder.Feedback()
        feedback.phase = phase
        feedback.current_machine_id = machine_id
        feedback.completed = order.completed
        feedback.total = order.quantity
        feedback.battery_percentage = float(self.state.battery * 100.0)
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    def _control_task(self, request, response):
        with self._lock:
            if self._active_order is None:
                response.message = "no active task"
                response.task_state = "idle"
                return response
            if request.task_id and request.task_id != self._active_order.order_id:
                response.message = "task id does not match the active task"
                response.task_state = "rejected"
                return response

            if request.command == ControlTask.Request.PAUSE:
                self._task_paused = True
                response.task_state = "paused"
            elif request.command == ControlTask.Request.RESUME:
                self._task_paused = False
                response.task_state = "running"
            elif request.command == ControlTask.Request.CANCEL:
                self._external_cancel = True
                response.task_state = "canceling"
            else:
                response.message = "unknown task command"
                response.task_state = "rejected"
                return response

            response.accepted = True
            response.message = "accepted"
            return response

    def _machine_command(self, request, response):
        with self._lock:
            machine = self.state.machines.get(request.machine_id)
            if machine is None:
                response.error_code = 1
                response.message = "unknown machine"
                return response
            fixture_clamped = False
            try:
                self._validate_machine_inventory_transition(request)
                # A semantic LOAD confirmation is valid only after Gazebo has
                # created the physical vise constraint. Logging a publish
                # request as success allowed a workpiece to drift during the
                # simulated machining cycle.
                if request.command == MachineCommand.Request.CONFIRM_LOAD:
                    self._clamp_workpiece(request.part_id)
                    fixture_clamped = True

                self._apply_machine_command(
                    machine,
                    request.command,
                    request.part_id,
                )
                self._commit_machine_inventory_transition(request)
                response.accepted = True
                response.message = "accepted"
            except ValueError as exc:
                if fixture_clamped:
                    self._fixture_clamp.request_release(request.part_id)
                response.error_code = 2
                response.message = str(exc)
        return response

    def _part_transfer(self, request, response):
        """Commit inventory only after its physical action has succeeded."""
        with self._lock:
            try:
                if request.event == PartTransfer.Request.PICK_FROM_RAW:
                    self._commit_raw_pick(request.part_id)
                elif request.event == PartTransfer.Request.PLACE_IN_FINISHED:
                    self._commit_finished_place(request.part_id)
                else:
                    raise ValueError("unknown part transfer event")
                response.accepted = True
                response.message = "accepted"
            except ValueError as exc:
                response.error_code = 1
                response.message = str(exc)
        return response

    def _commit_raw_pick(self, part_id: str) -> None:
        if not part_id:
            raise ValueError("raw pick requires part_id")
        if self.state.held_part_id:
            raise ValueError(
                f"robot already holds {self.state.held_part_id}"
            )
        if self.state.raw_part_count <= 0:
            raise ValueError("raw inventory is empty")
        self.state.raw_part_count -= 1
        self.state.held_part_id = part_id
        self.state.held_part_kind = HeldPartKind.RAW

    def _commit_finished_place(self, part_id: str) -> None:
        if self.state.held_part_id != part_id:
            raise ValueError(
                f"robot holds {self.state.held_part_id or 'no part'}, not {part_id}"
            )
        if self.state.held_part_kind != HeldPartKind.FINISHED:
            raise ValueError(f"{part_id} has not completed machining")
        self.state.finished_part_count += 1
        self.state.held_part_id = ""
        self.state.held_part_kind = HeldPartKind.NONE

    def _validate_machine_inventory_transition(self, request) -> None:
        if request.command == MachineCommand.Request.CONFIRM_LOAD:
            held = self.state.held_part_id
            if held and held != request.part_id:
                raise ValueError(f"robot holds {held}, not {request.part_id}")
        elif request.command == MachineCommand.Request.CONFIRM_UNLOAD:
            held = self.state.held_part_id
            if held and held != request.part_id:
                raise ValueError(f"robot already holds {held}")

    def _commit_machine_inventory_transition(self, request) -> None:
        if request.command == MachineCommand.Request.CONFIRM_LOAD:
            if self.state.held_part_id == request.part_id:
                self.state.held_part_id = ""
                self.state.held_part_kind = HeldPartKind.NONE
        elif request.command == MachineCommand.Request.CONFIRM_UNLOAD:
            self.state.held_part_id = request.part_id
            self.state.held_part_kind = HeldPartKind.FINISHED

    def _clamp_workpiece(self, part_id: str) -> None:
        """Request the simulated vise and require its state feedback."""
        deadline = time.monotonic() + 5.0
        next_request = 0.0
        while time.monotonic() < deadline:
            if self._fixture_clamp.is_clamped(part_id) is True:
                self.get_logger().info(f"Physical fixture clamped {part_id}")
                return
            if time.monotonic() >= next_request:
                self._fixture_clamp.request_clamp(part_id)
                next_request = time.monotonic() + 0.25
            time.sleep(0.05)
        raise ValueError(f"physical fixture did not clamp {part_id}")

    @staticmethod
    def _apply_machine_command(
        machine,
        command: int,
        part_id: str = "",
    ) -> None:
        if command == MachineCommand.Request.CONFIRM_LOAD:
            if not part_id:
                raise ValueError("confirm load requires part_id")
            machine.load(part_id)
            return

        if command == MachineCommand.Request.CONFIRM_UNLOAD:
            if not part_id:
                raise ValueError("confirm unload requires part_id")
            if machine.part_id != part_id:
                raise ValueError(
                    f"{machine.machine_id}: fixture contains "
                    f"{machine.part_id or 'no part'}, not {part_id}"
                )
            machine.unload()
            return

        operations = {
            MachineCommand.Request.OPEN_DOOR: machine.open_door,
            MachineCommand.Request.CLOSE_DOOR: machine.close_door,
            MachineCommand.Request.START: machine.start,
            MachineCommand.Request.RESET: machine.reset,
            MachineCommand.Request.INJECT_FAULT: lambda: machine.inject_fault(99),
            MachineCommand.Request.HOLD: machine.hold,
            MachineCommand.Request.RESUME: machine.resume,
        }
        operation = operations.get(command)
        if operation is None:
            raise ValueError("unknown command")
        operation()

    def _get_state(self, _request, response):
        with self._lock:
            response.machines = [
                self._machine_message(machine) for machine in self.state.machines.values()
            ]
            response.raw_part_count = self.state.raw_part_count
            response.finished_part_count = self.state.finished_part_count
            response.battery = self._battery_message()
            response.held_part_id = self.state.held_part_id
            response.active_order_id = (
                self._active_order.order_id if self._active_order else ""
            )
        return response

    def _machine_message(self, machine) -> MachineStateMsg:
        message = MachineStateMsg()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = machine.machine_id
        message.machine_id = machine.machine_id
        message.state = int(machine.mode)
        message.door_open = machine.door_open
        message.part_present = bool(machine.part_id)
        message.part_id = machine.part_id
        whole_seconds = int(machine.remaining_seconds)
        message.remaining_time.sec = whole_seconds
        message.remaining_time.nanosec = int(
            (machine.remaining_seconds - whole_seconds) * 1e9
        )
        message.fault_code = machine.fault_code
        return message

    def _on_physical_battery(self, message: BatteryState) -> None:
        if not 0.0 <= message.percentage <= 1.0:
            self.get_logger().warning(
                "Ignoring invalid physical battery percentage"
            )
            return
        with self._lock:
            self.state.battery = float(message.percentage)
            self._physical_battery_current = float(message.current)
            self.state.charging = (
                message.power_supply_status
                == BatteryState.POWER_SUPPLY_STATUS_CHARGING
            )

    def _battery_message(self) -> BatteryState:
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.percentage = float(self.state.battery)
        message.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if self.state.charging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        if self._physical_battery_current is not None:
            message.current = self._physical_battery_current
        elif self.state.charging:
            message.current = 5.0
        else:
            message.current = -1.0
        message.present = True
        return message

    def _publish_state(self) -> None:
        now = time.monotonic()
        elapsed = min(max(0.0, now - self._last_autonomous_tick), 1.0)
        self._last_autonomous_tick = now
        with self._lock:
            # In hardware the PLC advances independently of robot orders. The
            # semantic SimulationEngine already advances machines while its
            # ExecuteOrder action is active, so only idle runtime ownership is
            # needed here for direct physical-cell workflows.
            if self._active_order is None:
                for machine in self.state.machines.values():
                    machine.tick(elapsed)
            for machine_id, machine in self.state.machines.items():
                self._machine_publishers[machine_id].publish(
                    self._machine_message(machine)
                )
            self._battery_publisher.publish(self._battery_message())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FactoryRuntime()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
