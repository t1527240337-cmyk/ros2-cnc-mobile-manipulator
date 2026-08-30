"""ROS 2 coordinator that turns available capacity into bounded orders."""

from __future__ import annotations

import math
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from factory_interfaces.action import ExecuteOrder
from factory_interfaces.msg import MachineState
from factory_interfaces.srv import ControlProduction, GetFactoryState
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .automatic_production import (
    FactoryAvailability,
    choose_automatic_order,
)


class AutomaticProductionCoordinator(Node):
    """Dispatch production orders while automatic mode is enabled.

    The coordinator owns no robot motion. It reads the semantic factory
    snapshot, creates one bounded ExecuteOrder goal, and waits for that goal
    to finish before considering another batch.
    """

    _KNOWN_MACHINES = ("machine_1", "machine_2", "machine_3")

    def __init__(self) -> None:
        super().__init__("automatic_production_coordinator")
        self._declare_parameters()
        self._callbacks = ReentrantCallbackGroup()
        self._lock = threading.RLock()

        self._enabled = bool(self.get_parameter("enabled_on_start").value)
        self._stop_after_current = False
        self._state = "waiting" if self._enabled else "stopped"
        self._message = (
            "automatic production enabled"
            if self._enabled
            else "automatic production is disabled"
        )
        self._allowed_machine_ids = self._configured_machines()
        self._active_order_id = ""
        self._goal_pending = False
        self._state_request_pending = False
        self._retry_not_before = 0.0
        self._completed_orders = 0
        self._completed_parts = 0
        self._failed_orders = 0

        self._factory_state = self.create_client(
            GetFactoryState,
            "/factory/get_state",
            callback_group=self._callbacks,
        )
        self._orders = ActionClient(
            self,
            ExecuteOrder,
            "/factory/execute_order",
            callback_group=self._callbacks,
        )
        self.create_service(
            ControlProduction,
            "/factory/control_production",
            self._control_production,
            callback_group=self._callbacks,
        )
        period = float(self.get_parameter("dispatch_period").value)
        self.create_timer(period, self._tick, callback_group=self._callbacks)
        self.get_logger().info(
            f"Automatic production coordinator ready; state={self._state}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled_on_start", False)
        self.declare_parameter("dispatch_period", 0.5)
        self.declare_parameter("retry_delay", 2.0)
        self.declare_parameter("maximum_batch_size", 3)
        self.declare_parameter("minimum_dispatch_battery", 0.25)
        self.declare_parameter(
            "default_machine_order",
            ["machine_2", "machine_1", "machine_3"],
        )

    def _configured_machines(self) -> tuple[str, ...]:
        configured = tuple(
            str(value)
            for value in self.get_parameter("default_machine_order").value
        )
        machines = tuple(dict.fromkeys(configured))
        invalid = set(machines) - set(self._KNOWN_MACHINES)
        if invalid:
            raise ValueError(
                f"unknown machines in default_machine_order: {sorted(invalid)}"
            )
        return machines

    def _tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            if not self._enabled:
                return
            if (
                self._active_order_id
                or self._goal_pending
                or self._state_request_pending
                or now < self._retry_not_before
            ):
                return

        if not self._factory_state.service_is_ready():
            self._set_waiting(
                "waiting_dependencies", "factory state service is unavailable"
            )
            return
        if not self._orders.server_is_ready():
            self._set_waiting(
                "waiting_dependencies", "order action server is unavailable"
            )
            return

        with self._lock:
            self._state_request_pending = True
        future = self._factory_state.call_async(GetFactoryState.Request())
        future.add_done_callback(self._consider_factory_state)

    def _consider_factory_state(self, future) -> None:
        with self._lock:
            self._state_request_pending = False
            if not self._enabled or self._active_order_id or self._goal_pending:
                return
            allowed = self._allowed_machine_ids

        try:
            response = future.result()
            battery = float(response.battery.percentage)
            if not math.isfinite(battery):
                raise ValueError("factory reported a non-finite battery value")
            availability = FactoryAvailability(
                raw_part_count=int(response.raw_part_count),
                battery_percentage=battery,
                idle_machine_ids=tuple(
                    machine.machine_id
                    for machine in response.machines
                    if machine.state == MachineState.IDLE
                ),
                active_order_id=str(response.active_order_id),
            )
            decision = choose_automatic_order(
                availability,
                allowed_machine_ids=allowed,
                max_batch_size=int(
                    self.get_parameter("maximum_batch_size").value
                ),
                minimum_battery=float(
                    self.get_parameter("minimum_dispatch_battery").value
                ),
            )
        except Exception as exc:
            self._retry(
                "waiting_state",
                f"cannot evaluate factory state: {exc}",
            )
            return

        if not decision.should_dispatch:
            self._set_waiting(decision.state, decision.reason)
            return
        self._dispatch(decision.quantity, decision.allowed_machine_ids)

    def _dispatch(
        self, quantity: int, allowed_machine_ids: tuple[str, ...]
    ) -> None:
        order_id = f"auto-{uuid.uuid4().hex[:10]}"
        goal = ExecuteOrder.Goal()
        goal.order_id = order_id
        goal.quantity = quantity
        goal.allowed_machine_ids = list(allowed_machine_ids)
        goal.auto_recharge = True

        with self._lock:
            if not self._enabled:
                return
            self._goal_pending = True
            self._active_order_id = order_id
            self._state = "dispatching"
            self._message = (
                f"dispatching {quantity} part(s) to "
                f"{', '.join(allowed_machine_ids)}"
            )
        self.get_logger().info(f"{order_id}: {self._message}")
        self._orders.send_goal_async(goal).add_done_callback(
            self._goal_response
        )

    def _goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._dispatch_rejected(f"order request failed: {exc}")
            return
        if not goal_handle.accepted:
            self._dispatch_rejected("robot is busy; automatic order will retry")
            return

        with self._lock:
            self._goal_pending = False
            self._state = (
                "draining" if self._stop_after_current else "running"
            )
            self._message = "automatic order is executing"
        goal_handle.get_result_async().add_done_callback(self._order_result)

    def _dispatch_rejected(self, message: str) -> None:
        with self._lock:
            self._goal_pending = False
            self._active_order_id = ""
            if self._stop_after_current:
                self._stop_after_current = False
                self._state = "stopped"
                self._message = (
                    "automatic production stopped before an order started"
                )
                return
        self._retry("waiting_robot", message)

    def _order_result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            succeeded = (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and bool(result.success)
            )
            result_message = str(result.message)
            completed = int(result.completed)
        except Exception as exc:
            succeeded = False
            result_message = f"cannot read order result: {exc}"
            completed = 0

        with self._lock:
            order_id = self._active_order_id
            self._active_order_id = ""
            self._goal_pending = False
            if succeeded:
                self._completed_orders += 1
                self._completed_parts += completed
                if self._stop_after_current:
                    self._enabled = False
                    self._stop_after_current = False
                    self._state = "stopped"
                    self._message = "current order completed; automatic mode stopped"
                else:
                    self._state = "waiting"
                    self._message = "order completed; checking the next batch"
            else:
                self._failed_orders += 1
                self._enabled = False
                self._stop_after_current = False
                self._state = "faulted"
                self._message = (
                    f"automatic mode stopped after order failure: "
                    f"{result_message}"
                )

        log = self.get_logger().info if succeeded else self.get_logger().error
        log(f"{order_id}: {self._message}")

    def _control_production(self, request, response):
        if request.command == ControlProduction.Request.QUERY:
            return self._fill_response(response, accepted=True)
        if request.command == ControlProduction.Request.START:
            try:
                machines = self._validate_requested_machines(
                    request.allowed_machine_ids
                )
            except ValueError as exc:
                return self._fill_response(
                    response, accepted=False, message=str(exc)
                )
            with self._lock:
                self._allowed_machine_ids = machines
                self._enabled = True
                self._stop_after_current = False
                if not self._active_order_id:
                    self._state = "waiting"
                    self._message = "automatic production enabled"
            return self._fill_response(response, accepted=True)
        if request.command == ControlProduction.Request.STOP_AFTER_CURRENT:
            with self._lock:
                self._enabled = False
                if self._active_order_id or self._goal_pending:
                    self._stop_after_current = True
                    self._state = "draining"
                    self._message = (
                        "no new order will start; current order will finish"
                    )
                else:
                    self._stop_after_current = False
                    self._state = "stopped"
                    self._message = "automatic production stopped"
            return self._fill_response(response, accepted=True)
        return self._fill_response(
            response, accepted=False, message="unknown production command"
        )

    def _validate_requested_machines(self, requested) -> tuple[str, ...]:
        machines = tuple(dict.fromkeys(str(value) for value in requested))
        if not machines:
            return self._configured_machines()
        invalid = set(machines) - set(self._KNOWN_MACHINES)
        if invalid:
            raise ValueError(f"unknown machines: {sorted(invalid)}")
        return machines

    def _fill_response(self, response, *, accepted: bool, message: str = ""):
        with self._lock:
            response.accepted = accepted
            response.enabled = self._enabled
            response.stop_after_current = self._stop_after_current
            response.state = self._state
            response.active_order_id = self._active_order_id
            response.completed_orders = self._completed_orders
            response.completed_parts = self._completed_parts
            response.failed_orders = self._failed_orders
            response.message = message or self._message
        return response

    def _set_waiting(self, state: str, message: str) -> None:
        with self._lock:
            if self._enabled and not self._active_order_id:
                self._state = state
                self._message = message

    def _retry(self, state: str, message: str) -> None:
        with self._lock:
            self._retry_not_before = time.monotonic() + float(
                self.get_parameter("retry_delay").value
            )
        self._set_waiting(state, message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutomaticProductionCoordinator()
    executor = MultiThreadedExecutor(num_threads=4)
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
