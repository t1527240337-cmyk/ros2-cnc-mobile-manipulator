#!/usr/bin/env python3
"""Protocol acceptance for BehaviorTree.CPP task ordering and cleanup."""

from __future__ import annotations

import threading
import time

from action_msgs.msg import GoalStatus
from factory_interfaces.action import ExecuteOrder, ExecutePhysicalStep, ExecuteRobotTask
from factory_interfaces.msg import MachineState
from factory_interfaces.srv import GetFactoryState
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


LOAD_SEQUENCE = [
    ExecutePhysicalStep.Goal.BEGIN_TASK,
    ExecutePhysicalStep.Goal.ENSURE_ENERGY,
    ExecutePhysicalStep.Goal.CHECK_MACHINE_IDLE,
    ExecutePhysicalStep.Goal.DOCK,
    ExecutePhysicalStep.Goal.PICK,
    ExecutePhysicalStep.Goal.COMMIT_PICK_RAW,
    ExecutePhysicalStep.Goal.UNDOCK,
    ExecutePhysicalStep.Goal.OPEN_DOOR,
    ExecutePhysicalStep.Goal.DOCK,
    ExecutePhysicalStep.Goal.PLACE,
    ExecutePhysicalStep.Goal.CONFIRM_LOAD,
    ExecutePhysicalStep.Goal.UNDOCK,
    ExecutePhysicalStep.Goal.CLOSE_DOOR,
    ExecutePhysicalStep.Goal.START_MACHINE,
    ExecutePhysicalStep.Goal.COMPLETE_TASK,
]


UNLOAD_SEQUENCE = [
    ExecutePhysicalStep.Goal.BEGIN_TASK,
    ExecutePhysicalStep.Goal.ENSURE_ENERGY,
    ExecutePhysicalStep.Goal.WAIT_MACHINE_DONE,
    ExecutePhysicalStep.Goal.DOCK,
    ExecutePhysicalStep.Goal.PICK,
    ExecutePhysicalStep.Goal.CONFIRM_UNLOAD,
    ExecutePhysicalStep.Goal.UNDOCK,
    ExecutePhysicalStep.Goal.DOCK,
    ExecutePhysicalStep.Goal.PLACE,
    ExecutePhysicalStep.Goal.COMMIT_PLACE_FINISHED,
    ExecutePhysicalStep.Goal.UNDOCK,
    ExecutePhysicalStep.Goal.COMPLETE_TASK,
]


class FakePhysicalSteps(Node):
    def __init__(self) -> None:
        super().__init__("fake_physical_steps")
        self.callback_group = ReentrantCallbackGroup()
        self.received: list[int] = []
        self.task_ids: list[str] = []
        self.fail_kind: int | None = None
        self.delay_kind: int | None = None
        self.delay_started = threading.Event()
        self.server = ActionServer(
            self,
            ExecutePhysicalStep,
            "/factory/execute_physical_step",
            execute_callback=self.execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )

    def reset(self) -> None:
        self.received.clear()
        self.task_ids.clear()
        self.fail_kind = None
        self.delay_kind = None
        self.delay_started.clear()

    def execute(self, goal_handle) -> ExecutePhysicalStep.Result:
        kind = int(goal_handle.request.step_kind)
        self.received.append(kind)
        self.task_ids.append(goal_handle.request.task_id)
        if kind == self.delay_kind:
            self.delay_started.set()
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self.result(False, ExecutePhysicalStep.Result.CANCELLED)
                time.sleep(0.01)
        if kind == self.fail_kind:
            goal_handle.abort()
            return self.result(False, ExecutePhysicalStep.Result.EXECUTION_FAILED)
        goal_handle.succeed()
        return self.result(True, ExecutePhysicalStep.Result.OK)

    @staticmethod
    def result(success: bool, error_code: int) -> ExecutePhysicalStep.Result:
        result = ExecutePhysicalStep.Result()
        result.success = success
        result.retryable = False
        result.error_code = error_code
        result.message = "fake success" if success else "injected leaf failure"
        return result


class TaskClient(Node):
    def __init__(self) -> None:
        super().__init__("task_bt_protocol_client")
        self.client = ActionClient(
            self, ExecuteRobotTask, "/factory/execute_robot_task"
        )
        self.feedback: list[tuple[str, str]] = []
        self.order_client = ActionClient(
            self, ExecuteOrder, "/factory/execute_order"
        )
        self.order_feedback: list[tuple[str, str, int]] = []
        self.machine_publishers = {
            machine_id: self.create_publisher(
                MachineState, f"/{machine_id}/state", 10
            )
            for machine_id in ("machine_1", "machine_2", "machine_3")
        }
        self.machine_states = {
            machine_id: MachineState.IDLE
            for machine_id in self.machine_publishers
        }
        self.machine_timer = self.create_timer(0.05, self.publish_idle_machines)
        self.raw_part_count = 3
        self.finished_part_count = 0
        self.inventory_service = self.create_service(
            GetFactoryState, "/factory/get_state", self.get_factory_state
        )

    def send(
        self,
        task_id: str,
        task_kind: int = ExecuteRobotTask.Goal.LOAD_RAW,
    ):
        goal = ExecuteRobotTask.Goal()
        goal.task_id = task_id
        goal.task_kind = task_kind
        goal.machine_id = "machine_1"
        goal.part_id = "raw_part_1"
        goal.auto_recharge = True
        self.feedback.clear()
        handle_future = self.client.send_goal_async(
            goal, feedback_callback=self._remember_feedback
        )
        return handle_future

    def _remember_feedback(self, message) -> None:
        feedback = message.feedback
        self.feedback.append((feedback.phase, feedback.detail))


    def publish_idle_machines(self) -> None:
        for machine_id, publisher in self.machine_publishers.items():
            state = MachineState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.machine_id = machine_id
            state.state = self.machine_states[machine_id]
            state.door_open = False
            state.part_present = False
            publisher.publish(state)

    def get_factory_state(self, _request, response):
        response.raw_part_count = self.raw_part_count
        response.finished_part_count = self.finished_part_count
        response.battery.percentage = 1.0
        return response

    def send_order(
        self, order_id: str, quantity: int = 2, allowed: list[str] | None = None
    ):
        goal = ExecuteOrder.Goal()
        goal.order_id = order_id
        goal.quantity = quantity
        goal.allowed_machine_ids = allowed or ["machine_1", "machine_2"]
        goal.auto_recharge = True
        self.order_feedback.clear()
        return self.order_client.send_goal_async(
            goal, feedback_callback=self._remember_order_feedback
        )

    def _remember_order_feedback(self, message) -> None:
        feedback = message.feedback
        self.order_feedback.append(
            (feedback.phase, feedback.current_machine_id, feedback.completed)
        )


def wait_future(future, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.01)
    raise TimeoutError("ROS future timed out")


def main() -> None:
    rclpy.init()
    fake = FakePhysicalSteps()
    client = TaskClient()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(fake)
    executor.add_node(client)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        if not client.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("BehaviorTree task action server is unavailable")
        if not client.order_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("BehaviorTree order action server is unavailable")

        rejected = wait_future(
            client.send_order("over-capacity", quantity=4), 3.0
        )
        assert rejected.accepted
        rejected_result = wait_future(rejected.get_result_async(), 5.0)
        assert rejected_result.status == GoalStatus.STATUS_ABORTED
        assert not rejected_result.result.success
        assert "inventory" in rejected_result.result.message
        assert fake.received == []

        client.raw_part_count = 5
        variable_inventory = wait_future(
            client.send_order("variable-inventory", quantity=4), 3.0
        )
        assert variable_inventory.accepted
        variable_result = wait_future(
            variable_inventory.get_result_async(), 20.0
        )
        assert variable_result.status == GoalStatus.STATUS_SUCCEEDED
        assert variable_result.result.completed == 4
        assert fake.received == (LOAD_SEQUENCE * 2 + UNLOAD_SEQUENCE * 2) * 2
        client.raw_part_count = 3

        fake.reset()
        handle = wait_future(client.send("bt-success"), 3.0)
        wrapped = wait_future(handle.get_result_async(), 8.0)
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED, wrapped
        assert wrapped.result.success
        assert fake.received == LOAD_SEQUENCE, fake.received
        assert ("EnsureEnergy", "fake success") in client.feedback

        fake.reset()
        handle = wait_future(
            client.send("bt-unload", ExecuteRobotTask.Goal.UNLOAD_FINISHED),
            3.0,
        )
        wrapped = wait_future(handle.get_result_async(), 8.0)
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED, wrapped
        assert wrapped.result.success
        assert fake.received == UNLOAD_SEQUENCE, fake.received

        fake.reset()
        fake.fail_kind = ExecutePhysicalStep.Goal.PLACE
        handle = wait_future(client.send("bt-failure"), 3.0)
        wrapped = wait_future(handle.get_result_async(), 8.0)
        assert wrapped.status == GoalStatus.STATUS_ABORTED, wrapped
        assert not wrapped.result.success
        assert fake.received[-1] == ExecutePhysicalStep.Goal.ABORT_TASK
        assert ExecutePhysicalStep.Goal.CONFIRM_LOAD not in fake.received

        fake.reset()
        fake.delay_kind = ExecutePhysicalStep.Goal.DOCK
        handle = wait_future(client.send("bt-cancel"), 3.0)
        assert fake.delay_started.wait(timeout=3.0)
        wait_future(handle.cancel_goal_async(), 3.0)
        wrapped = wait_future(handle.get_result_async(), 8.0)
        assert wrapped.status == GoalStatus.STATUS_CANCELED, wrapped
        assert ExecutePhysicalStep.Goal.ABORT_TASK in fake.received

        fake.reset()
        handle = wait_future(client.send_order("bt-order", quantity=2), 3.0)
        wrapped = wait_future(handle.get_result_async(), 15.0)
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED, wrapped
        assert wrapped.result.success
        assert wrapped.result.completed == 2
        expected = LOAD_SEQUENCE + LOAD_SEQUENCE + UNLOAD_SEQUENCE + UNLOAD_SEQUENCE
        assert fake.received == expected, fake.received
        assert any(item[0] == "OrderInitialized" for item in client.order_feedback)
        assert any(item[0] == "BatchPrepared" for item in client.order_feedback)
        assert sum(item[0] == "PartCompleted" for item in client.order_feedback) == 2
        assert any(":load:" in task_id for task_id in fake.task_ids)
        assert any(":unload:" in task_id for task_id in fake.task_ids)

        fake.reset()
        client.machine_states["machine_1"] = MachineState.FAULT
        time.sleep(0.20)
        handle = wait_future(
            client.send_order(
                "bt-order-partial-capacity",
                quantity=2,
                allowed=["machine_1", "machine_2"],
            ),
            3.0,
        )
        wrapped = wait_future(handle.get_result_async(), 15.0)
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED, wrapped
        assert wrapped.result.completed == 2
        expected = (LOAD_SEQUENCE + UNLOAD_SEQUENCE) * 2
        assert fake.received == expected, fake.received
        assert any(
            item[0] == "BatchCapacityReached"
            for item in client.order_feedback
        )
        client.machine_states["machine_1"] = MachineState.IDLE

        fake.reset()
        fake.fail_kind = ExecutePhysicalStep.Goal.COMMIT_PLACE_FINISHED
        client.machine_states["machine_1"] = MachineState.FAULT
        time.sleep(0.20)
        handle = wait_future(
            client.send_order(
                "bt-order-partial-unload-failure",
                quantity=2,
                allowed=["machine_1", "machine_2"],
            ),
            3.0,
        )
        wrapped = wait_future(handle.get_result_async(), 15.0)
        assert wrapped.status == GoalStatus.STATUS_ABORTED, wrapped
        assert wrapped.result.completed == 0
        assert any(
            ":part:1" in task_id and ":unload:" in task_id
            for task_id in fake.task_ids
        )
        assert not any(
            ":part:2" in task_id and ":load:" in task_id
            for task_id in fake.task_ids
        )
        client.machine_states["machine_1"] = MachineState.IDLE

        fake.reset()
        fake.fail_kind = ExecutePhysicalStep.Goal.PLACE
        handle = wait_future(client.send_order("bt-order-load-failure", quantity=2), 3.0)
        wrapped = wait_future(handle.get_result_async(), 10.0)
        assert wrapped.status == GoalStatus.STATUS_ABORTED, wrapped
        assert not wrapped.result.success
        assert wrapped.result.completed == 0
        assert any(":load:" in task_id for task_id in fake.task_ids)
        assert not any(":unload:" in task_id for task_id in fake.task_ids)
        assert fake.received[-1] == ExecutePhysicalStep.Goal.ABORT_TASK

        fake.reset()
        fake.delay_kind = ExecutePhysicalStep.Goal.DOCK
        handle = wait_future(client.send_order("bt-order-cancel", quantity=1), 3.0)
        assert fake.delay_started.wait(timeout=3.0)
        wait_future(handle.cancel_goal_async(), 3.0)
        wrapped = wait_future(handle.get_result_async(), 10.0)
        assert wrapped.status == GoalStatus.STATUS_CANCELED, wrapped
        assert not wrapped.result.success
        assert ExecutePhysicalStep.Goal.ABORT_TASK in fake.received

        print(
            "task_bt_protocol_ok load_order unload_order failure_abort "
            "cancel_abort live_inventory_rejection variable_inventory "
            "order_tree_batching partial_machine_capacity stop_after_unload_failure "
            "order_failure order_cancel"
        )
    finally:
        executor.shutdown()
        fake.destroy_node()
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
