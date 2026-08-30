from __future__ import annotations

import json
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from factory_interfaces.action import ExecuteOrder
from factory_interfaces.msg import MachineState
from factory_interfaces.srv import (
    ControlProduction,
    ControlTask,
    ExecuteAgentCommand,
    GetFactoryState,
    MachineCommand,
    SubmitNaturalLanguage,
)

from .audit import CommandAuditLog
from .command_interpreter import AgentCommandInterpreter
from .command_models import (
    AGENT_TOOL_DESCRIPTIONS,
    AgentCommand,
    AgentOperation,
)
from .operator_state import OperatorState, capability_summary


MACHINE_STATE_NAMES = {
    MachineState.IDLE: "IDLE",
    MachineState.READY: "READY",
    MachineState.PROCESSING: "PROCESSING",
    MachineState.DONE: "DONE",
    MachineState.FAULT: "FAULT",
    MachineState.HELD: "HELD",
}

BATTERY_STATUS_NAMES = {
    BatteryState.POWER_SUPPLY_STATUS_UNKNOWN: "UNKNOWN",
    BatteryState.POWER_SUPPLY_STATUS_CHARGING: "CHARGING",
    BatteryState.POWER_SUPPLY_STATUS_DISCHARGING: "DISCHARGING",
    BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING: "NOT_CHARGING",
    BatteryState.POWER_SUPPLY_STATUS_FULL: "FULL",
}


def enum_name(mapping: dict[int, str], value: int) -> str:
    """Expose ROS integer enums to language models without ambiguity."""
    numeric_value = int(value)
    return mapping.get(numeric_value, f"UNRECOGNIZED_{numeric_value}")


class FactoryAgentNode(Node):
    """Converts natural language into allow-listed, high-level ROS requests."""

    def __init__(self) -> None:
        super().__init__("factory_agent")
        self._group = ReentrantCallbackGroup()
        self._interpreter = AgentCommandInterpreter()
        self._state = OperatorState()
        self._audit_log = CommandAuditLog()
        self._order_client = ActionClient(
            self, ExecuteOrder, "/factory/execute_order", callback_group=self._group
        )
        self._state_client = self.create_client(
            GetFactoryState, "/factory/get_state", callback_group=self._group
        )
        self._task_client = self.create_client(
            ControlTask, "/factory/control_task", callback_group=self._group
        )
        self._production_client = self.create_client(
            ControlProduction,
            "/factory/control_production",
            callback_group=self._group,
        )
        self._machine_client = self.create_client(
            MachineCommand, "/factory/machine_command", callback_group=self._group
        )
        self.create_service(
            SubmitNaturalLanguage,
            "/factory_agent/submit",
            self._submit,
            callback_group=self._group,
        )
        self.create_service(
            ExecuteAgentCommand,
            "/factory_agent/command",
            self._structured_command,
            callback_group=self._group,
        )
        self.create_timer(0.5, self._refresh_state, callback_group=self._group)
        self.get_logger().info(
            "Factory agent ready: intent/RAG only; low-level control is blocked"
        )

    def _submit(self, request, response):
        request_id = f"request-{uuid.uuid4().hex[:12]}"
        try:
            interpreted = self._interpreter.interpret(request.text)
        except ValueError as exc:
            result = self._reject(response, "rules", str(exc))
            self._audit_response(
                result, request_id, "natural_language", "parse"
            )
            return result

        result = self._dispatch(
            interpreted.command,
            response,
            parser=interpreted.parser,
            fallback_reason=interpreted.fallback_reason,
            explanation_query=request.text,
        )
        self._audit_response(
            result,
            request_id,
            interpreted.parser,
            interpreted.command.operation.value,
        )
        return result

    def _structured_command(self, request, response):
        response.request_id = f"request-{uuid.uuid4().hex[:12]}"
        source = request.source.strip().replace("\n", " ")[:40]
        response.parser = source or "structured"
        response.operation = request.operation
        try:
            command = AgentCommand(
                operation=request.operation,
                quantity=request.quantity or None,
                allowed_machine_ids=list(request.allowed_machine_ids),
                target_machine_id=request.target_machine_id or None,
                task_id=request.task_id or None,
                auto_recharge=request.auto_recharge,
                query=request.query or None,
            )
        except ValueError as exc:
            result = self._reject(response, response.parser, str(exc))
            self._audit_response(
                result,
                response.request_id,
                response.parser,
                request.operation,
            )
            return result

        response.operation = command.operation.value
        result = self._dispatch(
            command,
            response,
            parser=response.parser,
            explanation_query=command.query or "",
        )
        self._audit_response(
            result,
            response.request_id,
            response.parser,
            command.operation.value,
        )
        return result

    def _dispatch(
        self,
        command,
        response,
        *,
        parser: str,
        fallback_reason: str = "",
        explanation_query: str = "",
    ):
        response.parser = parser
        operation = command.operation
        if operation == AgentOperation.SUBMIT_ORDER:
            return self._submit_order(
                command,
                response,
                parser=parser,
                fallback_reason=fallback_reason,
            )
        if operation == AgentOperation.GET_FACTORY_STATE:
            return self._get_factory_state(response)
        if operation == AgentOperation.GET_TASK_STATUS:
            return self._get_task_status(command, response)
        if operation == AgentOperation.LIST_CAPABILITIES:
            return self._accept(
                response,
                capability_summary(),
                data=self._capability_payload(),
            )
        if operation == AgentOperation.EXPLAIN_FAILURE:
            explanation = self._interpreter.knowledge.explain(
                explanation_query
            )
            return self._accept(
                response,
                explanation,
                data={"query": explanation_query, "explanation": explanation},
            )
        if operation in {
            AgentOperation.START_AUTOMATIC,
            AgentOperation.STOP_AUTOMATIC,
            AgentOperation.GET_AUTOMATIC_STATUS,
        }:
            return self._control_production(command, response)
        if operation in {
            AgentOperation.PAUSE_TASK,
            AgentOperation.RESUME_TASK,
            AgentOperation.CANCEL_TASK,
        }:
            return self._control_task(command, response)
        return self._control_machine(command, response)

    def _submit_order(
        self,
        command,
        response,
        *,
        parser: str,
        fallback_reason: str,
    ):
        if not self._order_client.wait_for_server(timeout_sec=1.0):
            return self._reject(response, parser, "订单执行器不可用")
        order_id = f"agent-{uuid.uuid4().hex[:10]}"
        goal = ExecuteOrder.Goal()
        goal.order_id = order_id
        goal.quantity = command.quantity
        goal.allowed_machine_ids = command.allowed_machine_ids
        goal.auto_recharge = command.auto_recharge
        future = self._order_client.send_goal_async(
            goal, feedback_callback=self._feedback
        )
        if not self._wait_for_future(future, timeout_sec=2.0):
            return self._reject(response, parser, "订单执行器响应超时")
        try:
            goal_handle = future.result()
        except Exception as exc:
            return self._reject(
                response, parser, f"订单提交失败：{exc}"
            )
        if not goal_handle.accepted:
            return self._reject(response, parser, "订单被确定性执行器拒绝")

        self._state.start_order(order_id)
        goal_handle.get_result_async().add_done_callback(self._order_result)
        response.order_id = order_id
        suffix = (
            f"；已降级到规则解析：{fallback_reason}"
            if fallback_reason
            else ""
        )
        return self._accept(
            response,
            f"生产订单已提交{suffix}",
            data={"order_id": order_id, "accepted_by_executor": True},
        )

    def _control_production(self, command, response):
        if not self._production_client.wait_for_service(timeout_sec=1.0):
            return self._reject(
                response, response.parser, "自动生产协调器不可用"
            )
        request = ControlProduction.Request()
        request.command = {
            AgentOperation.START_AUTOMATIC: ControlProduction.Request.START,
            AgentOperation.STOP_AUTOMATIC:
                ControlProduction.Request.STOP_AFTER_CURRENT,
            AgentOperation.GET_AUTOMATIC_STATUS:
                ControlProduction.Request.QUERY,
        }[command.operation]
        request.allowed_machine_ids = command.allowed_machine_ids

        future = self._production_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=2.0):
            return self._reject(
                response, response.parser, "自动生产协调器响应超时"
            )
        try:
            result = future.result()
        except Exception as exc:
            return self._reject(
                response, response.parser, f"自动生产请求失败：{exc}"
            )

        summary = (
            f"自动生产={'开启' if result.enabled else '关闭'}；"
            f"状态={result.state}；"
            f"当前订单={result.active_order_id or '无'}；"
            f"累计完成={result.completed_parts}件；{result.message}"
        )
        if not result.accepted:
            return self._reject(response, response.parser, summary)
        return self._accept(
            response,
            summary,
            data={
                "enabled": result.enabled,
                "stop_after_current": result.stop_after_current,
                "state": result.state,
                "active_order_id": result.active_order_id,
                "completed_parts": result.completed_parts,
                "failed_orders": result.failed_orders,
            },
        )

    def _get_factory_state(self, response):
        if not self._state_client.wait_for_service(timeout_sec=1.0):
            return self._reject(
                response, response.parser, "工厂状态服务不可用"
            )
        future = self._state_client.call_async(GetFactoryState.Request())
        if not self._wait_for_future(future, timeout_sec=2.0):
            return self._reject(
                response, response.parser, "工厂状态查询超时"
            )
        try:
            result = future.result()
            self._state.update_factory(result)
        except Exception as exc:
            return self._reject(
                response, response.parser, f"工厂状态查询失败：{exc}"
            )
        return self._accept(
            response, self._state.factory_summary(), data=self._factory_payload(result)
        )

    def _get_task_status(self, command, response):
        requested_task_id = command.task_id
        tracked_task_id = self._state.tracked_order_id
        if requested_task_id and requested_task_id != tracked_task_id:
            return self._reject(
                response,
                response.parser,
                f"Agent 未跟踪任务：{requested_task_id}",
            )
        return self._accept(
            response,
            self._state.task_summary(),
            data=self._task_payload(),
        )

    def _control_task(self, command, response):
        if not self._task_client.wait_for_service(timeout_sec=1.0):
            return self._reject(response, response.parser, "任务控制服务不可用")
        request = ControlTask.Request()
        request.task_id = command.task_id or self._state.active_order_id
        request.command = {
            AgentOperation.PAUSE_TASK: ControlTask.Request.PAUSE,
            AgentOperation.RESUME_TASK: ControlTask.Request.RESUME,
            AgentOperation.CANCEL_TASK: ControlTask.Request.CANCEL,
        }[command.operation]
        future = self._task_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=2.0):
            return self._reject(response, response.parser, "任务控制响应超时")
        try:
            result = future.result()
        except Exception as exc:
            return self._reject(
                response, response.parser, f"任务控制失败：{exc}"
            )
        summary = (
            f"任务={request.task_id or '当前任务'}；"
            f"状态={result.task_state}；{result.message}"
        )
        if not result.accepted:
            return self._reject(response, response.parser, summary)
        self._state.phase = result.task_state
        self._state.detail = result.message
        return self._accept(
            response,
            summary,
            data={
                "task_id": request.task_id,
                "task_state": result.task_state,
                "operation": command.operation.value,
            },
        )

    def _control_machine(self, command, response):
        if not self._machine_client.wait_for_service(timeout_sec=1.0):
            return self._reject(response, response.parser, "机床控制服务不可用")
        request = MachineCommand.Request()
        request.machine_id = command.target_machine_id
        request.command = (
            MachineCommand.Request.HOLD
            if command.operation == AgentOperation.HOLD_MACHINE
            else MachineCommand.Request.RESUME
        )
        future = self._machine_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=2.0):
            return self._reject(response, response.parser, "机床控制响应超时")
        try:
            result = future.result()
        except Exception as exc:
            return self._reject(
                response, response.parser, f"机床控制失败：{exc}"
        )
        summary = (
            f"机床={command.target_machine_id}；"
            f"操作={command.operation.value}；{result.message}"
        )
        if not result.accepted:
            return self._reject(response, response.parser, summary)
        return self._accept(
            response,
            summary,
            data={
                "machine_id": command.target_machine_id,
                "operation": command.operation.value,
                "accepted": True,
            },
        )

    @staticmethod
    def _wait_for_future(future, *, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def _audit_response(
        self,
        response,
        request_id: str,
        source: str,
        operation: str,
    ) -> None:
        try:
            self._audit_log.record(
                request_id=request_id,
                source=source,
                operation=operation,
                accepted=response.accepted,
                order_id=response.order_id,
                message=response.message,
            )
        except OSError as exc:
            self.get_logger().warning(
                f"could not write Agent audit log: {exc}"
            )

    def _refresh_state(self) -> None:
        if not self._state_client.service_is_ready():
            return
        self._state_client.call_async(GetFactoryState.Request()).add_done_callback(
            self._store_state
        )

    def _store_state(self, future) -> None:
        try:
            self._state.update_factory(future.result())
        except Exception as exc:  # ROS transport failures are reported, not hidden.
            self.get_logger().warning(f"state refresh failed: {exc}")

    def _feedback(self, message) -> None:
        feedback = message.feedback
        self._state.phase = feedback.phase
        self._state.progress = f"{feedback.completed}/{feedback.total}"
        self._state.detail = feedback.detail

    def _order_result(self, future) -> None:
        result = future.result().result
        self._state.finish_order(
            success=result.success,
            message=result.message,
        )

    @staticmethod
    def _factory_payload(result) -> dict:
        return {
            "machines": [
                {
                    "machine_id": machine.machine_id,
                    "state": machine.state,
                    "state_name": enum_name(MACHINE_STATE_NAMES, machine.state),
                    "door_open": machine.door_open,
                    "part_present": machine.part_present,
                    "part_id": machine.part_id,
                    "remaining_seconds": (
                        machine.remaining_time.sec
                        + machine.remaining_time.nanosec / 1_000_000_000.0
                    ),
                    "fault_code": machine.fault_code,
                }
                for machine in result.machines
            ],
            "inventory": {
                "raw_parts": result.raw_part_count,
                "finished_parts": result.finished_part_count,
                "held_part_id": result.held_part_id,
            },
            "battery": {
                "percentage": result.battery.percentage,
                "voltage": result.battery.voltage,
                "current": result.battery.current,
                "power_supply_status": result.battery.power_supply_status,
                "power_supply_status_name": enum_name(
                    BATTERY_STATUS_NAMES,
                    result.battery.power_supply_status,
                ),
            },
            "active_order_id": result.active_order_id,
        }

    def _task_payload(self) -> dict:
        return {
            "order_id": self._state.tracked_order_id,
            "phase": self._state.phase,
            "progress": self._state.progress,
            "detail": self._state.detail,
        }

    @staticmethod
    def _capability_payload() -> dict:
        return {
            "tools": [
                {
                    "operation": operation.value,
                    "description": description,
                }
                for operation, description in AGENT_TOOL_DESCRIPTIONS.items()
            ]
        }

    @staticmethod
    def _accept(response, message, *, data=None):
        response.accepted = True
        response.message = message
        if hasattr(response, "data_json"):
            response.data_json = json.dumps(
                data if data is not None else {},
                ensure_ascii=False,
                separators=(",", ":"),
            )
        return response

    @staticmethod
    def _reject(response, parser, message):
        response.accepted = False
        response.parser = parser
        response.message = message
        if hasattr(response, "data_json"):
            response.data_json = "{}"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FactoryAgentNode()
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
