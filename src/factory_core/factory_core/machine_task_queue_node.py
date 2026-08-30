from __future__ import annotations

import threading
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from factory_interfaces.action import ExecuteRobotTask
from factory_interfaces.msg import MachineState as MachineStateMsg
from factory_interfaces.msg import RobotTaskState as RobotTaskStateMsg
from factory_interfaces.srv import GetRobotTaskQueue
from factory_interfaces.srv import ReconcileRobotTask

from .machine_event_adapter import MachineEventAdapter
from .machine_task_runtime_store import (
    MachineTaskRuntimeState,
    MachineTaskRuntimeStore,
)
from .robot_task_request import (
    PhysicalRobotTaskRequest,
    physical_request_from_task,
)
from .state_reconciler import MachineStateReconciler
from .task_models import (
    InterruptedTaskResolution,
    ReconciliationContext,
    RobotTask,
    RobotTaskKind,
    RobotTaskStatus,
    TERMINAL_TASK_STATUSES,
)
from .task_queue import RobotTaskQueue


_KIND_TO_MSG = {
    RobotTaskKind.UNLOAD_FINISHED: RobotTaskStateMsg.KIND_UNLOAD_FINISHED,
    RobotTaskKind.LOAD_RAW: RobotTaskStateMsg.KIND_LOAD_RAW,
    RobotTaskKind.DOCK_AND_CHARGE: RobotTaskStateMsg.KIND_DOCK_AND_CHARGE,
}
_STATUS_TO_MSG = {
    RobotTaskStatus.PENDING: RobotTaskStateMsg.STATUS_PENDING,
    RobotTaskStatus.RESERVED: RobotTaskStateMsg.STATUS_RESERVED,
    RobotTaskStatus.RUNNING: RobotTaskStateMsg.STATUS_RUNNING,
    RobotTaskStatus.SUCCEEDED: RobotTaskStateMsg.STATUS_SUCCEEDED,
    RobotTaskStatus.FAILED: RobotTaskStateMsg.STATUS_FAILED,
    RobotTaskStatus.CANCELED: RobotTaskStateMsg.STATUS_CANCELED,
}
_RESOLUTION_FROM_MSG = {
    ReconcileRobotTask.Request.RETRY: InterruptedTaskResolution.RETRY,
    ReconcileRobotTask.Request.MARK_SUCCEEDED: (
        InterruptedTaskResolution.MARK_SUCCEEDED
    ),
    ReconcileRobotTask.Request.CANCEL: InterruptedTaskResolution.CANCEL,
}


class MachineTaskQueueNode(Node):
    """Persist idempotent robot work derived from authoritative machine state."""

    def __init__(self) -> None:
        super().__init__("machine_task_queue")
        self._declare_parameters()
        self._lock = threading.RLock()

        queue_path = str(self.get_parameter("queue_path").value)
        self._store = MachineTaskRuntimeStore(queue_path)
        self._runtime = self._load_or_create_runtime()
        self._reconciler = MachineStateReconciler(self._runtime.queue)
        order_id = str(self.get_parameter("order_id").value)
        planned_quantity = int(self.get_parameter("planned_quantity").value)
        if planned_quantity < 1:
            raise ValueError("planned_quantity must be positive")
        self._reconciliation_context = ReconciliationContext(
            order_id=order_id,
            production_part_ids=tuple(
                f"{order_id}:part:{index}"
                for index in range(1, planned_quantity + 1)
            ),
            allow_loading=bool(self.get_parameter("allow_loading").value),
        )
        self._dispatch_enabled = bool(
            self.get_parameter("dispatch_enabled").value
        )
        self._auto_recharge = bool(self.get_parameter("auto_recharge").value)
        self._maximum_attempts = int(
            self.get_parameter("maximum_attempts").value
        )
        self._active_task_id = ""
        self._dispatch_halted = False
        self._task_client = ActionClient(
            self,
            ExecuteRobotTask,
            "/factory/execute_robot_task",
        )

        if self._dispatch_enabled:
            recovered = self._runtime.queue.recover_interrupted_tasks()
            if recovered:
                self._store.save(self._runtime)
                self.get_logger().warning(
                    "Recovered interrupted task states: "
                    + ", ".join(
                        f"{task_id}={status.value}"
                        for task_id, status in recovered.items()
                    )
                )
            self._dispatch_halted = any(
                task.status == RobotTaskStatus.FAILED
                for task in self._runtime.queue.tasks()
            )
            if self._dispatch_halted:
                self.get_logger().error(
                    "Dispatch halted: persisted task failure requires reconciliation"
                )

        machine_ids = tuple(self.get_parameter("machine_ids").value)
        self._subscriptions = [
            self.create_subscription(
                MachineStateMsg,
                f"/{machine_id}/state",
                self._on_machine_state,
                10,
            )
            for machine_id in machine_ids
        ]
        self._query_service = self.create_service(
            GetRobotTaskQueue,
            "/factory/get_robot_task_queue",
            self._get_queue,
        )
        self._reconcile_service = self.create_service(
            ReconcileRobotTask,
            "/factory/reconcile_robot_task",
            self._reconcile_task,
        )
        self._dispatch_timer = self.create_timer(
            float(self.get_parameter("dispatch_period").value),
            self._dispatch_next,
            # Dispatch is an orchestration heartbeat. A ROS-time timer created
            # before /clock activates can inherit a system-time deadline and
            # then wait forever after Gazebo resets time to zero.
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        mode = (
            "load generation enabled"
            if self._reconciliation_context.allow_loading
            else "unload-only"
        )
        worker = "dispatch enabled" if self._dispatch_enabled else "observer only"
        self.get_logger().info(
            f"Machine task queue ready ({mode}, {worker}, "
            f"{len(self._runtime.queue.tasks())} restored tasks) "
            f"at {self._store.path}"
        )

    def _declare_parameters(self) -> None:
        default_path = str(Path.home() / ".ros" / "factory_robot_tasks.json")
        self.declare_parameter(
            "machine_ids", ["machine_1", "machine_2", "machine_3"]
        )
        self.declare_parameter("queue_path", default_path)
        self.declare_parameter("order_id", "autonomous-production")
        self.declare_parameter("planned_quantity", 6)
        self.declare_parameter("allow_loading", True)
        self.declare_parameter("dispatch_enabled", False)
        self.declare_parameter("auto_recharge", True)
        self.declare_parameter("maximum_attempts", 2)
        self.declare_parameter("dispatch_period", 0.2)

    def _load_or_create_runtime(self) -> MachineTaskRuntimeState:
        if self._store.exists():
            return self._store.load()
        return MachineTaskRuntimeState(
            queue=RobotTaskQueue(),
            adapter=MachineEventAdapter(),
        )

    def _on_machine_state(self, message: MachineStateMsg) -> None:
        with self._lock:
            previous_runtime = MachineTaskRuntimeState.from_dict(
                self._runtime.to_dict()
            )
            try:
                event = self._runtime.adapter.observe(
                    machine_id=message.machine_id,
                    mode=message.state,
                    part_id=message.part_id,
                    door_open=message.door_open,
                    part_present=message.part_present,
                )
                if event is None:
                    return
                result = self._reconciler.apply(
                    event, self._reconciliation_context
                )
                self._store.save(self._runtime)
            except (KeyError, TypeError, ValueError, OSError) as exc:
                # Keep queue state and its event cursor transactional. A
                # repeated PLC sample can retry after a persistence failure.
                self._runtime = previous_runtime
                self._reconciler = MachineStateReconciler(self._runtime.queue)
                self.get_logger().error(
                    f"Rejected machine snapshot from {message.machine_id}: {exc}"
                )
                return

        if result.tasks_added or result.tasks_canceled:
            self.get_logger().info(
                f"{event.event_id}: added={result.tasks_added}, "
                f"canceled={result.tasks_canceled}"
            )
        for warning in result.warnings:
            self.get_logger().warning(warning)

    def _dispatch_next(self) -> None:
        if (
            not self._dispatch_enabled
            or self._dispatch_halted
            or self._active_task_id
            or not self._task_client.server_is_ready()
        ):
            return

        request = self._reserve_next_request()
        if request is None:
            return
        if not self._mark_active_task_dispatched():
            return

        goal = self._action_goal(request)
        self.get_logger().info(
            f"Dispatching robot task {request.task_id}: "
            f"{request.kind.value} {request.part_id} at {request.machine_id}"
        )
        try:
            future = self._task_client.send_goal_async(
                goal,
                feedback_callback=self._on_task_feedback,
            )
        except Exception as exc:
            self._finish_active_task(
                request.task_id,
                success=False,
                retryable=True,
                detail=f"action request was not sent: {exc}",
            )
            return
        future.add_done_callback(self._on_task_goal_response)

    def _action_goal(
        self, request: PhysicalRobotTaskRequest
    ) -> ExecuteRobotTask.Goal:
        goal = ExecuteRobotTask.Goal()
        goal.task_id = request.task_id
        goal.task_kind = (
            ExecuteRobotTask.Goal.LOAD_RAW
            if request.kind == RobotTaskKind.LOAD_RAW
            else ExecuteRobotTask.Goal.UNLOAD_FINISHED
        )
        goal.machine_id = request.machine_id
        goal.part_id = request.part_id
        goal.auto_recharge = self._auto_recharge
        return goal

    def _reserve_next_request(
        self,
    ) -> PhysicalRobotTaskRequest | None:
        """Durably reserve one task before it can reach the robot."""

        with self._lock:
            previous_runtime = MachineTaskRuntimeState.from_dict(
                self._runtime.to_dict()
            )
            task = self._runtime.queue.reserve_next()
            if task is None:
                return None

            try:
                request = physical_request_from_task(task)
            except (KeyError, TypeError, ValueError) as exc:
                self._reject_unmappable_task(
                    previous_runtime,
                    task.task_id,
                    str(exc),
                )
                return None

            try:
                self._store.save(self._runtime)
            except OSError as exc:
                self._restore_runtime(previous_runtime)
                self.get_logger().error(
                    f"Could not reserve {task.task_id}: {exc}"
                )
                return None

            self._active_task_id = task.task_id
            return request

    def _mark_active_task_dispatched(self) -> bool:
        """Persist the unsafe-to-replay boundary before sending an action."""

        with self._lock:
            task_id = self._active_task_id
            if not task_id:
                return False
            previous_runtime = MachineTaskRuntimeState.from_dict(
                self._runtime.to_dict()
            )
            try:
                self._runtime.queue.start(task_id)
                self._store.save(self._runtime)
            except (KeyError, ValueError, OSError) as exc:
                # No action has been sent yet. Restore RESERVED in memory;
                # restart recovery may safely return it to PENDING.
                self._restore_runtime(previous_runtime)
                self._active_task_id = ""
                self.get_logger().error(
                    f"Could not persist dispatch boundary for {task_id}: {exc}"
                )
                return False
            return True

    def _reject_unmappable_task(
        self,
        previous_runtime: MachineTaskRuntimeState,
        task_id: str,
        detail: str,
    ) -> None:
        self._restore_runtime(previous_runtime)
        task = self._runtime.queue.reserve_next()
        if task is None or task.task_id != task_id:
            raise RuntimeError("task reservation changed while holding the lock")
        self._runtime.queue.fail(task_id, detail)
        self._dispatch_halted = True
        try:
            self._store.save(self._runtime)
        except OSError as exc:
            self.get_logger().error(
                f"Could not persist rejected task {task_id}: {exc}"
            )
        self.get_logger().error(
            f"Task {task_id} has no safe physical mapping: {detail}"
        )

    def _restore_runtime(
        self, previous_runtime: MachineTaskRuntimeState
    ) -> None:
        self._runtime = previous_runtime
        self._reconciler = MachineStateReconciler(self._runtime.queue)

    def _on_task_goal_response(self, future) -> None:
        task_id = self._active_task_id
        try:
            goal_handle = future.result()
        except Exception as exc:  # rclpy transport errors are not uniform.
            self._finish_active_task(
                task_id,
                success=False,
                retryable=False,
                detail=f"action request failed before acceptance: {exc}",
            )
            return

        if not goal_handle.accepted:
            self._finish_active_task(
                task_id,
                success=False,
                retryable=True,
                detail="physical executor rejected the task",
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_task_result)

    def _on_task_result(self, future) -> None:
        task_id = self._active_task_id
        try:
            wrapped = future.result()
            result = wrapped.result
            succeeded = (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and result.success
            )
            self._finish_active_task(
                task_id,
                success=succeeded,
                retryable=bool(result.retryable),
                detail=result.message,
            )
        except Exception as exc:  # rclpy transport errors are not uniform.
            self._finish_active_task(
                task_id,
                success=False,
                retryable=False,
                detail=f"lost physical result; manual reconciliation required: {exc}",
            )

    def _finish_active_task(
        self,
        task_id: str,
        *,
        success: bool,
        retryable: bool,
        detail: str,
    ) -> None:
        with self._lock:
            if not task_id or task_id != self._active_task_id:
                return
            task = self._runtime.queue.get(task_id)
            if success:
                self._runtime.queue.succeed(task_id, detail)
            else:
                can_retry = retryable and task.attempts < self._maximum_attempts
                self._runtime.queue.fail(
                    task_id,
                    detail,
                    retryable=can_retry,
                )
                self._dispatch_halted = not can_retry
            try:
                self._store.save(self._runtime)
            except OSError as exc:
                self.get_logger().error(
                    f"Could not persist final state for {task_id}: {exc}"
                )
            self._active_task_id = ""

        outcome = "succeeded" if success else "failed"
        self.get_logger().info(f"Robot task {task_id} {outcome}: {detail}")
        if self._dispatch_halted:
            self.get_logger().error(
                "Physical outcome is unsafe to replay; dispatch is halted "
                "until robot, part, inventory and PLC state are reconciled"
            )

    def _on_task_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        with self._lock:
            task_id = self._active_task_id
            if task_id:
                previous_runtime = MachineTaskRuntimeState.from_dict(
                    self._runtime.to_dict()
                )
                try:
                    self._runtime.queue.record_progress(
                        task_id,
                        phase=feedback.phase,
                        feedback=feedback.detail,
                    )
                    self._store.save(self._runtime)
                except (KeyError, ValueError, OSError) as exc:
                    self._restore_runtime(previous_runtime)
                    self.get_logger().error(
                        f"Could not persist progress for {task_id}: {exc}"
                    )
        self.get_logger().info(
            f"{feedback.phase} {feedback.machine_id}: {feedback.detail}"
        )

    def _reconcile_task(self, request, response):
        """Apply a verified operator resolution and durably resume the queue."""
        with self._lock:
            if self._active_task_id:
                response.message = (
                    "cannot reconcile while a physical task is active"
                )
                return response
            previous_runtime = MachineTaskRuntimeState.from_dict(
                self._runtime.to_dict()
            )
            try:
                resolution = _RESOLUTION_FROM_MSG.get(request.resolution)
                if resolution is None:
                    raise ValueError(
                        f"unknown reconciliation resolution: {request.resolution}"
                    )
                task = self._runtime.queue.resolve_interrupted_task(
                    request.task_id,
                    resolution=resolution,
                    physical_state_verified=request.physical_state_verified,
                    operator_note=request.operator_note,
                )
                self._store.save(self._runtime)
            except (KeyError, ValueError, OSError) as exc:
                self._restore_runtime(previous_runtime)
                response.message = str(exc)
                return response

            self._dispatch_halted = any(
                item.status == RobotTaskStatus.FAILED
                for item in self._runtime.queue.tasks()
            )
            response.accepted = True
            response.task = self._task_message(task)
            response.message = (
                f"{task.task_id} reconciled as {task.status.value}; "
                + (
                    "dispatch remains halted by another failed task"
                    if self._dispatch_halted
                    else "dispatch may resume"
                )
            )
            return response

    def _get_queue(self, request, response):
        with self._lock:
            all_tasks = self._runtime.queue.tasks()
            visible_tasks = [
                task
                for task in all_tasks
                if request.include_terminal
                or task.status not in TERMINAL_TASK_STATUSES
            ]
            response.tasks = [self._task_message(task) for task in visible_tasks]
            response.pending_count = sum(
                task.status == RobotTaskStatus.PENDING for task in all_tasks
            )
            response.active_count = sum(
                task.status
                in (RobotTaskStatus.RESERVED, RobotTaskStatus.RUNNING)
                for task in all_tasks
            )
            response.terminal_count = sum(
                task.status in TERMINAL_TASK_STATUSES for task in all_tasks
            )
            response.persistence_path = str(self._store.path)
            response.message = (
                "queue snapshot; dispatch halted for manual reconciliation"
                if self._dispatch_halted
                else "queue snapshot"
            )
        return response

    @staticmethod
    def _task_message(task: RobotTask) -> RobotTaskStateMsg:
        message = RobotTaskStateMsg()
        message.task_id = task.task_id
        message.kind = _KIND_TO_MSG[task.kind]
        message.priority = int(task.priority)
        message.machine_id = task.machine_id
        message.part_id = task.part_id
        message.order_id = task.order_id
        message.source_event_id = task.source_event_id
        message.depends_on = list(task.depends_on)
        message.status = _STATUS_TO_MSG[task.status]
        message.attempts = task.attempts
        message.created_sequence = task.created_sequence
        message.detail = task.detail
        message.last_phase = task.last_phase
        message.last_feedback = task.last_feedback
        message.reconciliation_note = task.reconciliation_note
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MachineTaskQueueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
