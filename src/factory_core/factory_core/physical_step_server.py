"""Atomic ROS 2 Action boundary used by the BehaviorTree.CPP task layer."""

from __future__ import annotations

from factory_interfaces.action import ExecutePhysicalStep
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from .physical_order_plan import (
    MachineOperation,
    Manipulation,
    PhysicalStep,
    StepKind,
    TransferEvent,
)


class PhysicalStepServer:
    """Expose one verified physical operation at a time.

    BEGIN_TASK opens an exclusive session. All later goals must use the same
    task ID, and COMPLETE_TASK or ABORT_TASK closes it. The session preserves
    cross-step evidence such as a visually selected finished-bin slot without
    giving this worker ownership of the production sequence.
    """

    def __init__(self, owner, callback_group) -> None:
        self._owner = owner
        self._server = ActionServer(
            owner,
            ExecutePhysicalStep,
            "/factory/execute_physical_step",
            execute_callback=self._execute,
            goal_callback=self._accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=callback_group,
        )

    def _accept(self, goal: ExecutePhysicalStep.Goal) -> GoalResponse:
        error = self._validation_error(goal)
        if error:
            self._owner.get_logger().warning(
                f"Rejecting physical step for {goal.task_id!r}: {error}"
            )
            return GoalResponse.REJECT

        with self._owner._state_lock:
            if self._owner._goal_reserved:
                return GoalResponse.REJECT
            active = self._owner._active_order_id
            if goal.step_kind == ExecutePhysicalStep.Goal.BEGIN_TASK:
                if active:
                    return GoalResponse.REJECT
            elif goal.step_kind == ExecutePhysicalStep.Goal.ABORT_TASK:
                if active and active != goal.task_id:
                    return GoalResponse.REJECT
            elif active != goal.task_id:
                return GoalResponse.REJECT
            self._owner._goal_reserved = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle) -> ExecutePhysicalStep.Result:
        goal = goal_handle.request
        began_session = False
        success_message = None
        try:
            if goal.step_kind == ExecutePhysicalStep.Goal.BEGIN_TASK:
                self._owner._begin_order(goal.task_id)
                began_session = True
                self._owner._require_dependencies()
            elif goal.step_kind == ExecutePhysicalStep.Goal.COMPLETE_TASK:
                self._close_session()
            elif goal.step_kind == ExecutePhysicalStep.Goal.ABORT_TASK:
                self._close_session()
            else:
                self._owner._wait_at_safe_boundary(goal_handle, 0, 1)
                success_message = self._execute_operation(goal, goal_handle)
            self._owner.get_logger().info(
                "Physical step operation returned: "
                f"{self._phase(goal.step_kind)}"
            )

            goal_handle.succeed()
            self._owner.get_logger().info(
                f"Physical step terminal state published: {self._phase(goal.step_kind)}"
            )
            return self._result(
                True,
                ExecutePhysicalStep.Result.OK,
                success_message or f"{self._phase(goal.step_kind)} completed",
            )
        except Exception as error:
            cancelled = self._owner._is_cancel_requested(goal_handle)
            if began_session:
                self._close_session()
            if cancelled:
                goal_handle.canceled()
                return self._result(
                    False,
                    ExecutePhysicalStep.Result.CANCELLED,
                    str(error),
                )
            goal_handle.abort()
            return self._result(
                False,
                ExecutePhysicalStep.Result.EXECUTION_FAILED,
                str(error),
            )
        finally:
            # COMPLETE/ABORT already close the session. Every
            # other atomic goal releases only the per-goal reservation.
            with self._owner._state_lock:
                self._owner._goal_reserved = False

    def _close_session(self) -> None:
        with self._owner._state_lock:
            self._owner._active_order_id = ""
            self._owner._pause_requested = False
            self._owner._cancel_requested = False
            self._owner._pending_finished_slot = None

    def _execute_operation(self, goal, goal_handle) -> str | None:
        kind = goal.step_kind
        if kind == ExecutePhysicalStep.Goal.ENSURE_ENERGY:
            recharged = self._owner._ensure_cycle_energy(
                goal_handle,
                auto_recharge=bool(goal.auto_recharge),
                completed=0,
                total=1,
            )
            with self._owner._state_lock:
                battery = self._owner._battery_percentage
            percentage = (battery or 0.0) * 100.0
            if recharged:
                return (
                    f"charged to {percentage:.1f}%; resuming production"
                )
            return f"energy ready at {percentage:.1f}%"
        if kind == ExecutePhysicalStep.Goal.CHECK_MACHINE_IDLE:
            selected = self._owner._select_idle_machine(
                (goal.machine_id,), goal_handle, 0, 1
            )
            if selected != goal.machine_id:
                raise RuntimeError(
                    f"machine precondition selected {selected}, "
                    f"not {goal.machine_id}"
                )
            return

        step = self._physical_step(goal)
        self._owner._execute_step(step, goal_handle)

    @staticmethod
    def _physical_step(goal) -> PhysicalStep:
        kind = goal.step_kind
        if kind == ExecutePhysicalStep.Goal.DOCK:
            return PhysicalStep(StepKind.DOCK, station_id=goal.station_id)
        if kind == ExecutePhysicalStep.Goal.UNDOCK:
            return PhysicalStep(
                StepKind.UNDOCK,
                station_id=goal.station_id,
                stow_arm=bool(goal.stow_arm),
            )
        if kind in (
            ExecutePhysicalStep.Goal.PICK,
            ExecutePhysicalStep.Goal.PLACE,
        ):
            operation = (
                Manipulation.PICK
                if kind == ExecutePhysicalStep.Goal.PICK
                else Manipulation.PLACE
            )
            return PhysicalStep(
                StepKind.MANIPULATE,
                station_id=goal.station_id,
                part_id=goal.part_id,
                manipulation=operation,
            )

        machine_operations = {
            ExecutePhysicalStep.Goal.OPEN_DOOR: MachineOperation.OPEN_DOOR,
            ExecutePhysicalStep.Goal.CLOSE_DOOR: MachineOperation.CLOSE_DOOR,
            ExecutePhysicalStep.Goal.START_MACHINE: MachineOperation.START,
            ExecutePhysicalStep.Goal.CONFIRM_LOAD: MachineOperation.CONFIRM_LOAD,
            ExecutePhysicalStep.Goal.CONFIRM_UNLOAD: (
                MachineOperation.CONFIRM_UNLOAD
            ),
        }
        if kind in machine_operations:
            return PhysicalStep(
                StepKind.MACHINE_COMMAND,
                machine_id=goal.machine_id,
                part_id=goal.part_id,
                machine_operation=machine_operations[kind],
            )
        if kind == ExecutePhysicalStep.Goal.WAIT_MACHINE_DONE:
            return PhysicalStep(
                StepKind.WAIT_MACHINE_DONE,
                machine_id=goal.machine_id,
                part_id=goal.part_id,
            )

        transfers = {
            ExecutePhysicalStep.Goal.COMMIT_PICK_RAW: TransferEvent.PICK_FROM_RAW,
            ExecutePhysicalStep.Goal.COMMIT_PLACE_FINISHED: (
                TransferEvent.PLACE_IN_FINISHED
            ),
        }
        if kind in transfers:
            return PhysicalStep(
                StepKind.COMMIT_TRANSFER,
                part_id=goal.part_id,
                transfer_event=transfers[kind],
            )
        raise ValueError(f"unsupported physical step kind: {kind}")

    def _validation_error(self, goal) -> str:
        if not goal.task_id.strip():
            return "task_id is required"
        if goal.step_kind not in self._known_kinds():
            return "unknown step_kind"
        if goal.step_kind in (
            ExecutePhysicalStep.Goal.DOCK,
            ExecutePhysicalStep.Goal.UNDOCK,
            ExecutePhysicalStep.Goal.PICK,
            ExecutePhysicalStep.Goal.PLACE,
        ) and goal.station_id not in self._owner._stations:
            return f"unknown station {goal.station_id!r}"
        if goal.step_kind in (
            ExecutePhysicalStep.Goal.CHECK_MACHINE_IDLE,
            ExecutePhysicalStep.Goal.OPEN_DOOR,
            ExecutePhysicalStep.Goal.CLOSE_DOOR,
            ExecutePhysicalStep.Goal.START_MACHINE,
            ExecutePhysicalStep.Goal.CONFIRM_LOAD,
            ExecutePhysicalStep.Goal.CONFIRM_UNLOAD,
            ExecutePhysicalStep.Goal.WAIT_MACHINE_DONE,
        ) and goal.machine_id not in self._owner._machine_ids:
            return f"unknown machine {goal.machine_id!r}"
        if goal.step_kind in (
            ExecutePhysicalStep.Goal.PICK,
            ExecutePhysicalStep.Goal.PLACE,
        ) and not goal.part_id.strip():
            return "manipulation requires part_id"
        if goal.step_kind in (
            ExecutePhysicalStep.Goal.CONFIRM_LOAD,
            ExecutePhysicalStep.Goal.CONFIRM_UNLOAD,
            ExecutePhysicalStep.Goal.WAIT_MACHINE_DONE,
            ExecutePhysicalStep.Goal.COMMIT_PICK_RAW,
            ExecutePhysicalStep.Goal.COMMIT_PLACE_FINISHED,
        ) and not goal.part_id.strip():
            return "step requires part_id"
        return ""

    @staticmethod
    def _known_kinds() -> range:
        return range(
            ExecutePhysicalStep.Goal.BEGIN_TASK,
            ExecutePhysicalStep.Goal.ABORT_TASK + 1,
        )

    @staticmethod
    def _phase(kind: int) -> str:
        names = {
            ExecutePhysicalStep.Goal.BEGIN_TASK: "begin_task",
            ExecutePhysicalStep.Goal.ENSURE_ENERGY: "ensure_energy",
            ExecutePhysicalStep.Goal.CHECK_MACHINE_IDLE: "check_machine_idle",
            ExecutePhysicalStep.Goal.DOCK: "dock",
            ExecutePhysicalStep.Goal.UNDOCK: "undock",
            ExecutePhysicalStep.Goal.PICK: "pick",
            ExecutePhysicalStep.Goal.PLACE: "place",
            ExecutePhysicalStep.Goal.OPEN_DOOR: "open_door",
            ExecutePhysicalStep.Goal.CLOSE_DOOR: "close_door",
            ExecutePhysicalStep.Goal.START_MACHINE: "start_machine",
            ExecutePhysicalStep.Goal.CONFIRM_LOAD: "confirm_load",
            ExecutePhysicalStep.Goal.CONFIRM_UNLOAD: "confirm_unload",
            ExecutePhysicalStep.Goal.WAIT_MACHINE_DONE: "wait_machine_done",
            ExecutePhysicalStep.Goal.COMMIT_PICK_RAW: "commit_pick_raw",
            ExecutePhysicalStep.Goal.COMMIT_PLACE_FINISHED: (
                "commit_place_finished"
            ),
            ExecutePhysicalStep.Goal.COMPLETE_TASK: "complete_task",
            ExecutePhysicalStep.Goal.ABORT_TASK: "abort_task",
        }
        return names[kind]

    @staticmethod
    def _result(
        success: bool, error_code: int, message: str
    ) -> ExecutePhysicalStep.Result:
        result = ExecutePhysicalStep.Result()
        result.success = success
        result.retryable = False
        result.error_code = error_code
        result.message = message
        return result
