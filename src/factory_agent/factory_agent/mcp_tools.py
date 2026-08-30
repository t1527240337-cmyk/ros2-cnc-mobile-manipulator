from __future__ import annotations

import json
from typing import Annotated, Any, Callable, Literal, Optional

from pydantic import Field

from .command_models import AgentCommand, AgentOperation


CommandExecutor = Callable[[AgentCommand, str], dict]

OrderQuantity = Annotated[int, Field(ge=1)]
MachineId = Literal[
    "machine_1",
    "machine_2",
    "machine_3",
]

MCP_TOOL_NAMES = (
    "get_factory_state",
    "submit_order",
    "get_task_status",
    "start_automatic",
    "stop_automatic",
    "get_automatic_status",
    "pause_task",
    "resume_task",
    "cancel_task",
    "hold_machine",
    "resume_machine",
    "explain_failure",
    "list_capabilities",
)


def decode_json_payload(value: str) -> Any:
    """Decode the structured ROS payload without importing ROS dependencies."""

    if not value:
        return {}
    try:
        return json.loads(value)
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            "factory Agent returned invalid data_json"
        ) from exc


class FactoryMcpTools:
    """Allow-listed MCP tools backed by one structured Agent command service."""

    def __init__(self, execute: CommandExecutor):
        self._execute = execute

    def get_factory_state(self) -> dict:
        """Return machine states, inventory, battery and active order."""
        return self._run(AgentOperation.GET_FACTORY_STATE)

    def submit_order(
        self,
        quantity: OrderQuantity,
        allowed_machine_ids: Optional[list[MachineId]] = None,
        auto_recharge: bool = True,
    ) -> dict:
        """Submit an order whose quantity is checked against live inventory."""
        return self._run(
            AgentOperation.SUBMIT_ORDER,
            quantity=quantity,
            allowed_machine_ids=list(allowed_machine_ids or []),
            auto_recharge=auto_recharge,
        )

    def get_task_status(self, task_id: Optional[str] = None) -> dict:
        """Return the current high-level task phase and progress."""
        return self._run(AgentOperation.GET_TASK_STATUS, task_id=task_id)

    def start_automatic(
        self,
        allowed_machine_ids: Optional[list[MachineId]] = None,
    ) -> dict:
        """Start automatic production while raw stock and a machine are available."""
        return self._run(
            AgentOperation.START_AUTOMATIC,
            allowed_machine_ids=list(allowed_machine_ids or []),
        )

    def stop_automatic(self) -> dict:
        """Stop automatic dispatch after the current safely held work is finished."""
        return self._run(AgentOperation.STOP_AUTOMATIC)

    def get_automatic_status(self) -> dict:
        """Return automatic-production state and production counters."""
        return self._run(AgentOperation.GET_AUTOMATIC_STATUS)

    def pause_task(self, task_id: Optional[str] = None) -> dict:
        """Pause a task at a deterministic safe checkpoint."""
        return self._run(AgentOperation.PAUSE_TASK, task_id=task_id)

    def resume_task(self, task_id: Optional[str] = None) -> dict:
        """Resume a task that was paused at a safe checkpoint."""
        return self._run(AgentOperation.RESUME_TASK, task_id=task_id)

    def cancel_task(self, task_id: Optional[str] = None) -> dict:
        """Cancel a task through the ROS task manager; this is not an emergency stop."""
        return self._run(AgentOperation.CANCEL_TASK, task_id=task_id)

    def hold_machine(self, machine_id: MachineId) -> dict:
        """Request a controlled feed hold on one machine."""
        return self._run(
            AgentOperation.HOLD_MACHINE,
            target_machine_id=machine_id,
        )

    def resume_machine(self, machine_id: MachineId) -> dict:
        """Resume a machine that is in controlled hold."""
        return self._run(
            AgentOperation.RESUME_MACHINE,
            target_machine_id=machine_id,
        )

    def explain_failure(self, query: str) -> dict:
        """Retrieve relevant SOP guidance and explain a failure."""
        return self._run(AgentOperation.EXPLAIN_FAILURE, query=query)

    def list_capabilities(self) -> dict:
        """List every high-level operation exposed through this MCP server."""
        return self._run(AgentOperation.LIST_CAPABILITIES)

    def _run(self, operation: AgentOperation, **arguments) -> dict:
        command = AgentCommand(operation=operation, **arguments)
        return self._execute(command, "mcp")
