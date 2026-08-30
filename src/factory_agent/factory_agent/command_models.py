from __future__ import annotations

import re
from enum import Enum
from typing import List, Optional

try:
    from pydantic.v1 import BaseModel, Field, root_validator, validator
except ImportError:
    # Pydantic 1.x before the compatibility namespace was introduced.
    from pydantic import BaseModel, Field, root_validator, validator


MACHINE_PATTERN = re.compile(r"^machine_[1-3]$")
TASK_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,79}$")


class AgentOperation(str, Enum):
    SUBMIT_ORDER = "submit_order"
    GET_FACTORY_STATE = "get_factory_state"
    GET_TASK_STATUS = "get_task_status"
    START_AUTOMATIC = "start_automatic"
    STOP_AUTOMATIC = "stop_automatic"
    GET_AUTOMATIC_STATUS = "get_automatic_status"
    PAUSE_TASK = "pause_task"
    RESUME_TASK = "resume_task"
    CANCEL_TASK = "cancel_task"
    HOLD_MACHINE = "hold_machine"
    RESUME_MACHINE = "resume_machine"
    EXPLAIN_FAILURE = "explain_failure"
    LIST_CAPABILITIES = "list_capabilities"


class AgentCommand(BaseModel):
    """Only high-level, policy-checked commands may leave the Agent."""

    operation: AgentOperation
    quantity: Optional[int] = Field(default=None, ge=1)
    allowed_machine_ids: List[str] = Field(default_factory=list)
    target_machine_id: Optional[str] = None
    task_id: Optional[str] = None
    auto_recharge: bool = True
    query: Optional[str] = Field(default=None, max_length=500)

    @validator("allowed_machine_ids")
    def validate_allowed_machines(cls, value):
        machines = list(dict.fromkeys(value))
        invalid = [machine for machine in machines if not MACHINE_PATTERN.fullmatch(machine)]
        if invalid:
            raise ValueError(f"invalid machine ids: {invalid}")
        return machines

    @validator("target_machine_id")
    def validate_target_machine(cls, value):
        if value is not None and not MACHINE_PATTERN.fullmatch(value):
            raise ValueError(f"invalid target machine: {value}")
        return value

    @validator("task_id")
    def validate_task_id(cls, value):
        if value is None:
            return value
        normalized = value.strip()
        if not TASK_ID_PATTERN.fullmatch(normalized):
            raise ValueError("invalid task id")
        return normalized

    @validator("query")
    def validate_query(cls, value):
        if value is None:
            return value
        normalized = value.strip()
        if not normalized:
            raise ValueError("query cannot be empty")
        return normalized

    @root_validator
    def validate_operation_arguments(cls, values):
        operation = values.get("operation")
        if operation == AgentOperation.SUBMIT_ORDER and values.get("quantity") is None:
            raise ValueError("submit_order requires quantity")
        machine_operations = {
            AgentOperation.HOLD_MACHINE,
            AgentOperation.RESUME_MACHINE,
        }
        if operation in machine_operations and not values.get("target_machine_id"):
            raise ValueError(f"{operation.value} requires target_machine_id")
        if (
            operation == AgentOperation.EXPLAIN_FAILURE
            and not values.get("query")
        ):
            raise ValueError("explain_failure requires query")
        return values


AGENT_TOOL_DESCRIPTIONS = {
    AgentOperation.SUBMIT_ORDER: "提交正整数数量的生产订单；实时库存和成品区容量决定是否接单",
    AgentOperation.GET_FACTORY_STATE: "查询机床、库存、电量与当前订单",
    AgentOperation.GET_TASK_STATUS: "查询当前任务阶段和进度",
    AgentOperation.START_AUTOMATIC: "启动有料且机床空闲时的自动生产，可限制允许使用的机床",
    AgentOperation.STOP_AUTOMATIC: "完成当前订单后停止自动派发，不中断手持零件",
    AgentOperation.GET_AUTOMATIC_STATUS: "查询自动模式、当前自动订单和累计完成量",
    AgentOperation.PAUSE_TASK: "在安全检查点暂停当前任务",
    AgentOperation.RESUME_TASK: "恢复已暂停任务",
    AgentOperation.CANCEL_TASK: "取消任务并保存检查点",
    AgentOperation.HOLD_MACHINE: "请求单台机床受控进给保持，不等同于急停",
    AgentOperation.RESUME_MACHINE: "恢复处于进给保持状态的机床",
    AgentOperation.EXPLAIN_FAILURE: "结合状态和 SOP 解释故障及人工处理建议",
    AgentOperation.LIST_CAPABILITIES: "列出 Agent 可以执行的高层操作",
}


FORBIDDEN_LOW_LEVEL_INTERFACES = frozenset({
    "/cmd_vel",
    "/arm_controller/joint_trajectory",
    "/controller_manager/switch_controller",
    "emergency_stop_output",
    "spindle_motor_output",
})
