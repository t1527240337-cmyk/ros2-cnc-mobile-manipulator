from __future__ import annotations

from dataclasses import dataclass, field

from .command_models import AGENT_TOOL_DESCRIPTIONS


MACHINE_STATE_NAMES = {
    0: "IDLE",
    1: "READY",
    2: "PROCESSING",
    3: "DONE",
    4: "FAULT",
    5: "HELD",
}


@dataclass
class OperatorState:
    """Small presentation cache; it never decides machine transitions."""

    machines: dict[str, str] = field(default_factory=dict)
    battery_percentage: float | None = None
    raw_parts: int | None = None
    finished_parts: int | None = None
    active_order_id: str = ""
    last_order_id: str = ""
    phase: str = "idle"
    progress: str = "0/0"
    detail: str = ""

    def update_factory(self, response) -> None:
        self.machines = {
            machine.machine_id: MACHINE_STATE_NAMES.get(machine.state, "UNKNOWN")
            for machine in response.machines
        }
        self.battery_percentage = response.battery.percentage * 100.0
        self.raw_parts = response.raw_part_count
        self.finished_parts = response.finished_part_count
        self.active_order_id = response.active_order_id
        if response.active_order_id:
            self.last_order_id = response.active_order_id

    @property
    def tracked_order_id(self) -> str:
        return self.active_order_id or self.last_order_id

    def factory_summary(self) -> str:
        machines = ", ".join(
            f"{machine_id}={state}" for machine_id, state in sorted(self.machines.items())
        ) or "机床状态尚未收到"
        battery = (
            f"{self.battery_percentage:.1f}%"
            if self.battery_percentage is not None
            else "未知"
        )
        inventory = (
            f"毛坯={self.raw_parts}, 成品={self.finished_parts}"
            if self.raw_parts is not None
            else "库存未知"
        )
        return f"{machines}; {inventory}; 电量={battery}; 当前订单={self.active_order_id or '无'}"

    def start_order(self, order_id: str) -> None:
        self.active_order_id = order_id
        self.last_order_id = order_id
        self.phase = "accepted"
        self.progress = "0/0"
        self.detail = "订单已被确定性执行器接受"

    def finish_order(self, *, success: bool, message: str) -> None:
        self.last_order_id = self.tracked_order_id
        self.phase = "complete" if success else "failed"
        self.detail = message

    def task_summary(self) -> str:
        return (
            f"订单={self.tracked_order_id or '无'}; 阶段={self.phase}; "
            f"进度={self.progress}; {self.detail}"
        ).rstrip("; ")


def capability_summary() -> str:
    return "；".join(
        f"{operation.value}: {description}"
        for operation, description in AGENT_TOOL_DESCRIPTIONS.items()
    )
