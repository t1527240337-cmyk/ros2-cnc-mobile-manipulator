from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .task_models import RobotTask
from .task_queue import RobotTaskQueue


class DispatchKind(str, Enum):
    EXECUTE_TASK = "execute_task"
    DOCK_AND_CHARGE = "dock_and_charge"
    WAIT = "wait"


@dataclass(frozen=True)
class DispatchDecision:
    kind: DispatchKind
    task: RobotTask | None = None
    detail: str = ""


class RobotTaskDispatcher:
    """Applies robot availability and battery policy before reserving work."""

    def __init__(self, low_battery: float = 0.25):
        if not 0.0 < low_battery < 1.0:
            raise ValueError("low_battery must be between zero and one")
        self.low_battery = low_battery

    def next_decision(
        self,
        queue: RobotTaskQueue,
        *,
        battery: float,
        robot_busy: bool = False,
        holding_part: bool = False,
    ) -> DispatchDecision:
        if robot_busy or holding_part:
            return DispatchDecision(
                DispatchKind.WAIT,
                detail="current mission must reach a safe checkpoint",
            )
        if battery < self.low_battery:
            return DispatchDecision(
                DispatchKind.DOCK_AND_CHARGE,
                detail="battery below dispatch threshold; queued work is preserved",
            )

        task = queue.reserve_next()
        if task is None:
            return DispatchDecision(DispatchKind.WAIT, detail="no eligible robot task")
        return DispatchDecision(DispatchKind.EXECUTE_TASK, task=task)
