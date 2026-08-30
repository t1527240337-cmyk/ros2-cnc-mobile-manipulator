from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .machine_event_adapter import MachineEventAdapter
from .task_queue import RobotTaskQueue


@dataclass
class MachineTaskRuntimeState:
    queue: RobotTaskQueue
    adapter: MachineEventAdapter

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": 1,
            "queue": self.queue.to_dict(),
            "event_adapter": self.adapter.to_dict(),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MachineTaskRuntimeState":
        if data.get("version") != 1:
            raise ValueError("unsupported machine task runtime version")
        return cls(
            queue=RobotTaskQueue.from_dict(data["queue"]),
            adapter=MachineEventAdapter.from_dict(data["event_adapter"]),
        )


class MachineTaskRuntimeStore:
    """Atomically persist the queue and its PLC event cursor together."""

    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser()

    def save(self, state: MachineTaskRuntimeState) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(state.to_dict(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        os.replace(temporary, self.path)

    def load(self) -> MachineTaskRuntimeState:
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        return MachineTaskRuntimeState.from_dict(payload)

    def exists(self) -> bool:
        return self.path.is_file()
