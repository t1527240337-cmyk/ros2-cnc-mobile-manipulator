from __future__ import annotations

import json
import os
from pathlib import Path

from .task_queue import RobotTaskQueue


class RobotTaskQueueStore:
    """Atomic JSON persistence for event-generated robot work."""

    def __init__(self, path: str | Path):
        self.path = Path(path)

    def save(self, queue: RobotTaskQueue) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(queue.to_dict(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        os.replace(temporary, self.path)

    def load(self) -> RobotTaskQueue:
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        return RobotTaskQueue.from_dict(payload)

    def exists(self) -> bool:
        return self.path.is_file()
