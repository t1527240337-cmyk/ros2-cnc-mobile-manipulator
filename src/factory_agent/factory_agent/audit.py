from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path


class CommandAuditLog:
    """Append-only record of high-level Agent decisions and ROS acknowledgements."""

    def __init__(self, path: str | Path | None = None):
        configured_path = path or os.getenv("FACTORY_AGENT_AUDIT_LOG")
        self.path = (
            Path(configured_path).expanduser()
            if configured_path
            else Path.home() / ".ros" / "factory_agent_commands.jsonl"
        )
        self._lock = threading.Lock()

    def record(
        self,
        *,
        request_id: str,
        source: str,
        operation: str,
        accepted: bool,
        order_id: str,
        message: str,
    ) -> None:
        entry = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "request_id": request_id,
            "source": source,
            "operation": operation,
            "accepted": accepted,
            "order_id": order_id,
            "message": message,
        }
        line = json.dumps(
            entry,
            ensure_ascii=False,
            separators=(",", ":"),
        )
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(line)
                stream.write("\n")
