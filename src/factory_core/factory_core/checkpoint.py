from __future__ import annotations

import json
import os
from dataclasses import asdict
from pathlib import Path

from .domain import FactoryState, ProductionOrder


class CheckpointStore:
    def __init__(self, path: str | Path):
        self.path = Path(path)

    def save(self, state: FactoryState, order: ProductionOrder) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        payload = {"version": 1, "state": state.to_dict(), "order": asdict(order)}
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(temporary, self.path)

    def load(self) -> tuple[FactoryState, ProductionOrder]:
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        if payload.get("version") != 1:
            raise ValueError("unsupported checkpoint version")
        return FactoryState.from_dict(payload["state"]), ProductionOrder(**payload["order"])

    def exists(self) -> bool:
        return self.path.is_file()
