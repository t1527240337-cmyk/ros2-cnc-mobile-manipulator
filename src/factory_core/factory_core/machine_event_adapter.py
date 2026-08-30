from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
from uuid import uuid4

from .domain import MachineMode
from .task_models import MachineEvent


@dataclass
class MachineEventAdapter:
    """Convert repeated ROS machine snapshots into ordered semantic events.

    PLC state topics publish continuously. Robot tasks, however, must only be
    created when the task-relevant state changes. Door motion is deliberately
    excluded: opening an IDLE machine must not enqueue a second load.
    """

    controller_session: str = field(default_factory=lambda: uuid4().hex)
    sequences: dict[str, int] = field(default_factory=dict)
    snapshots: dict[str, tuple[int, str, bool]] = field(default_factory=dict)

    def observe(
        self,
        *,
        machine_id: str,
        mode: int,
        part_id: str = "",
        door_open: bool = False,
        part_present: bool | None = None,
    ) -> MachineEvent | None:
        if not machine_id:
            raise ValueError("machine_id is required")
        MachineMode(mode)

        # Door motion changes the PLC snapshot but does not create robot work.
        del door_open
        present = bool(part_id) if part_present is None else bool(part_present)
        snapshot = (int(mode), part_id, present)
        if self.snapshots.get(machine_id) == snapshot:
            return None

        sequence = self.sequences.get(machine_id, 0) + 1
        self.sequences[machine_id] = sequence
        self.snapshots[machine_id] = snapshot
        return MachineEvent(
            machine_id=machine_id,
            controller_session=self.controller_session,
            sequence=sequence,
            mode=int(mode),
            part_present=present,
            part_id=part_id,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": 1,
            "controller_session": self.controller_session,
            "sequences": dict(self.sequences),
            "snapshots": {
                machine_id: {
                    "mode": mode,
                    "part_id": part_id,
                    "part_present": part_present,
                }
                for machine_id, (mode, part_id, part_present)
                in self.snapshots.items()
            },
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MachineEventAdapter":
        if data.get("version") != 1:
            raise ValueError("unsupported machine event adapter version")
        session = str(data.get("controller_session", ""))
        if not session:
            raise ValueError("controller_session is required")

        adapter = cls(controller_session=session)
        adapter.sequences = {
            str(machine_id): int(sequence)
            for machine_id, sequence in data.get("sequences", {}).items()
        }
        adapter.snapshots = {
            str(machine_id): (
                int(snapshot["mode"]),
                str(snapshot.get("part_id", "")),
                bool(
                    snapshot.get(
                        "part_present", snapshot.get("part_id", "")
                    )
                ),
            )
            for machine_id, snapshot in data.get("snapshots", {}).items()
        }
        return adapter
