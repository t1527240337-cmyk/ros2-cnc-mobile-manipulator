from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import IntEnum
from typing import Any


class MachineMode(IntEnum):
    """Technology-neutral machine states exposed to the cell controller."""

    IDLE = 0
    READY = 1
    PROCESSING = 2
    DONE = 3
    FAULT = 4
    HELD = 5


class HeldPartKind(IntEnum):
    NONE = 0
    RAW = 1
    FINISHED = 2


@dataclass
class Machine:
    """Deterministic machine-tool state machine.

    The class models controller registers, not motors. A real PLC/ESP32 adapter
    may drive the door and publish the resulting state, but it must respect the
    same transition guards.
    """

    machine_id: str
    mode: MachineMode = MachineMode.IDLE
    door_open: bool = False
    part_id: str = ""
    remaining_seconds: float = 0.0
    cycle_seconds: float = 12.0
    fault_code: int = 0

    def open_door(self) -> None:
        if self.mode in (MachineMode.PROCESSING, MachineMode.HELD):
            raise ValueError(f"{self.machine_id}: stop and secure the spindle before opening")
        if self.mode == MachineMode.FAULT:
            raise ValueError(f"{self.machine_id}: reset fault before opening door")
        self.door_open = True

    def close_door(self) -> None:
        if self.mode == MachineMode.FAULT:
            raise ValueError(f"{self.machine_id}: reset fault before closing door")
        self.door_open = False

    def load(self, part_id: str) -> None:
        if self.mode != MachineMode.IDLE or not self.door_open or self.part_id:
            raise ValueError(f"{self.machine_id}: not ready to load")
        self.part_id = part_id
        self.mode = MachineMode.READY

    def start(self) -> None:
        if self.mode != MachineMode.READY or self.door_open or not self.part_id:
            raise ValueError(f"{self.machine_id}: close a loaded machine before start")
        self.mode = MachineMode.PROCESSING
        self.remaining_seconds = self.cycle_seconds

    def hold(self) -> None:
        """Request a controlled feed hold; this is not an emergency stop."""

        if self.mode != MachineMode.PROCESSING:
            raise ValueError(f"{self.machine_id}: only a processing machine can be held")
        self.mode = MachineMode.HELD

    def resume(self) -> None:
        if self.mode != MachineMode.HELD:
            raise ValueError(f"{self.machine_id}: machine is not held")
        if self.door_open or not self.part_id:
            raise ValueError(f"{self.machine_id}: cannot resume with unsafe door or fixture state")
        self.mode = MachineMode.PROCESSING

    def unload(self) -> str:
        if self.mode != MachineMode.DONE or not self.door_open or not self.part_id:
            raise ValueError(f"{self.machine_id}: finished part is not available")
        part_id = self.part_id
        self.part_id = ""
        self.mode = MachineMode.IDLE
        self.remaining_seconds = 0.0
        return part_id

    def tick(self, seconds: float) -> None:
        if self.mode != MachineMode.PROCESSING:
            return
        self.remaining_seconds = max(0.0, self.remaining_seconds - max(0.0, seconds))
        if self.remaining_seconds == 0.0:
            self.mode = MachineMode.DONE
            # The machine controller exposes a safe unload-ready state.
            self.door_open = True

    def inject_fault(self, code: int = 1) -> None:
        self.mode = MachineMode.FAULT
        self.fault_code = code
        self.remaining_seconds = 0.0

    def reset(self) -> None:
        if self.part_id:
            raise ValueError(f"{self.machine_id}: cannot reset while a part is trapped")
        self.mode = MachineMode.IDLE
        self.fault_code = 0
        self.door_open = False

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Machine":
        value = dict(data)
        value["mode"] = MachineMode(value["mode"])
        return cls(**value)


@dataclass
class ProductionOrder:
    order_id: str
    quantity: int
    allowed_machine_ids: list[str]
    auto_recharge: bool = True
    started: int = 0
    completed: int = 0

    def validate(self, known_machines: set[str], available_raw: int) -> None:
        if not self.order_id.strip():
            raise ValueError("order_id is required")
        if self.quantity < 1:
            raise ValueError("quantity must be positive")
        if self.quantity > available_raw:
            raise ValueError("quantity exceeds available raw-part stock")
        if not self.allowed_machine_ids:
            raise ValueError("at least one machine must be allowed")
        invalid = set(self.allowed_machine_ids) - known_machines
        if invalid:
            raise ValueError(f"unknown machines: {sorted(invalid)}")


@dataclass
class FactoryState:
    machines: dict[str, Machine]
    raw_part_count: int = 6
    finished_part_count: int = 0
    battery: float = 1.0
    held_part_id: str = ""
    held_part_kind: HeldPartKind = HeldPartKind.NONE
    pending_machine_id: str = ""
    charging: bool = False
    simulated_time: float = 0.0
    next_part_serial: int = 1
    events: list[str] = field(default_factory=list)

    @classmethod
    def default(
        cls, cycle_seconds: float = 12.0, raw_part_count: int = 6
    ) -> "FactoryState":
        if raw_part_count < 0:
            raise ValueError("raw_part_count cannot be negative")
        machines = {
            f"machine_{index}": Machine(
                f"machine_{index}", cycle_seconds=cycle_seconds
            )
            for index in range(1, 4)
        }
        return cls(machines=machines, raw_part_count=raw_part_count)

    def tick(self, seconds: float, battery_drain: float = 0.001) -> None:
        duration = max(0.0, seconds)
        self.simulated_time += duration
        for machine in self.machines.values():
            machine.tick(duration)
        rate = 0.03 if self.charging else -battery_drain
        self.battery = min(1.0, max(0.0, self.battery + duration * rate))

    def add_event(self, message: str) -> None:
        self.events.append(f"{self.simulated_time:07.2f}s {message}")

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["machines"] = {
            key: asdict(machine) for key, machine in self.machines.items()
        }
        value["held_part_kind"] = int(self.held_part_kind)
        for machine in value["machines"].values():
            machine["mode"] = int(machine["mode"])
        return value

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "FactoryState":
        value = dict(data)
        value["machines"] = {
            key: Machine.from_dict(machine)
            for key, machine in value["machines"].items()
        }
        value["held_part_kind"] = HeldPartKind(value["held_part_kind"])
        return cls(**value)
