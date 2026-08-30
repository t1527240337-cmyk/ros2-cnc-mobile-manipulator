"""Pure, reviewable production steps for one physical workpiece cycle."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class StepKind(str, Enum):
    DOCK = "dock"
    UNDOCK = "undock"
    MANIPULATE = "manipulate"
    MACHINE_COMMAND = "machine_command"
    WAIT_MACHINE_DONE = "wait_machine_done"
    COMMIT_TRANSFER = "commit_transfer"


class Manipulation(str, Enum):
    PICK = "pick"
    PLACE = "place"


class MachineOperation(str, Enum):
    OPEN_DOOR = "open_door"
    CLOSE_DOOR = "close_door"
    START = "start"
    CONFIRM_LOAD = "confirm_load"
    CONFIRM_UNLOAD = "confirm_unload"


class TransferEvent(str, Enum):
    PICK_FROM_RAW = "pick_from_raw"
    PLACE_IN_FINISHED = "place_in_finished"


class EnergyDecision(str, Enum):
    """Safe-boundary battery decision for a physical production cycle."""

    CONTINUE = "continue"
    RECHARGE = "recharge"
    BLOCK = "block"


def energy_decision(
    battery_percentage: float,
    *,
    auto_recharge: bool,
    low_threshold: float,
) -> EnergyDecision:
    """Decide whether a new part-transfer cycle may start.

    This policy is evaluated only while the gripper is empty. Once a pick or
    unload cycle starts, the executor finishes placing the held workpiece
    before evaluating battery state again.
    """
    if not 0.0 <= battery_percentage <= 1.0:
        raise ValueError("battery_percentage must be in [0, 1]")
    if not 0.0 < low_threshold < 1.0:
        raise ValueError("low_threshold must be in (0, 1)")
    if battery_percentage >= low_threshold:
        return EnergyDecision.CONTINUE
    if auto_recharge:
        return EnergyDecision.RECHARGE
    return EnergyDecision.BLOCK


@dataclass(frozen=True)
class PhysicalPartAssignment:
    """One traceable workpiece assigned to one CNC."""

    machine_id: str
    part_id: str

    def validate(self) -> None:
        if not self.machine_id:
            raise ValueError("machine_id is required")
        if not self.part_id:
            raise ValueError("part_id is required")


PART_CYCLE_STEP_COUNT = 20


@dataclass(frozen=True)
class PhysicalStep:
    """One atomic boundary in a physical production cycle."""

    kind: StepKind
    station_id: str = ""
    machine_id: str = ""
    part_id: str = ""
    manipulation: Manipulation | None = None
    machine_operation: MachineOperation | None = None
    transfer_event: TransferEvent | None = None
    stow_arm: bool = False

    @property
    def phase(self) -> str:
        if self.manipulation is not None:
            return f"{self.manipulation.value}_{self.station_id}"
        if self.machine_operation is not None:
            return f"machine_{self.machine_operation.value}"
        if self.transfer_event is not None:
            return f"commit_{self.transfer_event.value}"
        if self.kind == StepKind.WAIT_MACHINE_DONE:
            return "wait_machine_done"
        return f"{self.kind.value}_{self.station_id}"


def build_load_cycle(
    assignment: PhysicalPartAssignment,
) -> tuple[PhysicalStep, ...]:
    """Build raw-bin pick through CNC process start, without waiting.

    State-changing acknowledgement steps deliberately follow the physical
    action that proves them. This ordering prevents a failed motion from
    creating an impossible inventory or PLC state.
    """
    assignment.validate()
    machine_id = assignment.machine_id
    part_id = assignment.part_id

    return (
        PhysicalStep(StepKind.DOCK, station_id="raw_bin"),
        PhysicalStep(
            StepKind.MANIPULATE,
            station_id="raw_bin",
            part_id=part_id,
            manipulation=Manipulation.PICK,
        ),
        PhysicalStep(
            StepKind.COMMIT_TRANSFER,
            part_id=part_id,
            transfer_event=TransferEvent.PICK_FROM_RAW,
        ),
        PhysicalStep(StepKind.UNDOCK, station_id="raw_bin"),
        PhysicalStep(
            StepKind.MACHINE_COMMAND,
            machine_id=machine_id,
            machine_operation=MachineOperation.OPEN_DOOR,
        ),
        PhysicalStep(StepKind.DOCK, station_id=machine_id),
        PhysicalStep(
            StepKind.MANIPULATE,
            station_id=machine_id,
            part_id=part_id,
            manipulation=Manipulation.PLACE,
        ),
        PhysicalStep(
            StepKind.MACHINE_COMMAND,
            machine_id=machine_id,
            part_id=part_id,
            machine_operation=MachineOperation.CONFIRM_LOAD,
        ),
        # Machine PLACE already finishes at the collision-checked travel
        # posture used for loaded navigation. Keep it while waiting outside
        # the same CNC instead of folding and immediately unfolding again.
        PhysicalStep(StepKind.UNDOCK, station_id=machine_id),
        PhysicalStep(
            StepKind.MACHINE_COMMAND,
            machine_id=machine_id,
            machine_operation=MachineOperation.CLOSE_DOOR,
        ),
        PhysicalStep(
            StepKind.MACHINE_COMMAND,
            machine_id=machine_id,
            machine_operation=MachineOperation.START,
        ),
    )


def build_unload_cycle(
    assignment: PhysicalPartAssignment,
) -> tuple[PhysicalStep, ...]:
    """Build process completion wait through finished-bin inventory commit."""
    assignment.validate()
    machine_id = assignment.machine_id
    part_id = assignment.part_id

    return (
        PhysicalStep(
            StepKind.WAIT_MACHINE_DONE,
            machine_id=machine_id,
            part_id=part_id,
        ),
        PhysicalStep(StepKind.DOCK, station_id=machine_id),
        PhysicalStep(
            StepKind.MANIPULATE,
            station_id=machine_id,
            part_id=part_id,
            manipulation=Manipulation.PICK,
        ),
        PhysicalStep(
            StepKind.MACHINE_COMMAND,
            machine_id=machine_id,
            part_id=part_id,
            machine_operation=MachineOperation.CONFIRM_UNLOAD,
        ),
        PhysicalStep(StepKind.UNDOCK, station_id=machine_id),
        PhysicalStep(StepKind.DOCK, station_id="finished_bin"),
        PhysicalStep(
            StepKind.MANIPULATE,
            station_id="finished_bin",
            part_id=part_id,
            manipulation=Manipulation.PLACE,
        ),
        PhysicalStep(
            StepKind.COMMIT_TRANSFER,
            part_id=part_id,
            transfer_event=TransferEvent.PLACE_IN_FINISHED,
        ),
        PhysicalStep(StepKind.UNDOCK, station_id="finished_bin"),
    )


def build_part_cycle(
    *,
    machine_id: str,
    part_id: str,
) -> tuple[PhysicalStep, ...]:
    """Build the complete one-workpiece workflow."""
    assignment = PhysicalPartAssignment(
        machine_id=machine_id,
        part_id=part_id,
    )
    steps = build_load_cycle(assignment) + build_unload_cycle(assignment)
    if len(steps) != PART_CYCLE_STEP_COUNT:
        raise RuntimeError(
            f"physical part cycle has {len(steps)} steps, "
            f"expected {PART_CYCLE_STEP_COUNT}"
        )
    return steps


def production_batch_sizes(
    quantity: int, available_machine_count: int
) -> tuple[int, ...]:
    """Split an order into waves that never reuse a busy CNC."""
    if quantity < 1:
        raise ValueError("quantity must be positive")
    if available_machine_count < 1:
        raise ValueError("at least one machine is required")

    sizes: list[int] = []
    remaining = quantity
    while remaining:
        size = min(remaining, available_machine_count)
        sizes.append(size)
        remaining -= size
    return tuple(sizes)
