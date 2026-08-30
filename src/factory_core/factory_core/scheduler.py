from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .domain import FactoryState, HeldPartKind, MachineMode, ProductionOrder


class DecisionKind(str, Enum):
    PICK_RAW = "pick_raw"
    LOAD_MACHINE = "load_machine"
    PICK_FINISHED = "pick_finished"
    PLACE_FINISHED = "place_finished"
    DOCK = "dock"
    WAIT = "wait"
    COMPLETE = "complete"
    BLOCKED = "blocked"


@dataclass(frozen=True)
class Decision:
    kind: DecisionKind
    machine_id: str = ""
    detail: str = ""


class Scheduler:
    """Pure deterministic dispatcher; it never emits actuator commands."""

    def __init__(self, low_battery: float = 0.25, charge_target: float = 0.80):
        if not 0.0 < low_battery < charge_target <= 1.0:
            raise ValueError("invalid battery thresholds")
        self.low_battery = low_battery
        self.charge_target = charge_target

    def next_decision(self, state: FactoryState, order: ProductionOrder) -> Decision:
        if order.completed >= order.quantity:
            return Decision(DecisionKind.COMPLETE, detail="requested quantity completed")

        allowed = [state.machines[mid] for mid in order.allowed_machine_ids]
        idle = sorted(
            (machine for machine in allowed if machine.mode == MachineMode.IDLE),
            key=lambda machine: machine.machine_id,
        )

        if state.held_part_kind == HeldPartKind.RAW:
            target = state.machines.get(state.pending_machine_id)
            if target is not None and target.mode == MachineMode.IDLE:
                return Decision(
                    DecisionKind.LOAD_MACHINE,
                    target.machine_id,
                    "finish the safe transfer before recharge",
                )
            if idle:
                return Decision(
                    DecisionKind.LOAD_MACHINE,
                    idle[0].machine_id,
                    f"reassign held raw part from unavailable "
                    f"{state.pending_machine_id or 'machine'}",
                )
            return Decision(
                DecisionKind.BLOCKED,
                state.pending_machine_id,
                "held raw part has no healthy idle machine; keep the part "
                "secured and request operator assistance",
            )
        if state.held_part_kind == HeldPartKind.FINISHED:
            return Decision(DecisionKind.PLACE_FINISHED, detail="place held finished part")

        if order.auto_recharge and state.battery < self.low_battery:
            return Decision(DecisionKind.DOCK, detail="battery below threshold")

        if order.started < order.quantity and state.raw_part_count > 0 and idle:
            return Decision(DecisionKind.PICK_RAW, idle[0].machine_id,
                            "fill idle machine before collecting output")

        done = sorted((m for m in allowed if m.mode == MachineMode.DONE), key=lambda m: m.machine_id)
        if done:
            return Decision(DecisionKind.PICK_FINISHED, done[0].machine_id, "collect completed part")

        trapped = [m for m in allowed if m.mode == MachineMode.FAULT and m.part_id]
        if trapped:
            return Decision(DecisionKind.BLOCKED, trapped[0].machine_id,
                            "part trapped in faulted machine; manual intervention required")

        active = any(
            machine.mode in (MachineMode.READY, MachineMode.PROCESSING, MachineMode.HELD)
            for machine in allowed
        )
        if active:
            return Decision(DecisionKind.WAIT, detail="machines are processing")
        if order.started < order.quantity and not idle:
            return Decision(DecisionKind.BLOCKED, detail="no healthy idle machine available")
        return Decision(DecisionKind.BLOCKED, detail="order cannot make progress")


class SimulationEngine:
    """Semantic adapter used by tests and the ROS demo runtime.

    Real Nav2/MoveIt adapters implement the same decision effects but only commit
    state after their action result succeeds.
    """

    ACTION_SECONDS = {
        DecisionKind.PICK_RAW: 2.0,
        DecisionKind.LOAD_MACHINE: 3.0,
        DecisionKind.PICK_FINISHED: 3.0,
        DecisionKind.PLACE_FINISHED: 2.0,
        DecisionKind.DOCK: 4.0,
        DecisionKind.WAIT: 1.0,
    }

    def __init__(self, state: FactoryState, order: ProductionOrder, scheduler: Scheduler | None = None):
        order.validate(set(state.machines), state.raw_part_count)
        self.state = state
        self.order = order
        self.scheduler = scheduler or Scheduler()

    def step(self) -> Decision:
        decision = self.scheduler.next_decision(self.state, self.order)
        state = self.state

        if decision.kind == DecisionKind.PICK_RAW:
            state.raw_part_count -= 1
            state.held_part_id = f"raw_{state.next_part_serial:03d}"
            state.next_part_serial += 1
            state.held_part_kind = HeldPartKind.RAW
            state.pending_machine_id = decision.machine_id
            state.add_event(f"picked {state.held_part_id} for {decision.machine_id}")

        elif decision.kind == DecisionKind.LOAD_MACHINE:
            machine = state.machines[decision.machine_id]
            machine.open_door()
            machine.load(state.held_part_id)
            machine.close_door()
            machine.start()
            self.order.started += 1
            state.add_event(f"loaded {machine.part_id} into {machine.machine_id}; cycle started")
            state.held_part_id = ""
            state.held_part_kind = HeldPartKind.NONE
            state.pending_machine_id = ""

        elif decision.kind == DecisionKind.PICK_FINISHED:
            machine = state.machines[decision.machine_id]
            machine.open_door()
            state.held_part_id = machine.unload().replace("raw_", "finished_")
            state.held_part_kind = HeldPartKind.FINISHED
            state.add_event(f"unloaded {state.held_part_id} from {machine.machine_id}")
            machine.close_door()

        elif decision.kind == DecisionKind.PLACE_FINISHED:
            state.finished_part_count += 1
            self.order.completed += 1
            state.add_event(f"placed {state.held_part_id} in finished bin")
            state.held_part_id = ""
            state.held_part_kind = HeldPartKind.NONE

        elif decision.kind == DecisionKind.DOCK:
            state.charging = True
            state.add_event("docked at charge station")
            charge_seconds = (self.scheduler.charge_target - state.battery) / 0.03
            state.tick(max(0.0, charge_seconds), battery_drain=0.0)
            state.charging = False
            state.add_event(f"charge complete at {state.battery:.0%}")
            return decision

        duration = self.ACTION_SECONDS.get(decision.kind, 0.0)
        if duration:
            state.tick(duration)
        return decision

    def run(self, max_steps: int = 500) -> Decision:
        for _ in range(max_steps):
            decision = self.step()
            if decision.kind in (DecisionKind.COMPLETE, DecisionKind.BLOCKED):
                return decision
        return Decision(DecisionKind.BLOCKED, detail="step budget exhausted")
