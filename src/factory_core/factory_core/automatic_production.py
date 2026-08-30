"""Pure decision policy for automatic factory production."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FactoryAvailability:
    """The small state snapshot needed to decide whether to dispatch."""

    raw_part_count: int
    battery_percentage: float
    idle_machine_ids: tuple[str, ...]
    active_order_id: str = ""

    def __post_init__(self) -> None:
        if self.raw_part_count < 0:
            raise ValueError("raw_part_count cannot be negative")
        if not 0.0 <= self.battery_percentage <= 1.0:
            raise ValueError("battery_percentage must be within [0, 1]")


@dataclass(frozen=True)
class AutomaticOrderDecision:
    """Either a bounded order or a readable reason to keep waiting."""

    quantity: int = 0
    allowed_machine_ids: tuple[str, ...] = ()
    state: str = "waiting"
    reason: str = ""

    @property
    def should_dispatch(self) -> bool:
        return self.quantity > 0


def choose_automatic_order(
    availability: FactoryAvailability,
    *,
    allowed_machine_ids: tuple[str, ...],
    max_batch_size: int,
    minimum_battery: float,
) -> AutomaticOrderDecision:
    """Choose one production batch without mutating factory state."""

    if not 1 <= max_batch_size <= 3:
        raise ValueError("max_batch_size must be within [1, 3]")
    if not 0.0 <= minimum_battery <= 1.0:
        raise ValueError("minimum_battery must be within [0, 1]")

    if availability.active_order_id:
        return AutomaticOrderDecision(
            state="waiting_robot",
            reason=f"another order is active: {availability.active_order_id}",
        )
    if availability.raw_part_count == 0:
        return AutomaticOrderDecision(
            state="waiting_material",
            reason="raw inventory is empty",
        )
    if availability.battery_percentage < minimum_battery:
        return AutomaticOrderDecision(
            state="waiting_battery",
            reason=(
                f"battery {availability.battery_percentage:.1%} is below "
                f"the {minimum_battery:.1%} dispatch threshold"
            ),
        )

    idle = set(availability.idle_machine_ids)
    candidates = tuple(
        machine_id for machine_id in allowed_machine_ids if machine_id in idle
    )
    if not candidates:
        return AutomaticOrderDecision(
            state="waiting_machine",
            reason="no allowed machine is idle",
        )

    quantity = min(
        availability.raw_part_count,
        max_batch_size,
        len(candidates),
    )
    return AutomaticOrderDecision(
        quantity=quantity,
        allowed_machine_ids=candidates[:quantity],
        state="ready",
        reason=f"dispatch {quantity} part(s) to idle machines",
    )
