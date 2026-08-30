"""Deterministic geometry for an unordered, single-layer parts bin."""

from __future__ import annotations

from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class WorkspaceBounds:
    """Horizontal work area available to workpiece centres."""

    x: tuple[float, float]
    y: tuple[float, float]

    def validate(self) -> None:
        if self.x[0] >= self.x[1] or self.y[0] >= self.y[1]:
            raise ValueError("workspace bounds must have positive area")


@dataclass(frozen=True)
class LayoutPoint:
    """One unordered workpiece centre and its irrelevant upright yaw."""

    x: float
    y: float
    yaw: float


def sample_unordered_layout(
    *,
    seed: int,
    count: int,
    bounds: WorkspaceBounds,
    minimum_center_distance: float,
    attempts_per_part: int = 2000,
    layout_restarts: int = 80,
) -> tuple[LayoutPoint, ...]:
    """Sample a reproducible layout without assigning parts to lanes or slots.

    Rejection sampling enforces a real geometric separation contract.  A
    layout that cannot satisfy the requested clearance raises instead of
    silently reducing the distance or returning a partially provisioned bin.
    """

    bounds.validate()
    if count < 1:
        raise ValueError("layout count must be positive")
    if minimum_center_distance <= 0.0:
        raise ValueError("minimum_center_distance must be positive")
    if attempts_per_part < 1 or layout_restarts < 1:
        raise ValueError("sampling budgets must be positive")

    generator = random.Random(seed)
    for _ in range(layout_restarts):
        points: list[LayoutPoint] = []
        for _part_index in range(count):
            accepted = None
            for _attempt in range(attempts_per_part):
                candidate = LayoutPoint(
                    x=generator.uniform(*bounds.x),
                    y=generator.uniform(*bounds.y),
                    yaw=generator.uniform(-math.pi, math.pi),
                )
                if all(
                    math.hypot(
                        candidate.x - existing.x,
                        candidate.y - existing.y,
                    ) >= minimum_center_distance
                    for existing in points
                ):
                    accepted = candidate
                    break
            if accepted is None:
                break
            points.append(accepted)
        if len(points) == count:
            return tuple(points)

    raise ValueError(
        "could not generate an unordered layout with the requested count "
        "and minimum centre distance"
    )
