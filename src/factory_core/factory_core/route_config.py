"""Typed configuration for deterministic factory transit routes."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import yaml

from .station_config import PlanarPose


@dataclass(frozen=True)
class FactoryRoute:
    """A named sequence of collision-free poses in the map frame."""

    name: str
    waypoints: tuple[PlanarPose, ...]


def load_factory_routes(config_path: str | Path) -> dict[str, FactoryRoute]:
    """Load named routes and reject empty or malformed waypoint lists."""
    path = Path(config_path)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    raw_routes = document.get("routes") if isinstance(document, dict) else None
    if not isinstance(raw_routes, dict) or not raw_routes:
        raise ValueError(f"No routes defined in {path}")

    routes: dict[str, FactoryRoute] = {}
    for route_name, raw_route in raw_routes.items():
        if not isinstance(raw_route, dict):
            raise ValueError(f"Route {route_name} must be a mapping")
        raw_waypoints = raw_route.get("waypoints")
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise ValueError(f"Route {route_name} needs at least one waypoint")
        routes[route_name] = FactoryRoute(
            name=route_name,
            waypoints=tuple(
                _waypoint(route_name, index, values)
                for index, values in enumerate(raw_waypoints)
            ),
        )
    return routes


def _waypoint(route_name: str, index: int, values: object) -> PlanarPose:
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(
            f"Route {route_name} waypoint {index} must be [x, y, yaw]"
        )
    try:
        x, y, yaw = (float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"Route {route_name} waypoint {index} must be numeric"
        ) from error
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError(
            f"Route {route_name} waypoint {index} must be finite"
        )
    return PlanarPose(x=x, y=y, yaw=yaw)
