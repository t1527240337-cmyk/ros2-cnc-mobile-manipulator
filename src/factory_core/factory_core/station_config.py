"""Typed loading and validation for physical factory stations."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


VALID_STATION_TYPES = {"charging", "non_charging"}


@dataclass(frozen=True)
class PlanarPose:
    """A 2D pose expressed in the factory map frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class StationDefinition:
    """Configuration shared by navigation, perception and docking."""

    name: str
    station_type: str
    tag_id: int
    staging_pose: PlanarPose
    dock_pose: PlanarPose


def load_station_definitions(
    config_path: str | Path,
) -> dict[str, StationDefinition]:
    """Load every station and reject ambiguous physical configuration."""
    path = Path(config_path)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    raw_stations = document.get("stations") if isinstance(document, dict) else None
    if not isinstance(raw_stations, dict) or not raw_stations:
        raise ValueError(f"No stations defined in {path}")

    definitions: dict[str, StationDefinition] = {}
    used_tag_ids: set[int] = set()
    for name, raw_station in raw_stations.items():
        if not isinstance(raw_station, dict):
            raise ValueError(f"Station {name} must be a mapping")

        station_type = str(raw_station.get("type", ""))
        if station_type not in VALID_STATION_TYPES:
            raise ValueError(
                f"Station {name} type must be one of {sorted(VALID_STATION_TYPES)}"
            )

        tag_id = _tag_id(name, raw_station.get("tag_id"))
        if tag_id in used_tag_ids:
            raise ValueError(f"Station {name} reuses AprilTag ID {tag_id}")
        used_tag_ids.add(tag_id)

        definitions[name] = StationDefinition(
            name=name,
            station_type=station_type,
            tag_id=tag_id,
            staging_pose=_planar_pose(name, "staging_pose", raw_station),
            dock_pose=_planar_pose(name, "dock_pose", raw_station),
        )
    return definitions


def _tag_id(station_name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"Station {station_name} needs a non-negative integer tag_id")
    return value


def _planar_pose(
    station_name: str, field_name: str, station: dict
) -> PlanarPose:
    values = station.get(field_name)
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(
            f"Station {station_name} needs {field_name}: [x, y, yaw]"
        )
    try:
        x, y, yaw = (float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"Station {station_name} has a non-numeric {field_name}"
        ) from error
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError(f"Station {station_name} has a non-finite {field_name}")
    return PlanarPose(x=x, y=y, yaw=yaw)
