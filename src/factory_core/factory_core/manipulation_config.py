"""Typed geometry for perceived sources, CNC fixtures and output slots."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


PICK_OPERATION = 0
PLACE_OPERATION = 1

VALID_STATION_ROLES = {"source", "sink", "machine"}


@dataclass(frozen=True)
class CartesianPose:
    """A tool-center-point pose expressed in a named TF frame."""

    frame_id: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]

    def above(self, distance: float) -> "CartesianPose":
        """Return a vertically translated pose without mutating this pose."""
        return self.translated((0.0, 0.0, distance))

    def translated(self, offset: tuple[float, float, float]) -> "CartesianPose":
        """Return a pose translated in the same reference frame."""
        x, y, z = self.position
        dx, dy, dz = offset
        return CartesianPose(
            frame_id=self.frame_id,
            position=(x + dx, y + dy, z + dz),
            orientation=self.orientation,
        )


@dataclass(frozen=True)
class PlacementSlot:
    """One calibrated destination position in the finished-product tray."""

    slot_id: int
    pose: CartesianPose


@dataclass(frozen=True)
class ManipulationTarget:
    """Resolved nominal geometry for one manipulation request."""

    pose: CartesianPose
    placement_slot_id: int = 0


@dataclass(frozen=True)
class StationFixtureReference:
    """A calibrated loading datum expressed in a perceived station frame."""

    frame_id: str
    position: tuple[float, float, float]


@dataclass(frozen=True)
class ManipulationStation:
    """Geometry and role needed by the manipulation action server."""

    name: str
    role: str
    approach_offset: tuple[float, float, float]
    lift_offset: tuple[float, float, float]
    grasp_offset: tuple[float, float, float]
    fixture_reference: StationFixtureReference | None
    clearance_pose: CartesianPose | None
    source_selection_pose: CartesianPose | None
    machine_workpiece_pose: CartesianPose | None
    placement_slots: dict[int, PlacementSlot]

    def require_placement_slot(self, slot_id: int) -> PlacementSlot:
        try:
            return self.placement_slots[slot_id]
        except KeyError as error:
            raise ValueError(
                f"Station {self.name} has no placement slot {slot_id}"
            ) from error


def resolve_manipulation_request(
    stations: dict[str, ManipulationStation],
    operation: int,
    station_name: str,
    part_id: str,
    placement_slot_id: int = 0,
) -> tuple[ManipulationStation, ManipulationTarget]:
    """Validate business intent before any controller receives a command."""
    if not part_id:
        raise ValueError("Manipulation request needs a part_id")
    try:
        station = stations[station_name]
    except KeyError as error:
        raise ValueError(f"Unknown manipulation station: {station_name}") from error

    if operation == PICK_OPERATION:
        if station.role not in {"source", "machine"}:
            raise ValueError(f"Cannot pick from {station.role} station {station.name}")
        if placement_slot_id != 0:
            raise ValueError("PICK does not accept placement_slot_id")
        pose = (
            station.source_selection_pose
            if station.role == "source"
            else station.machine_workpiece_pose
        )
    elif operation == PLACE_OPERATION:
        if station.role not in {"sink", "machine"}:
            raise ValueError(f"Cannot place into {station.role} station {station.name}")
        if station.role == "sink":
            if placement_slot_id < 1:
                raise ValueError(
                    f"Placement at {station.name} requires placement_slot_id"
                )
            placement = station.require_placement_slot(placement_slot_id)
            return station, ManipulationTarget(
                pose=placement.pose,
                placement_slot_id=placement.slot_id,
            )
        if placement_slot_id != 0:
            raise ValueError("CNC PLACE does not accept placement_slot_id")
        pose = station.machine_workpiece_pose
    else:
        raise ValueError(f"Unsupported manipulation operation: {operation}")
    if pose is None:
        raise ValueError(
            f"Station {station.name} has no geometry for this operation"
        )
    return station, ManipulationTarget(pose=pose)


def load_manipulation_stations(
    config_path: str | Path,
) -> dict[str, ManipulationStation]:
    """Load role-specific manipulation geometry and reject ambiguity."""
    path = Path(config_path)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    station_entries = (
        document.get("stations") if isinstance(document, dict) else None
    )
    if not isinstance(station_entries, dict) or not station_entries:
        raise ValueError(f"No manipulation stations defined in {path}")

    stations: dict[str, ManipulationStation] = {}
    for name, station_data in station_entries.items():
        if not isinstance(station_data, dict):
            raise ValueError(f"Station {name} must be a mapping")
        role = str(station_data.get("role", ""))
        if role not in VALID_STATION_ROLES:
            raise ValueError(
                f"Station {name} role must be one of {sorted(VALID_STATION_ROLES)}"
            )
        approach_offset = _load_motion_offset(
            station_data.get("approach_offset"), f"{name}.approach_offset"
        )
        lift_offset = _load_motion_offset(
            station_data.get("lift_offset"), f"{name}.lift_offset"
        )
        grasp_offset = _load_grasp_offset(
            station_data.get("grasp_offset"), f"{name}.grasp_offset"
        )
        fixture_reference = _load_fixture_reference(
            station_data.get("fixture_reference"),
            station_name=name,
            station_role=role,
        )

        clearance_pose_data = station_data.get("clearance_pose")
        clearance_pose = (
            None
            if clearance_pose_data is None
            else _load_pose(clearance_pose_data, f"{name}.clearance_pose")
        )
        source_selection_pose = _optional_role_pose(
            station_data.get("source_selection_pose"),
            field_name=f"{name}.source_selection_pose",
            station_name=name,
            station_role=role,
            required_role="source",
        )
        machine_workpiece_pose = _optional_role_pose(
            station_data.get("machine_workpiece_pose"),
            field_name=f"{name}.machine_workpiece_pose",
            station_name=name,
            station_role=role,
            required_role="machine",
        )
        placement_slots = _load_placement_slots(
            name,
            station_data.get("placement_slots"),
            required=role == "sink",
        )
        _validate_role_geometry(
            name,
            role,
            source_selection_pose,
            machine_workpiece_pose,
            placement_slots,
        )
        stations[name] = ManipulationStation(
            name=name,
            role=role,
            approach_offset=approach_offset,
            lift_offset=lift_offset,
            grasp_offset=grasp_offset,
            fixture_reference=fixture_reference,
            clearance_pose=clearance_pose,
            source_selection_pose=source_selection_pose,
            machine_workpiece_pose=machine_workpiece_pose,
            placement_slots=placement_slots,
        )
    return stations


def _load_fixture_reference(
    value: object,
    *,
    station_name: str,
    station_role: str,
) -> StationFixtureReference | None:
    """Load an optional perception frame used for final CNC alignment."""
    if value is None:
        return None
    if station_role != "machine":
        raise ValueError(
            f"{station_name}.fixture_reference is only valid for a machine"
        )
    if not isinstance(value, dict):
        raise ValueError(f"{station_name}.fixture_reference must be a mapping")

    frame_id = value.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError(
            f"{station_name}.fixture_reference needs a frame_id"
        )
    position = _number_tuple(
        value.get("position"),
        3,
        f"{station_name}.fixture_reference.position",
    )
    return StationFixtureReference(frame_id=frame_id, position=position)


def _optional_role_pose(
    value: object,
    *,
    field_name: str,
    station_name: str,
    station_role: str,
    required_role: str,
) -> CartesianPose | None:
    if value is None:
        return None
    if station_role != required_role:
        raise ValueError(
            f"{station_name}.{field_name.rsplit('.', 1)[-1]} is only valid "
            f"for a {required_role} station"
        )
    return _load_pose(value, field_name)


def _load_placement_slots(
    station_name: str,
    entries: object,
    *,
    required: bool,
) -> dict[int, PlacementSlot]:
    if entries is None and not required:
        return {}
    if not isinstance(entries, list) or not entries:
        raise ValueError(
            f"Station {station_name} needs non-empty placement_slots"
        )

    slots: dict[int, PlacementSlot] = {}
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError(
                f"Station {station_name} placement slot must be a mapping"
            )
        slot_id = entry.get("slot_id")
        if isinstance(slot_id, bool) or not isinstance(slot_id, int) or slot_id < 1:
            raise ValueError(
                f"Station {station_name} placement slot_id must be positive"
            )
        if slot_id in slots:
            raise ValueError(
                f"Station {station_name} repeats placement slot {slot_id}"
            )

        slots[slot_id] = PlacementSlot(
            slot_id=slot_id,
            pose=_load_pose(
                entry.get("pose"),
                f"{station_name}.placement_slots[{slot_id}].pose",
            ),
        )
    return slots


def _validate_role_geometry(
    station_name: str,
    role: str,
    source_selection_pose: CartesianPose | None,
    machine_workpiece_pose: CartesianPose | None,
    placement_slots: dict[int, PlacementSlot],
) -> None:
    if role == "source" and source_selection_pose is None:
        raise ValueError(
            f"Station {station_name} needs source_selection_pose"
        )
    if role == "machine" and machine_workpiece_pose is None:
        raise ValueError(
            f"Station {station_name} needs machine_workpiece_pose"
        )
    if role != "sink" and placement_slots:
        raise ValueError(
            f"Station {station_name} placement_slots are only valid for a sink"
        )


def _load_pose(pose_data: object, field_name: str) -> CartesianPose:
    if not isinstance(pose_data, dict):
        raise ValueError(f"{field_name} must be a mapping")
    frame_id = pose_data.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError(f"{field_name} needs frame_id")

    position = _number_tuple(
        pose_data.get("position"), 3, f"{field_name}.position"
    )
    orientation = _number_tuple(
        pose_data.get("orientation"), 4, f"{field_name}.orientation"
    )
    quaternion_norm = math.sqrt(sum(value * value for value in orientation))
    if not math.isclose(quaternion_norm, 1.0, abs_tol=1e-3):
        raise ValueError(f"{field_name}.orientation must be a unit quaternion")
    return CartesianPose(frame_id, position, orientation)


def _load_motion_offset(
    value: object, field_name: str
) -> tuple[float, float, float]:
    offset = _number_tuple(value, 3, field_name)
    magnitude = math.sqrt(sum(component * component for component in offset))
    if not 0.05 <= magnitude <= 0.50:
        raise ValueError(
            f"{field_name} magnitude must be between 0.05 and 0.50 m"
        )
    return offset


def _load_grasp_offset(
    value: object, field_name: str
) -> tuple[float, float, float]:
    """Load the TCP offset from a part center to its actual contact height."""
    offset = _number_tuple(value, 3, field_name)
    magnitude = math.sqrt(sum(component * component for component in offset))
    if magnitude > 0.10:
        raise ValueError(f"{field_name} magnitude must not exceed 0.10 m")
    return offset


def _number_tuple(value: object, size: int, field_name: str) -> tuple:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{field_name} needs {size} numbers")
    return tuple(_finite_number(item, field_name) for item in value)


def _finite_number(value: object, field_name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field_name} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field_name} must be numeric") from error
    if not math.isfinite(result):
        raise ValueError(f"{field_name} must be finite")
    return result
