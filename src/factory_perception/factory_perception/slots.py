from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Slot:
    slot_id: int
    center_u: int
    center_v: int
    half_size: int
    tray_x: float
    tray_y: float
    plane_depth: float | None = None


@dataclass(frozen=True)
class PinholeIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class SlotTarget:
    """A slot sample point expressed in the detector output frame."""

    slot_id: int
    x: float
    y: float
    sample_z: float


@dataclass(frozen=True)
class SlotObservation:
    slot_id: int
    occupied: bool
    median_depth: float
    height_above_tray: float
    valid_fraction: float
    tray_x: float
    tray_y: float
    observable: bool = True


def project_slot(
    target: SlotTarget,
    intrinsics: PinholeIntrinsics,
    camera_from_output_rotation: np.ndarray,
    camera_from_output_translation: tuple[float, float, float],
    *,
    surface_z: float,
    image_shape: tuple[int, int],
    half_size: int,
) -> Slot | None:
    """Project one known tray slot and predict its empty-surface depth."""

    rotation = np.asarray(camera_from_output_rotation, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("camera rotation must be 3x3")
    if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    if half_size < 1:
        raise ValueError("half_size must be positive")

    translation = np.asarray(camera_from_output_translation, dtype=float)
    sample = np.asarray((target.x, target.y, target.sample_z), dtype=float)
    sample_camera = rotation @ sample + translation
    if sample_camera[2] <= 0.0:
        return None

    u = intrinsics.fx * sample_camera[0] / sample_camera[2] + intrinsics.cx
    v = intrinsics.fy * sample_camera[1] / sample_camera[2] + intrinsics.cy
    height, width = image_shape
    if not 0.0 <= u < width or not 0.0 <= v < height:
        return None

    camera_origin = -(rotation.T @ translation)
    ray_camera = np.asarray(
        (
            (u - intrinsics.cx) / intrinsics.fx,
            (v - intrinsics.cy) / intrinsics.fy,
            1.0,
        )
    )
    ray_output = rotation.T @ ray_camera
    if abs(ray_output[2]) < 1.0e-9:
        return None
    scale = (surface_z - camera_origin[2]) / ray_output[2]
    if scale <= 0.0:
        return None
    surface_point = camera_origin + scale * ray_output
    surface_camera = rotation @ surface_point + translation
    if surface_camera[2] <= 0.0:
        return None

    return Slot(
        slot_id=target.slot_id,
        center_u=int(round(u)),
        center_v=int(round(v)),
        half_size=half_size,
        tray_x=target.x,
        tray_y=target.y,
        plane_depth=float(surface_camera[2]),
    )


def detect_slots(
    depth_m: np.ndarray,
    slots: list[Slot],
    tray_plane_depth: float,
    minimum_height: float = 0.02,
    minimum_valid_fraction: float = 0.5,
) -> list[SlotObservation]:
    if depth_m.ndim != 2:
        raise ValueError("depth image must be a 2-D array")
    observations: list[SlotObservation] = []
    height, width = depth_m.shape
    for slot in slots:
        u0, u1 = max(0, slot.center_u - slot.half_size), min(width, slot.center_u + slot.half_size + 1)
        v0, v1 = max(0, slot.center_v - slot.half_size), min(height, slot.center_v + slot.half_size + 1)
        roi = depth_m[v0:v1, u0:u1]
        valid = np.isfinite(roi) & (roi > 0.0)
        valid_fraction = float(valid.mean()) if roi.size else 0.0
        median = float(np.median(roi[valid])) if valid.any() else float("nan")
        plane_depth = (
            slot.plane_depth
            if slot.plane_depth is not None
            else tray_plane_depth
        )
        object_height = (
            max(0.0, plane_depth - median)
            if np.isfinite(median)
            else 0.0
        )
        occupied = valid_fraction >= minimum_valid_fraction and object_height >= minimum_height
        observations.append(SlotObservation(
            slot.slot_id, occupied, median, object_height, valid_fraction, slot.tray_x, slot.tray_y
        ))
    return observations



def detect_slots_in_point_cloud(
    depth_m: np.ndarray,
    intrinsics: PinholeIntrinsics,
    camera_from_output_rotation: np.ndarray,
    camera_from_output_translation: tuple[float, float, float],
    targets: tuple[SlotTarget, ...],
    *,
    surface_z: float,
    slot_half_size: float,
    minimum_height: float,
    maximum_height: float,
    minimum_region_points: int,
    minimum_object_points: int,
) -> list[SlotObservation]:
    """Classify taught slots from points expressed in the output frame.

    A 2-D median window fails when a small workpiece covers less than half of
    its pixels and mistakes any foreground occluder for stock. This detector
    instead back-projects valid depth pixels, transforms them into the surveyed
    tray frame and accepts only points inside the physical slot volume.
    """

    if depth_m.ndim != 2:
        raise ValueError("depth image must be a 2-D array")
    if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    rotation = np.asarray(camera_from_output_rotation, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("camera rotation must be 3x3")
    if slot_half_size <= 0.0:
        raise ValueError("slot_half_size must be positive")
    if not 0.0 < minimum_height < maximum_height:
        raise ValueError("height limits must satisfy 0 < minimum < maximum")
    if minimum_region_points < 1 or minimum_object_points < 1:
        raise ValueError("point-count thresholds must be positive")

    rows, columns = np.indices(depth_m.shape, dtype=np.float32)
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    camera_depth = depth_m[valid].astype(np.float64, copy=False)
    camera_x = (
        (columns[valid] - intrinsics.cx) * camera_depth / intrinsics.fx
    )
    camera_y = (
        (rows[valid] - intrinsics.cy) * camera_depth / intrinsics.fy
    )
    camera_points = np.column_stack((camera_x, camera_y, camera_depth))

    translation = np.asarray(camera_from_output_translation, dtype=float)
    output_points = (
        rotation.T @ (camera_points - translation).T
    ).T
    output_x = output_points[:, 0]
    output_y = output_points[:, 1]
    output_z = output_points[:, 2]
    lower_z = surface_z - 0.02
    upper_z = surface_z + maximum_height

    observations: list[SlotObservation] = []
    for target in targets:
        in_column = (
            (np.abs(output_x - target.x) <= slot_half_size)
            & (np.abs(output_y - target.y) <= slot_half_size)
            & (output_z >= lower_z)
            & (output_z <= upper_z)
        )
        region_count = int(np.count_nonzero(in_column))
        elevated = in_column & (output_z >= surface_z + minimum_height)
        object_count = int(np.count_nonzero(elevated))
        observable = region_count >= minimum_region_points
        occupied = observable and object_count >= minimum_object_points
        median_depth = (
            float(np.median(camera_depth[in_column]))
            if region_count
            else float("nan")
        )
        object_height = (
            float(np.percentile(output_z[elevated], 75) - surface_z)
            if object_count
            else 0.0
        )
        observations.append(
            SlotObservation(
                slot_id=target.slot_id,
                occupied=occupied,
                median_depth=median_depth,
                height_above_tray=object_height,
                valid_fraction=min(1.0, region_count / minimum_region_points),
                tray_x=target.x,
                tray_y=target.y,
                observable=observable,
            )
        )
    return observations
