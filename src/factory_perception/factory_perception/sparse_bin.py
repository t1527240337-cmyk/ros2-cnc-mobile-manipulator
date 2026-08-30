"""Geometry-only RGB-D helpers for a sparse, single-layer parts bin.

The detector deliberately avoids object IDs. Parts in a real raw-material bin
are interchangeable before they are picked; identity is assigned only after a
candidate has been reserved.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera parameters expressed in pixels."""

    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self) -> None:
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("camera focal lengths must be positive")


@dataclass(frozen=True)
class Region3D:
    """Axis-aligned work volume expressed in the output frame."""

    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]

    def validate(self) -> None:
        if any(
            lower >= upper
            for lower, upper in zip(self.minimum, self.maximum, strict=True)
        ):
            raise ValueError("every region minimum must be below its maximum")


@dataclass(frozen=True)
class PartCandidate:
    """One separated workpiece observation."""

    x: float
    y: float
    z: float
    pixel_count: int


def quaternion_rotation_matrix(
    x: float, y: float, z: float, w: float
) -> np.ndarray:
    """Return a 3x3 rotation matrix for a normalized quaternion."""
    norm = float(np.linalg.norm((x, y, z, w)))
    if norm <= 1e-9:
        raise ValueError("transform quaternion must be non-zero")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def fit_upright_cylinder_center(
    points: np.ndarray,
    *,
    expected_radius: float,
    radius_tolerance: float,
) -> tuple[float, float] | None:
    """Fit an upright cylinder axis from its visible side-wall points.

    A depth image measures the front surface, not the cylinder axis.  Fitting
    the horizontal circle removes that view-dependent bias.  The upper
    quartile is excluded because a camera mounted above the workpiece may also
    see its flat top cap, whose points fill a disk instead of lying on a circle.
    """

    samples = np.asarray(points, dtype=np.float64)
    if samples.ndim != 2 or samples.shape[1] != 3:
        raise ValueError("cylinder points must have shape (N, 3)")
    if expected_radius <= 0.0 or radius_tolerance <= 0.0:
        raise ValueError("cylinder radius and tolerance must be positive")
    if len(samples) < 12:
        return None

    side_wall = samples[
        samples[:, 2] <= np.percentile(samples[:, 2], 75.0)
    ]
    if len(side_wall) < 9:
        return None
    horizontal = side_wall[:, :2]
    origin = np.mean(horizontal, axis=0)
    local = horizontal - origin
    design = np.column_stack(
        (2.0 * local[:, 0], 2.0 * local[:, 1], np.ones(len(local)))
    )
    squared_distance = np.sum(local * local, axis=1)
    solution, _, rank, singular_values = np.linalg.lstsq(
        design, squared_distance, rcond=None
    )
    if rank < 3 or singular_values[-1] <= 1e-9:
        return None
    center_local = solution[:2]
    radius_squared = solution[2] + float(center_local @ center_local)
    if radius_squared <= 0.0:
        return None
    fitted_radius = float(np.sqrt(radius_squared))
    if abs(fitted_radius - expected_radius) > radius_tolerance:
        return None
    center = origin + center_local
    return float(center[0]), float(center[1])


def detect_sparse_parts(
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    rotation_to_output: np.ndarray,
    translation_to_output: Iterable[float],
    region: Region3D,
    *,
    minimum_component_pixels: int = 20,
    maximum_component_span: float = 0.14,
    maximum_candidates: int = 6,
    upright_cylinder_radius: float | None = None,
    supported_center_height: float | None = None,
    cylinder_radius_tolerance: float = 0.01,
) -> list[PartCandidate]:
    """Back-project depth and return separated candidates inside ``region``.

    Sparse-bin mode assumes upright workpieces in one layer with no severe
    image overlap. Connected image components are sufficient for this bounded
    MVP and make the failure explicit: touching or stacked parts are rejected
    instead of producing an unsafe grasp.
    """
    if depth_m.ndim != 2:
        raise ValueError("depth image must be a 2-D array")
    intrinsics.validate()
    region.validate()
    if rotation_to_output.shape != (3, 3):
        raise ValueError("rotation_to_output must be a 3x3 matrix")
    if minimum_component_pixels < 1:
        raise ValueError("minimum_component_pixels must be positive")
    if maximum_component_span <= 0.0:
        raise ValueError("maximum_component_span must be positive")
    if maximum_candidates < 1:
        raise ValueError("maximum_candidates must be positive")
    if upright_cylinder_radius is not None:
        if upright_cylinder_radius <= 0.0:
            raise ValueError("upright_cylinder_radius must be positive")
        if cylinder_radius_tolerance <= 0.0:
            raise ValueError("cylinder_radius_tolerance must be positive")
    if supported_center_height is not None and not (
        region.minimum[2] <= supported_center_height <= region.maximum[2]
    ):
        raise ValueError(
            "supported_center_height must lie inside the detection region"
        )

    translation = np.asarray(tuple(translation_to_output), dtype=np.float64)
    if translation.shape != (3,):
        raise ValueError("translation_to_output must contain three values")

    depth = np.asarray(depth_m, dtype=np.float64)
    rows, columns = np.indices(depth.shape, dtype=np.float64)
    valid = np.isfinite(depth) & (depth > 0.05)

    camera_points = np.empty((*depth.shape, 3), dtype=np.float64)
    camera_points[..., 0] = (
        (columns - intrinsics.cx) * depth / intrinsics.fx
    )
    camera_points[..., 1] = (rows - intrinsics.cy) * depth / intrinsics.fy
    camera_points[..., 2] = depth
    output_points = camera_points @ rotation_to_output.T + translation

    lower = np.asarray(region.minimum, dtype=np.float64)
    upper = np.asarray(region.maximum, dtype=np.float64)
    inside = valid & np.all(output_points >= lower, axis=2)
    inside &= np.all(output_points <= upper, axis=2)

    candidates: list[PartCandidate] = []
    for component in _connected_components(inside):
        if len(component) < minimum_component_pixels:
            continue
        component_rows, component_columns = zip(*component, strict=True)
        points = output_points[component_rows, component_columns]
        span = np.ptp(points[:, :2], axis=0)
        if float(max(span)) > maximum_component_span:
            # A merged silhouette is not a safe single-object candidate.
            continue

        # Generic sparse-bin observations use the silhouette midpoint. For a
        # known upright cylinder, recover its actual axis from the side wall;
        # a front-surface centroid would bias a CNC grasp toward the camera.
        if upright_cylinder_radius is None:
            horizontal_min = np.min(points[:, :2], axis=0)
            horizontal_max = np.max(points[:, :2], axis=0)
            horizontal_center = (horizontal_min + horizontal_max) / 2.0
        else:
            fitted_center = fit_upright_cylinder_center(
                points,
                expected_radius=upright_cylinder_radius,
                radius_tolerance=cylinder_radius_tolerance,
            )
            if fitted_center is None:
                continue
            horizontal_center = np.asarray(fitted_center)
        if supported_center_height is None:
            center_height = float(np.median(points[:, 2]))
        else:
            # The single-layer bin constrains upright stock to one surveyed
            # support plane. Occlusion changes visible depth, not centre z.
            center_height = supported_center_height
        candidates.append(
            PartCandidate(
                x=float(horizontal_center[0]),
                y=float(horizontal_center[1]),
                z=center_height,
                pixel_count=len(component),
            )
        )

    candidates.sort(key=lambda candidate: (-candidate.y, candidate.x))
    return candidates[:maximum_candidates]


def _connected_components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    """Return eight-connected components without an OpenCV dependency."""
    pixels = {
        (int(row), int(column)) for row, column in np.argwhere(mask)
    }
    components: list[list[tuple[int, int]]] = []
    while pixels:
        seed = pixels.pop()
        component = [seed]
        pending = [seed]
        while pending:
            row, column = pending.pop()
            for row_delta in (-1, 0, 1):
                for column_delta in (-1, 0, 1):
                    if row_delta == 0 and column_delta == 0:
                        continue
                    neighbour = (row + row_delta, column + column_delta)
                    if neighbour not in pixels:
                        continue
                    pixels.remove(neighbour)
                    component.append(neighbour)
                    pending.append(neighbour)
        components.append(component)
    return components
