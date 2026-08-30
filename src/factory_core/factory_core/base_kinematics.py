"""Pure planar-motion helpers used by the base calibration check."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class LocalMotion:
    forward: float
    lateral: float
    yaw: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def local_motion(start: PlanarPose, end: PlanarPose) -> LocalMotion:
    """Express an end pose as motion in the start pose's local frame."""
    delta_x = end.x - start.x
    delta_y = end.y - start.y
    cosine = math.cos(start.yaw)
    sine = math.sin(start.yaw)
    return LocalMotion(
        forward=cosine * delta_x + sine * delta_y,
        lateral=-sine * delta_x + cosine * delta_y,
        yaw=normalize_angle(end.yaw - start.yaw),
    )


def motion_error(reference: LocalMotion, measured: LocalMotion) -> LocalMotion:
    return LocalMotion(
        forward=measured.forward - reference.forward,
        lateral=measured.lateral - reference.lateral,
        yaw=normalize_angle(measured.yaw - reference.yaw),
    )


def is_planar_motion_settled(
    velocity_x: float,
    velocity_y: float,
    angular_z: float,
    *,
    linear_tolerance: float,
    angular_tolerance: float,
) -> bool:
    """Return whether a base is stationary enough for arm-frame planning."""
    if linear_tolerance < 0.0 or angular_tolerance < 0.0:
        raise ValueError("base settle tolerances must be non-negative")
    return (
        math.hypot(velocity_x, velocity_y) <= linear_tolerance
        and abs(angular_z) <= angular_tolerance
    )
