"""Small, testable helpers for planar mobile-base velocity commands."""

from __future__ import annotations

import math


def body_linear_to_world(
    body_x: float, body_y: float, yaw: float
) -> tuple[float, float]:
    """Rotate a body-frame planar velocity into the Gazebo world frame."""

    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * body_x - sine * body_y,
        sine * body_x + cosine * body_y,
    )


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a normalized orientation quaternion."""

    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )

def planar_distance(
    first: tuple[float, float], second: tuple[float, float]
) -> float:
    """Return Euclidean displacement between two planar positions."""

    return math.hypot(second[0] - first[0], second[1] - first[1])
