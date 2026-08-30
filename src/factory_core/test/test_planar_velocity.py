import math

import pytest

from factory_core.planar_velocity import (
    body_linear_to_world,
    planar_distance,
    quaternion_yaw,
)


@pytest.mark.parametrize(
    ("yaw", "expected"),
    [
        (0.0, (1.0, 0.0)),
        (math.pi / 2.0, (0.0, 1.0)),
        (math.pi, (-1.0, 0.0)),
    ],
)
def test_body_forward_rotates_into_world(yaw, expected):
    result = body_linear_to_world(1.0, 0.0, yaw)
    assert result == pytest.approx(expected, abs=1e-9)


def test_quaternion_yaw_round_trip():
    yaw = -1.2
    assert quaternion_yaw(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)) \
        == pytest.approx(yaw)


def test_planar_distance_is_independent_of_heading():
    assert planar_distance((1.0, -2.0), (1.3, -1.6)) == pytest.approx(0.5)
