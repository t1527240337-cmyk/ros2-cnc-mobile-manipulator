import math

import pytest

from factory_core.base_kinematics import PlanarPose
from factory_core.gazebo_truth_odometry import body_velocity, relative_pose


def test_relative_pose_removes_world_spawn_pose():
    origin = PlanarPose(2.0, -1.0, math.pi / 2.0)
    current = PlanarPose(2.0, 1.0, math.pi)
    result = relative_pose(origin, current)
    assert (result.x, result.y, result.yaw) == pytest.approx(
        (2.0, 0.0, math.pi / 2.0)
    )


def test_body_velocity_is_expressed_in_current_base_frame():
    previous = PlanarPose(0.0, 0.0, math.pi / 2.0)
    current = PlanarPose(0.0, 0.2, math.pi / 2.0)
    assert body_velocity(previous, current, 0.5) == pytest.approx(
        (0.4, 0.0, 0.0)
    )


def test_body_velocity_wraps_yaw_delta():
    previous = PlanarPose(0.0, 0.0, math.pi - 0.1)
    current = PlanarPose(0.0, 0.0, -math.pi + 0.1)
    assert body_velocity(previous, current, 0.5)[2] == pytest.approx(0.4)
