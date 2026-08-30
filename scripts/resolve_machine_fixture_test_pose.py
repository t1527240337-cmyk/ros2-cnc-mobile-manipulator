#!/usr/bin/env python3
"""Resolve an isolated CNC test fixture through the production calibration.

This helper is test setup only.  It transforms the configured AprilTag datum
into the map frame so an isolated workpiece starts exactly where a completed
production load would leave it.  Runtime manipulation never calls Gazebo pose
services and never uses this helper.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from factory_core.manipulation_config import load_manipulation_stations
from factory_core.manipulate_part_server import (
    Point3,
    resolve_machine_target_position,
)


def rotate_point(
    point: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Rotate a point by a normalized xyzw quaternion."""
    px, py, pz = point
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise RuntimeError("TF supplied a zero-length quaternion")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))

    # q * p * q^-1, expanded to avoid an extra geometry dependency.
    tx = 2.0 * (qy * pz - qz * py)
    ty = 2.0 * (qz * px - qx * pz)
    tz = 2.0 * (qx * py - qy * px)
    return (
        px + qw * tx + qy * tz - qz * ty,
        py + qw * ty + qz * tx - qx * tz,
        pz + qw * tz + qx * ty - qy * tx,
    )


def transform_point(point, transform) -> tuple[float, float, float]:
    rotation = transform.transform.rotation
    translated = rotate_point(
        tuple(float(value) for value in point),
        (rotation.x, rotation.y, rotation.z, rotation.w),
    )
    origin = transform.transform.translation
    return (
        translated[0] + origin.x,
        translated[1] + origin.y,
        translated[2] + origin.z,
    )


def wait_for_transform(
    node: Node,
    buffer: Buffer,
    target: str,
    source: str,
    timeout_sec: float,
):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buffer.can_transform(target, source, Time()):
            return buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=0.5),
            )
    raise RuntimeError(f"no fresh transform from {source} to {target}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("station")
    parser.add_argument("--timeout", type=float, default=8.0)
    args, ros_args = parser.parse_known_args()

    config_path = (
        Path(get_package_share_directory("factory_core"))
        / "config"
        / "manipulation.yaml"
    )
    stations = load_manipulation_stations(config_path)
    station = stations.get(args.station)
    if station is None or station.role != "machine":
        raise RuntimeError(f"{args.station} is not a configured machine")
    if station.fixture_reference is None:
        raise RuntimeError(f"{args.station} has no fixture calibration")
    workpiece_pose = station.machine_workpiece_pose
    if workpiece_pose is None:
        raise RuntimeError(f"{args.station} has no CNC workpiece pose")

    rclpy.init(args=ros_args)
    node = Node("resolve_machine_fixture_test_pose")
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node, spin_thread=False)
    try:
        base_from_tag = wait_for_transform(
            node,
            buffer,
            "base_link",
            station.fixture_reference.frame_id,
            args.timeout,
        )
        observed = transform_point(
            station.fixture_reference.position,
            base_from_tag,
        )
        target_in_base = resolve_machine_target_position(
            Point3(*observed),
            workpiece_pose.position,
        )
        map_from_base = wait_for_transform(
            node, buffer, "map", "base_link", args.timeout
        )
        target_in_map = transform_point(target_in_base, map_from_base)
        print("{:.6f} {:.6f} {:.6f}".format(*target_in_map))
        return 0
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"fixture pose resolution failed: {error}", file=sys.stderr)
        raise SystemExit(1)
