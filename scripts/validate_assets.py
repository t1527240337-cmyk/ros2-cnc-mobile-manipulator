#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import py_compile
import sys
import xml.etree.ElementTree as element_tree

import yaml


XACRO_NAMESPACE = "{http://www.ros.org/wiki/xacro}"
PASSIVE_CASTER_JOINTS = {
    "front_caster_swivel_joint",
    "front_caster_wheel_joint",
    "rear_caster_swivel_joint",
    "rear_caster_wheel_joint",
}


def _vector(text: str | None) -> tuple[float, float, float]:
    if text is None:
        return (0.0, 0.0, 0.0)
    values = tuple(float(value) for value in text.split())
    if len(values) != 3:
        raise ValueError(f"expected a three-element vector, got {text!r}")
    return values


def _box_bounds(
    visual: element_tree.Element,
    parent_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    box = visual.find("geometry/box")
    if box is None:
        raise ValueError(f"{visual.get('name')} must use a box geometry")
    size = _vector(box.get("size"))
    origin = _vector(
        visual.find("origin").get("xyz")
        if visual.find("origin") is not None
        else None
    )
    center = tuple(origin[index] + parent_offset[index] for index in range(3))
    return tuple(
        (center[index] - size[index] / 2.0, center[index] + size[index] / 2.0)
        for index in range(3)
    )


def _boxes_touch(
    first: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    second: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
) -> bool:
    tolerance = 1.0e-6
    return all(
        first[axis][1] + tolerance >= second[axis][0]
        and second[axis][1] + tolerance >= first[axis][0]
        for axis in range(3)
    )


def validate_sensor_mount_is_continuous(robot_description: pathlib.Path) -> None:
    root = element_tree.parse(robot_description).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    sensor_mast = links["sensor_mast"]
    base_link = links["base_link"]

    mast_visuals = {
        visual.get("name"): visual for visual in sensor_mast.findall("visual")
    }
    base_visuals = {
        visual.get("name"): visual for visual in base_link.findall("visual")
    }
    column = _box_bounds(mast_visuals["sensor_mast_column"])
    flange = _box_bounds(mast_visuals["mounting_flange"])
    boom = _box_bounds(mast_visuals["tag_camera_boom"])
    if not _boxes_touch(column, flange):
        raise ValueError("sensor mast column is detached from its mounting flange")
    if not _boxes_touch(column, boom):
        raise ValueError("station-camera boom is detached from the sensor mast")

    mast_joint = next(
        joint for joint in root.findall("joint")
        if joint.get("name") == "sensor_mast_joint"
    )
    mast_offset = _vector(mast_joint.find("origin").get("xyz"))
    deck = _box_bounds(base_visuals["upper_deck"])
    flange_on_base = _box_bounds(
        mast_visuals["mounting_flange"], parent_offset=mast_offset
    )
    if not _boxes_touch(deck, flange_on_base):
        raise ValueError("sensor mounting flange is detached from the upper deck")


def validate_passive_caster_state_interfaces(
    robot_description: pathlib.Path,
) -> None:
    root = element_tree.parse(robot_description).getroot()
    ros2_control = root.find("ros2_control")
    if ros2_control is None:
        raise ValueError("robot description has no ros2_control system")

    configured = {
        element.get("name")
        for element in ros2_control.findall(
            f"{XACRO_NAMESPACE}passive_state_joint"
        )
    }
    missing = PASSIVE_CASTER_JOINTS - configured
    if missing:
        raise ValueError(
            "passive caster state interfaces missing for "
            + ", ".join(sorted(missing))
        )


def main() -> int:
    root = pathlib.Path(__file__).resolve().parents[1]
    source = root / "src"
    python_files = list(source.rglob("*.py"))
    xml_files = (
        list(source.rglob("*.xml"))
        + list(source.rglob("*.xacro"))
        + list(source.rglob("*.sdf"))
    )
    yaml_files = list(source.rglob("*.yaml"))

    for path in python_files:
        py_compile.compile(str(path), doraise=True)
    for path in xml_files:
        element_tree.parse(path)
    for path in yaml_files:
        yaml.safe_load(path.read_text(encoding="utf-8"))

    robot_description = (
        source / "mobile_manipulator_description" / "urdf"
        / "mobile_manipulator.urdf.xacro"
    )
    validate_passive_caster_state_interfaces(robot_description)
    validate_sensor_mount_is_continuous(robot_description)

    pgm = source / "factory_bringup" / "maps" / "factory_map.pgm"
    rows = [
        line
        for line in pgm.read_text(encoding="ascii").splitlines()
        if not line.startswith("#")
    ]
    tokens = " ".join(rows).split()
    width, height = int(tokens[1]), int(tokens[2])
    pixels = tokens[4:]
    if len(pixels) != width * height:
        raise ValueError(f"PGM has {len(pixels)} pixels; expected {width * height}")

    station_document = yaml.safe_load(
        (source / "factory_core" / "config" / "stations.yaml").read_text()
    )
    nav2_document = yaml.safe_load(
        (source / "factory_bringup" / "config" / "nav2_params.yaml").read_text()
    )
    docking = nav2_document["docking_server"]["ros__parameters"]
    for name, station in station_document["stations"].items():
        configured = docking[name]["pose"]
        expected = station["dock_pose"]
        if configured != expected:
            raise ValueError(
                f"Dock pose mismatch for {name}: stations={expected}, nav2={configured}"
            )

    print(
        f"python_ok={len(python_files)} xml_ok={len(xml_files)} "
        f"yaml_ok={len(yaml_files)} pgm_ok={width}x{height}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
