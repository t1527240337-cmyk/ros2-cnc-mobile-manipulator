"""Explicit MoveIt collision geometry for factory manipulation."""

from __future__ import annotations

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


GRIPPER_TOUCH_LINKS = (
    "gripper_tcp",
    "gripper_parallel_base_link",
    "gripper_left_finger_tip_link",
    "gripper_right_finger_tip_link",
    "gripper_grasp_anchor",
)


WORKPIECE_HEIGHT = 0.12
WORKPIECE_RADIUS = 0.025

# Derived from the calibrated tag target and rigid fixture offsets.
# DART settles base_footprint at world z=0.04 m; base_link is another 0.34 m
# higher. The 0.554 m table top is therefore z=0.174 m in base_link.
_RAW_BIN_TABLE_POSITION = (1.182, 0.0, -0.076)
_RAW_BIN_TABLE_SIZE = (1.20, 0.90, 0.50)
_RAW_BIN_TAG_POSITION = (0.602, -0.55, 0.374)
_RAW_BIN_TAG_SIZE = (0.06, 0.10, 0.40)

# The finished-bin tag target differs, so derive its geometry separately.
# Table centre is tag_x + 0.612 m; support centre is tag_x + 0.032 m.
_FINISHED_BIN_TABLE_POSITION = (1.262, 0.0, -0.076)
_FINISHED_BIN_TAG_POSITION = (0.682, -0.55, 0.374)

# Collision boxes are expressed in base_link after a successful CNC dock.
# They intentionally model the access aperture and fixture, not decorative
# panels. The open sliding door is included because loaded OMPL motion must
# not sweep the attached workpiece through the panel parked at the side.
MACHINE_COLLISION_BOXES = (
    ("machine_base_cabinet", (1.50, 0.0, -0.010), (1.80, 1.55, 0.70)),
    ("machine_rear_wall", (2.34, 0.0, 1.090), (0.12, 1.55, 1.50)),
    ("machine_chip_pan", (1.45, 0.0, 0.510), (1.45, 1.24, 0.10)),
    ("machine_work_table", (1.10, 0.0, 0.650), (0.90, 0.72, 0.16)),
    ("machine_fixture_base", (0.95, 0.0, 0.800), (0.40, 0.38, 0.12)),
    ("machine_front_left_post", (0.64, 0.65, 1.110), (0.10, 0.18, 1.42)),
    ("machine_front_right_post", (0.64, -0.65, 1.110), (0.10, 0.18, 1.42)),
    ("machine_front_lintel", (0.64, 0.0, 1.720), (0.10, 1.18, 0.22)),
    ("machine_open_door", (0.56, 1.18, 1.110), (0.055, 1.13, 1.16)),
    ("machine_left_wall", (1.50, 0.715, 1.090), (1.80, 0.12, 1.50)),
    ("machine_right_wall", (1.50, -0.715, 1.090), (1.80, 0.12, 1.50)),
    ("machine_roof", (1.50, 0.0, 1.870), (1.80, 1.55, 0.12)),
    ("machine_spindle_head", (2.06, 0.0, 1.190), (0.46, 0.52, 0.58)),
    ("machine_spindle_nose", (1.75, 0.0, 1.080), (0.22, 0.21, 0.21)),
    ("machine_cutting_tool", (1.485, 0.0, 1.080), (0.23, 0.06, 0.06)),
)

MACHINE_FIXTURE_CONTACT_OBJECTS = (
    "machine_fixture_base",
)
FINISHED_BIN_SUPPORT_OBJECTS = (
    "finished_bin_table",
)


class PlanningSceneClient:
    """Keep MoveIt's station and workpiece geometry consistent with Gazebo."""

    def __init__(
        self,
        node: Node,
        *,
        service_name: str = "/apply_planning_scene",
        tool_frame: str = "gripper_tcp",
    ) -> None:
        self._client = node.create_client(ApplyPlanningScene, service_name)
        self._scene_reader = node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self._tool_frame = tool_frame
        self._active_station_object_ids: set[str] = set()

    def wait_until_ready(self, timeout_sec: float = 5.0) -> bool:
        return (
            self._client.wait_for_service(timeout_sec=timeout_sec)
            and self._scene_reader.wait_for_service(timeout_sec=timeout_sec)
        )

    def prepare_source_station(
        self,
        station_id: str,
        workpiece_positions: dict[str, tuple[float, float, float]],
    ):
        """Add the table, tag support, and every tray part before free motion."""
        if station_id != "raw_bin":
            raise ValueError(
                f"No source-station collision geometry for {station_id}"
            )

        objects = [
            self._box(
                "raw_bin_table",
                "base_link",
                _RAW_BIN_TABLE_POSITION,
                _RAW_BIN_TABLE_SIZE,
            ),
            self._box(
                "raw_bin_tag_support",
                "base_link",
                _RAW_BIN_TAG_POSITION,
                _RAW_BIN_TAG_SIZE,
            ),
        ]
        objects.extend(
            self._workpiece(part_id, "base_link", position)
            for part_id, position in sorted(workpiece_positions.items())
        )
        return self._replace_station_objects(objects)

    def prepare_finished_station(self):
        """Add the finished table and its physical marker support."""
        objects = [
            self._box(
                "finished_bin_table",
                "base_link",
                _FINISHED_BIN_TABLE_POSITION,
                _RAW_BIN_TABLE_SIZE,
            ),
            self._box(
                "finished_bin_tag_support",
                "base_link",
                _FINISHED_BIN_TAG_POSITION,
                _RAW_BIN_TAG_SIZE,
            ),
        ]
        return self._replace_station_objects(objects)

    def prepare_machine_station(
        self,
        station_id: str,
        workpiece_positions: dict[str, tuple[float, float, float]],
        *,
        station_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        """Add one repeatable CNC aperture, fixture, and visible workpieces."""
        if station_id not in {"machine_1", "machine_2", "machine_3"}:
            raise ValueError(f"No machine collision geometry for {station_id}")

        objects = [
            self._box(
                object_id,
                "base_link",
                self._translated(position, station_offset),
                size,
            )
            for object_id, position, size in MACHINE_COLLISION_BOXES
        ]
        # Measurements are in base_link already, so workpieces must not
        # receive the nominal-station offset a second time.
        objects.extend(
            self._workpiece(part_id, "base_link", position)
            for part_id, position in sorted(workpiece_positions.items())
        )
        return self._replace_station_objects(objects)

    def clear_station_geometry(self):
        """Remove geometry owned by the previously visited station.

        Station objects are measured in the robot's local frame at each dock.
        They must not survive a drive to another station as world-fixed ghost
        obstacles. Released workpieces are deliberately not tracked here and
        therefore remain in the world collision model.
        """
        removals = [
            self._remove_object(object_id)
            for object_id in sorted(self._active_station_object_ids)
        ]
        self._active_station_object_ids.clear()
        return self._apply_world_objects(removals)

    def _replace_station_objects(self, objects: list[CollisionObject]):
        """Atomically replace the last station model with the current one."""
        next_ids = {collision.id for collision in objects}
        removals = [
            self._remove_object(object_id)
            for object_id in sorted(self._active_station_object_ids - next_ids)
        ]
        self._active_station_object_ids = next_ids
        return self._apply_world_objects([*removals, *objects])

    def remove_world_object(self, object_id: str):
        """Remove one object only when process motion is allowed to touch it."""
        # The selected workpiece now leaves station-world ownership.  After
        # bilateral contact it will be represented as an attached object, so
        # a later station transition must not try to remove it from the world
        # for a second time.
        self._active_station_object_ids.discard(object_id)
        collision = self._remove_object(object_id)
        return self._apply_world_objects([collision])

    def attach_carried_workpiece_geometry(
        self, part_id: str, *, part_center_tool_y: float
    ):
        """Represent a physically held part in MoveIt's collision model.

        This PlanningScene diff cannot create force or a Gazebo constraint. The
        gripper must already hold the part through bilateral contact and friction.
        """
        if not -0.10 <= part_center_tool_y < 0.0:
            raise ValueError(
                "part_center_tool_y must be in [-0.10, 0.0)"
            )
        collision = self._workpiece(part_id, self._tool_frame, (0.0, 0.0, 0.0))
        # Tool X is the jaw-closing axis and tool Z points forward. This
        # inverse cyclic rotation keeps the carried collision cylinder upright.
        collision.pose.position.y = part_center_tool_y
        collision.pose.orientation.x = -0.5
        collision.pose.orientation.y = -0.5
        collision.pose.orientation.z = -0.5
        collision.pose.orientation.w = 0.5

        attached = AttachedCollisionObject()
        attached.link_name = self._tool_frame
        attached.object = collision
        # These links are expected to remain in physical contact with the
        # carried collision cylinder. Other robot-versus-part collisions stay active.
        attached.touch_links = list(GRIPPER_TOUCH_LINKS)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        return self._client.call_async(ApplyPlanningScene.Request(scene=scene))

    def place_released_workpiece_geometry(
        self,
        part_id: str,
        *,
        frame_id: str,
        position: tuple[float, float, float],
    ):
        """Move a released part from robot state to MoveIt's world geometry.

        This updates collision bookkeeping only; Gazebo release is produced by
        opening the physical fingers after support contact has been verified.
        """
        remove = AttachedCollisionObject()
        remove.link_name = self._tool_frame
        remove.object.id = part_id
        remove.object.operation = CollisionObject.REMOVE

        placed = self._workpiece(part_id, frame_id, position)
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [remove]
        scene.world.collision_objects = [placed]
        return self._client.call_async(ApplyPlanningScene.Request(scene=scene))

    def get_allowed_collision_matrix(self):
        """Read MoveIt's complete ACM before adding a process allowance."""
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        return self._scene_reader.call_async(request)

    def allow_finished_bin_support_contact(
        self,
        part_id: str,
        matrix: AllowedCollisionMatrix,
    ):
        """Allow only the held workpiece to seat on the finished table.

        The gripper, arm, marker support and table rims remain collision
        checked. This single process allowance lets a guarded one-millimetre
        descent produce physical support contact before release.
        """
        for support_name in FINISHED_BIN_SUPPORT_OBJECTS:
            self._allow_collision_pair(matrix, part_id, support_name)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = matrix
        return self._client.call_async(ApplyPlanningScene.Request(scene=scene))

    def allow_machine_fixture_contact(
        self,
        part_id: str,
        matrix: AllowedCollisionMatrix,
    ):
        """Allow only the held workpiece to touch the CNC vise.

        The final vertical insertion intentionally rests the cylinder on the
        fixture base. Robot links, the table, and the enclosure remain fully
        collision checked. The supplied complete matrix preserves SRDF
        allowances for adjacent robot links.
        """
        for fixture_name in MACHINE_FIXTURE_CONTACT_OBJECTS:
            self._allow_collision_pair(matrix, part_id, fixture_name)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = matrix
        return self._client.call_async(ApplyPlanningScene.Request(scene=scene))

    @staticmethod
    def _allow_collision_pair(
        matrix: AllowedCollisionMatrix,
        first_name: str,
        second_name: str,
    ) -> None:
        """Extend an ACM without discarding any existing robot-link entries."""
        for name in (first_name, second_name):
            if name in matrix.entry_names:
                continue
            old_size = len(matrix.entry_names)
            if len(matrix.entry_values) != old_size:
                raise ValueError("Allowed-collision matrix is not square")
            for entry in matrix.entry_values:
                if len(entry.enabled) != old_size:
                    raise ValueError("Allowed-collision matrix is not square")
                entry.enabled.append(False)
            matrix.entry_names.append(name)
            matrix.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * (old_size + 1))
            )

        first = matrix.entry_names.index(first_name)
        second = matrix.entry_names.index(second_name)
        matrix.entry_values[first].enabled[second] = True
        matrix.entry_values[second].enabled[first] = True

    def _apply_world_objects(self, objects: list[CollisionObject]):
        scene = PlanningScene()
        scene.is_diff = True
        # A world-only update must not replace the current joint state or its
        # attached workpiece with an empty default RobotState. Without this
        # flag, the next plan may briefly start from zero joints and report
        # false arm-versus-pedestal self collisions.
        scene.robot_state.is_diff = True
        scene.world.collision_objects = objects
        return self._client.call_async(ApplyPlanningScene.Request(scene=scene))

    @staticmethod
    def _remove_object(object_id: str) -> CollisionObject:
        """Build one world-object removal for an atomic scene transition."""
        collision = CollisionObject()
        collision.header.frame_id = "base_link"
        collision.id = object_id
        collision.operation = CollisionObject.REMOVE
        return collision

    @staticmethod
    def _box(
        object_id: str,
        frame_id: str,
        position: tuple[float, float, float],
        size: tuple[float, float, float],
    ) -> CollisionObject:
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        return PlanningSceneClient._collision_object(
            object_id, frame_id, position, box
        )

    @staticmethod
    def _workpiece(
        part_id: str,
        frame_id: str,
        position: tuple[float, float, float],
    ) -> CollisionObject:
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [WORKPIECE_HEIGHT, WORKPIECE_RADIUS]
        return PlanningSceneClient._collision_object(
            part_id, frame_id, position, cylinder
        )

    @staticmethod
    def _translated(
        position: tuple[float, float, float],
        offset: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Translate nominal station geometry into the current dock frame."""
        return tuple(
            coordinate + delta
            for coordinate, delta in zip(position, offset, strict=True)
        )

    @staticmethod
    def _collision_object(
        object_id: str,
        frame_id: str,
        position: tuple[float, float, float],
        primitive: SolidPrimitive,
    ) -> CollisionObject:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0

        collision = CollisionObject()
        collision.header.frame_id = frame_id
        collision.id = object_id
        collision.pose.orientation.w = 1.0
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD
        return collision
