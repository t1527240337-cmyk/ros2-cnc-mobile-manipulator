from factory_core.planning_scene_client import (
    FINISHED_BIN_SUPPORT_OBJECTS,
    GRIPPER_TOUCH_LINKS,
    MACHINE_FIXTURE_CONTACT_OBJECTS,
    MACHINE_COLLISION_BOXES,
    PlanningSceneClient,
)
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix, CollisionObject


def test_attached_workpiece_allows_only_gripper_contact_links():
    assert "gripper_left_finger_tip_link" in GRIPPER_TOUCH_LINKS
    assert "gripper_right_finger_tip_link" in GRIPPER_TOUCH_LINKS
    assert "base_link" not in GRIPPER_TOUCH_LINKS
    assert "arm_forearm_link" not in GRIPPER_TOUCH_LINKS


def test_machine_scene_models_aperture_and_flush_pallet():
    boxes = {name: (position, size) for name, position, size in MACHINE_COLLISION_BOXES}

    assert "machine_base_cabinet" in boxes
    assert "machine_rear_wall" in boxes
    assert "machine_fixture_base" in boxes
    fixture_position, fixture_size = boxes["machine_fixture_base"]
    assert fixture_position == (0.95, 0.0, 0.800)
    assert fixture_size == (0.40, 0.38, 0.12)
    assert "machine_fixture_left_jaw" not in boxes
    assert "machine_fixture_right_jaw" not in boxes
    assert "machine_front_lintel" in boxes
    assert "machine_front_left_post" in boxes
    assert "machine_front_right_post" in boxes
    assert "machine_spindle_head" in boxes
    assert "machine_spindle_nose" in boxes
    assert "machine_cutting_tool" in boxes


def test_machine_scene_offset_translates_nominal_geometry():
    translated = PlanningSceneClient._translated(
        (0.85, 0.10, 0.800),
        (0.094, -0.002, 0.0),
    )

    assert translated == (0.944, 0.098, 0.800)

class _CapturingPlanningSceneService:
    def __init__(self):
        self.request = None

    def call_async(self, request):
        self.request = request
        return object()


def test_world_update_is_a_robot_state_diff():
    service = _CapturingPlanningSceneService()
    client = PlanningSceneClient.__new__(PlanningSceneClient)
    client._client = service
    client._active_station_object_ids = set()

    client._apply_world_objects([])

    assert service.request.scene.is_diff
    assert service.request.scene.robot_state.is_diff


def test_station_transition_removes_previous_geometry_atomically():
    service = _CapturingPlanningSceneService()
    client = PlanningSceneClient.__new__(PlanningSceneClient)
    client._client = service
    client._active_station_object_ids = set()

    client.prepare_machine_station("machine_1", {})
    machine_ids = {name for name, _position, _size in MACHINE_COLLISION_BOXES}
    assert client._active_station_object_ids == machine_ids

    client.prepare_source_station("raw_bin", {"part_1": (0.8, 0.0, 0.25)})
    updates = service.request.scene.world.collision_objects
    removed = {
        item.id for item in updates if item.operation == CollisionObject.REMOVE
    }
    added = {
        item.id for item in updates if item.operation == CollisionObject.ADD
    }

    assert removed == machine_ids
    assert added == {
        "raw_bin_table",
        "raw_bin_tag_support",
        "part_1",
    }
    assert client._active_station_object_ids == added


def test_bin_scene_geometry_matches_calibrated_tag_offsets():
    service = _CapturingPlanningSceneService()
    client = PlanningSceneClient.__new__(PlanningSceneClient)
    client._client = service
    client._active_station_object_ids = set()

    client.prepare_source_station("raw_bin", {})
    raw_objects = {
        item.id: item
        for item in service.request.scene.world.collision_objects
        if item.operation == CollisionObject.ADD
    }
    assert (
        raw_objects["raw_bin_table"].primitive_poses[0].position.x
        == 1.182
    )
    assert (
        raw_objects["raw_bin_tag_support"].primitive_poses[0].position.x
        == 0.602
    )

    client.prepare_finished_station()
    finished_objects = {
        item.id: item
        for item in service.request.scene.world.collision_objects
        if item.operation == CollisionObject.ADD
    }
    assert (
        finished_objects["finished_bin_table"].primitive_poses[0].position.x
        == 1.262
    )
    assert (
        finished_objects[
            "finished_bin_tag_support"
        ].primitive_poses[0].position.x
        == 0.682
    )

def test_clear_station_geometry_does_not_remove_released_workpieces():
    service = _CapturingPlanningSceneService()
    client = PlanningSceneClient.__new__(PlanningSceneClient)
    client._client = service
    client._active_station_object_ids = {"finished_bin_table", "finished_bin_tag_support"}

    client.clear_station_geometry()

    updates = service.request.scene.world.collision_objects
    assert {item.id for item in updates} == {
        "finished_bin_table",
        "finished_bin_tag_support",
    }
    assert all(item.operation == CollisionObject.REMOVE for item in updates)
    assert client._active_station_object_ids == set()


def test_selected_workpiece_leaves_station_world_ownership():
    service = _CapturingPlanningSceneService()
    client = PlanningSceneClient.__new__(PlanningSceneClient)
    client._client = service
    client._active_station_object_ids = {
        "raw_bin_table",
        "raw_bin_tag_support",
        "part_1",
    }

    client.remove_world_object("part_1")

    update = service.request.scene.world.collision_objects
    assert len(update) == 1
    assert update[0].id == "part_1"
    assert update[0].operation == CollisionObject.REMOVE
    assert client._active_station_object_ids == {
        "raw_bin_table",
        "raw_bin_tag_support",
    }


def test_fixture_contact_permission_is_limited_to_workpiece_pairs():
    service = _CapturingPlanningSceneService()
    client = PlanningSceneClient.__new__(PlanningSceneClient)
    client._client = service
    client._active_station_object_ids = set()

    matrix = AllowedCollisionMatrix(
        entry_names=["arm_parent", "arm_child"],
        entry_values=[
            AllowedCollisionEntry(enabled=[False, True]),
            AllowedCollisionEntry(enabled=[True, False]),
        ],
    )
    client.allow_machine_fixture_contact("raw_part_2", matrix)

    scene = service.request.scene
    matrix = scene.allowed_collision_matrix
    names = list(matrix.entry_names)
    assert scene.is_diff
    assert scene.robot_state.is_diff
    assert names == [
        "arm_parent",
        "arm_child",
        "raw_part_2",
        *MACHINE_FIXTURE_CONTACT_OBJECTS,
    ]

    allowed_pairs = {
        frozenset((row_name, column_name))
        for row, row_name in enumerate(names)
        for column, column_name in enumerate(names)
        if matrix.entry_values[row].enabled[column]
    }
    expected_process_pairs = {
        frozenset(("raw_part_2", fixture_name))
        for fixture_name in MACHINE_FIXTURE_CONTACT_OBJECTS
    }
    assert allowed_pairs == {
        frozenset(("arm_parent", "arm_child"))
    } | expected_process_pairs
    assert "machine_work_table" not in names


def test_finished_contact_permission_is_only_part_to_table():
    service = _CapturingPlanningSceneService()
    client = PlanningSceneClient.__new__(PlanningSceneClient)
    client._client = service
    client._active_station_object_ids = set()

    matrix = AllowedCollisionMatrix(
        entry_names=["gripper_tcp", "arm_wrist_3_link"],
        entry_values=[
            AllowedCollisionEntry(enabled=[False, True]),
            AllowedCollisionEntry(enabled=[True, False]),
        ],
    )
    client.allow_finished_bin_support_contact("raw_part_2", matrix)

    scene = service.request.scene
    matrix = scene.allowed_collision_matrix
    names = list(matrix.entry_names)
    assert scene.is_diff
    assert scene.robot_state.is_diff
    assert names == [
        "gripper_tcp",
        "arm_wrist_3_link",
        "raw_part_2",
        *FINISHED_BIN_SUPPORT_OBJECTS,
    ]

    allowed_pairs = {
        frozenset((row_name, column_name))
        for row, row_name in enumerate(names)
        for column, column_name in enumerate(names)
        if matrix.entry_values[row].enabled[column]
    }
    assert allowed_pairs == {
        frozenset(("gripper_tcp", "arm_wrist_3_link")),
        frozenset(("raw_part_2", "finished_bin_table")),
    }
