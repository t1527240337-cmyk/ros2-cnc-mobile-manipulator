import pytest

from factory_core.planning_scene_client import PlanningSceneClient


class _CapturingPlanningSceneService:
    def __init__(self):
        self.request = None

    def call_async(self, request):
        self.request = request
        return object()


def _client():
    client = PlanningSceneClient.__new__(PlanningSceneClient)
    client._client = _CapturingPlanningSceneService()
    client._tool_frame = "gripper_tcp"
    client._active_station_object_ids = set()
    return client


def test_finished_scene_models_table_and_marker_support():
    client = _client()

    client.prepare_finished_station()

    objects = {
        collision.id: collision
        for collision in client._client.request.scene.world.collision_objects
    }
    assert set(objects) == {"finished_bin_table", "finished_bin_tag_support"}
    assert objects["finished_bin_table"].primitive_poses[0].position.x == 1.262
    assert objects["finished_bin_table"].primitive_poses[0].position.z == -0.076


def test_carried_workpiece_geometry_uses_measured_source_grasp_offset():
    client = _client()

    client.attach_carried_workpiece_geometry(
        "raw_part_2", part_center_tool_y=-0.05
    )

    attached = (
        client._client.request.scene.robot_state.attached_collision_objects[0]
    )
    assert attached.object.pose.position.y == -0.05


@pytest.mark.parametrize("invalid_offset", [0.0, 0.01, -0.101])
def test_carried_workpiece_geometry_rejects_impossible_offsets(invalid_offset):
    client = _client()

    with pytest.raises(ValueError, match="part_center_tool_y"):
        client.attach_carried_workpiece_geometry(
            "raw_part_2",
            part_center_tool_y=invalid_offset,
        )
