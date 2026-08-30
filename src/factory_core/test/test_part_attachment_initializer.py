from factory_core.part_attachment_initializer import startup_detach_topics


def test_startup_clears_gripper_joints_but_retains_raw_tray_fixtures():
    topics = startup_detach_topics(("raw_part_1", "raw_part_2"))

    assert topics == (
        "/factory/gripper/raw_part_1/detach",
        "/factory/gripper/raw_part_2/detach",
    )
    assert all("/fixture/" not in topic for topic in topics)
