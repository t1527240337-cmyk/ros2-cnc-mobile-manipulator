from xml.etree import ElementTree

import pytest

from factory_core.physics_description_filter import make_physics_description


FULL_DESCRIPTION = """
<robot name="test_robot">
  <link name="base"/>
  <link name="gripper_parallel_base_link"/>
  <link name="gripper_left_finger_tip_link">
    <collision name="left_near"><geometry><box size="0.006 0.060 0.012"/></geometry></collision>
    <collision name="left_far"><geometry><box size="0.006 0.060 0.012"/></geometry></collision>
  </link>
  <link name="gripper_right_finger_tip_link">
    <collision name="right_near"><geometry><box size="0.006 0.060 0.012"/></geometry></collision>
    <collision name="right_far"><geometry><box size="0.006 0.060 0.012"/></geometry></collision>
  </link>
  <joint name="gripper_left_finger_joint" type="prismatic">
    <parent link="gripper_parallel_base_link"/>
    <child link="gripper_left_finger_tip_link"/>
  </joint>
  <joint name="gripper_right_finger_joint" type="prismatic">
    <parent link="gripper_parallel_base_link"/>
    <child link="gripper_right_finger_tip_link"/>
  </joint>
</robot>
"""


def test_filter_keeps_one_shared_convex_gripper_description():
    description = make_physics_description(FULL_DESCRIPTION)
    root = ElementTree.fromstring(description)

    left = root.find("link[@name='gripper_left_finger_tip_link']")
    right_joint = root.find("joint[@name='gripper_right_finger_joint']")

    collisions = left.findall("collision/geometry/box")
    assert len(collisions) == 2
    assert all(box.get("size") == "0.006 0.060 0.012" for box in collisions)
    assert right_joint.find("mimic") is None
    assert description == FULL_DESCRIPTION


def test_filter_rejects_incomplete_parallel_gripper():
    incomplete = "<robot name='incomplete'><link name='base'/></robot>"

    with pytest.raises(ValueError, match="missing robot link"):
        make_physics_description(incomplete)
