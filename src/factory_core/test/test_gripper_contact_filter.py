from ros_gz_interfaces.msg import Contact, Contacts

from factory_core.gazebo_truth_observer import gripper_sides_touching_part


def _contact(gripper_collision: str) -> Contacts:
    contact = Contact()
    contact.collision1.name = f"factory_robot::{gripper_collision}::collision"
    contact.collision2.name = "raw_part_2::part::collision"
    message = Contacts()
    message.contacts = [contact]
    return message


def test_gripper_base_contact_cannot_prove_fingertip_grasp():
    message = _contact("gripper_parallel_base_link")

    assert gripper_sides_touching_part(message, "raw_part_2") == set()


def test_tool_adapter_contact_cannot_prove_fingertip_grasp():
    message = _contact("ur_to_robotiq_link")

    assert gripper_sides_touching_part(message, "raw_part_2") == set()


def test_finger_tip_contact_is_classified_by_side():
    message = _contact("gripper_left_finger_tip_link")

    assert gripper_sides_touching_part(message, "raw_part_2") == {"left"}
