import math

from factory_core.gripper_client import GripperClient
from sensor_msgs.msg import JointState


class RecordingPublisher:
    def __init__(self):
        self.commands = []

    def publish(self, message):
        self.commands.append(tuple(message.data))


def effort_gripper(*, left=0.0, right=0.0):
    gripper = object.__new__(GripperClient)
    gripper._command_publisher = RecordingPublisher()
    gripper._command_effort = 0.0
    gripper._positions = dict(zip(GripperClient.JOINT_NAMES, (left, right)))
    gripper._velocities = {}
    gripper._joint_state_time = None
    return gripper


def test_controller_commands_both_independent_fingers():
    assert GripperClient.JOINT_NAMES == (
        "gripper_left_finger_joint",
        "gripper_right_finger_joint",
    )


def test_contact_hold_applies_equal_bounded_force():
    gripper = effort_gripper(left=0.011, right=0.011)

    positions = gripper.hold_at_contact(0.022)

    assert positions == (0.011, 0.011)
    assert gripper._command_publisher.commands == [
        (GripperClient.HOLD_EFFORT, GripperClient.HOLD_EFFORT)
    ]


def test_search_and_hold_efforts_stay_inside_software_safety_bound():
    assert 0.0 < GripperClient.CLOSE_EFFORT < GripperClient.HOLD_EFFORT
    assert GripperClient.HOLD_EFFORT == 36.0
    assert GripperClient.HOLD_EFFORT < 40.0
    assert GripperClient.MAX_COMMAND_EFFORT < 40.0
    assert GripperClient.HOLD_EFFORT <= GripperClient.MAX_COMMAND_EFFORT
    assert abs(GripperClient.OPEN_EFFORT) <= GripperClient.MAX_COMMAND_EFFORT


def test_v_jaw_contact_position_is_not_used_as_grasp_identity():
    assert GripperClient.MAX_SAFE_PICK_POSITION == 0.006
    assert GripperClient.MIN_VERIFIED_TOTAL_CLOSURE == 0.0


def test_joint_state_sample_has_a_fresh_receipt_time():
    gripper = object.__new__(GripperClient)
    gripper._positions = {}
    gripper._velocities = {}
    gripper._joint_state_time = None
    message = JointState(
        name=list(GripperClient.JOINT_NAMES),
        position=[0.029, 0.028],
        velocity=[-0.001, 0.002],
    )

    gripper._remember_joint_state(message)
    position, velocity, received_at = gripper.measured_sample()

    assert math.isclose(position, 0.057)
    assert math.isclose(velocity, 0.002)
    assert gripper.measured_positions() == (0.029, 0.028)
    assert received_at is not None


def test_release_still_requires_both_fingers_inside_open_bound():
    assert GripperClient.SAFE_RELEASE_POSITION <= GripperClient.RELEASE_OPEN_POSITION



def test_close_to_rejects_targets_outside_physical_range():
    gripper = object.__new__(GripperClient)

    for invalid in (-0.01, GripperClient.CLOSED_POSITION + 0.01):
        try:
            gripper.close_to(invalid)
        except ValueError as error:
            assert "invalid gripper closure target" in str(error)
        else:
            raise AssertionError(f"accepted invalid target {invalid}")

def test_safe_pick_aperture_is_wider_than_part_release_aperture():
    assert GripperClient.OPEN_POSITION < GripperClient.MAX_SAFE_PICK_POSITION

def test_one_finger_joint_state_cannot_prove_parallel_grasp():
    gripper = object.__new__(GripperClient)
    gripper._positions = {}
    gripper._velocities = {}
    gripper._joint_state_time = None
    message = JointState(
        name=[GripperClient.JOINT_NAMES[0]],
        position=[0.016],
        velocity=[0.0],
    )

    gripper._remember_joint_state(message)

    assert gripper.measured_state() == (None, None)
    assert gripper.measured_sample()[2] is None


def test_mechanical_fingers_accept_small_state_difference():
    gripper = object.__new__(GripperClient)
    gripper._positions = {
        GripperClient.JOINT_NAMES[0]: 0.018,
        GripperClient.JOINT_NAMES[1]: 0.017,
    }

    assert gripper.fingers_are_symmetric()


def test_off_centre_bilateral_contact_can_hold_with_equal_force():
    gripper = effort_gripper(left=0.025, right=0.015)

    assert not gripper.fingers_are_symmetric()
    assert gripper.hold_at_contact(0.040) == (0.025, 0.015)
    assert gripper._command_publisher.commands[-1] == (
        GripperClient.HOLD_EFFORT,
        GripperClient.HOLD_EFFORT,
    )


def test_safe_opening_requires_both_fingers_inside_bound():
    gripper = effort_gripper(left=0.005, right=0.012)

    assert not gripper.both_fingers_at_or_below(
        GripperClient.MAX_SAFE_PICK_POSITION
    )
    gripper._positions[GripperClient.JOINT_NAMES[1]] = 0.006
    assert gripper.both_fingers_at_or_below(
        GripperClient.MAX_SAFE_PICK_POSITION
    )

def test_contact_hold_fails_without_both_measured_fingers():
    gripper = object.__new__(GripperClient)
    gripper._positions = {GripperClient.JOINT_NAMES[0]: 0.016}

    try:
        gripper.hold_at_contact(0.016)
    except RuntimeError as error:
        assert "both finger positions" in str(error)
    else:
        raise AssertionError("accepted a one-finger hold sample")

def test_release_contact_commands_bounded_v_jaw_clearance():
    gripper = effort_gripper()

    gripper.release_contact()

    assert gripper._command_publisher.commands == [
        (GripperClient.OPEN_EFFORT, GripperClient.OPEN_EFFORT)
    ]


def test_open_close_and_stop_are_equal_force_commands():
    gripper = effort_gripper()

    gripper.open()
    gripper.close_to(GripperClient.CLOSED_POSITION)
    gripper.stop()

    assert gripper._command_publisher.commands == [
        (GripperClient.OPEN_EFFORT, GripperClient.OPEN_EFFORT),
        (GripperClient.CLOSE_EFFORT, GripperClient.CLOSE_EFFORT),
        (0.0, 0.0),
    ]


def test_effort_refresh_repeats_the_owned_hold_command():
    gripper = effort_gripper(left=0.011, right=0.011)
    gripper.hold_at_contact(0.022)

    gripper._republish_command()

    expected = (GripperClient.HOLD_EFFORT, GripperClient.HOLD_EFFORT)
    assert gripper._command_publisher.commands == [expected, expected]


def test_shallow_v_contact_can_receive_force_before_proof_lift():
    gripper = effort_gripper(left=0.005, right=0.005)

    assert gripper.hold_at_contact(0.010) == (0.005, 0.005)
    assert gripper._command_publisher.commands[-1] == (
        GripperClient.HOLD_EFFORT,
        GripperClient.HOLD_EFFORT,
    )


def test_closed_stops_cannot_be_treated_as_a_workpiece_hold():
    gripper = effort_gripper(left=0.040, right=0.040)

    try:
        gripper.hold_at_contact(0.080)
    except RuntimeError as error:
        assert "outside the physical hold range" in str(error)
    else:
        raise AssertionError("accepted the empty closed stops as a grasp")
