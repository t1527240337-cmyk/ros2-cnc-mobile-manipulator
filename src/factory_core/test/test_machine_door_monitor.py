from sensor_msgs.msg import JointState

from factory_core.machine_door_monitor import (
    DoorPositionSample,
    MachineDoorMonitor,
    joint_position,
)


def test_joint_position_finds_the_named_door_joint():
    message = JointState()
    message.name = ["other_joint", "sliding_door_joint"]
    message.position = [0.25, 1.17]

    assert joint_position(message) == 1.17


def test_joint_position_rejects_missing_or_malformed_arrays():
    missing = JointState()
    missing.name = ["other_joint"]
    missing.position = [1.18]
    assert joint_position(missing) is None

    malformed = JointState()
    malformed.name = ["sliding_door_joint"]
    assert joint_position(malformed) is None


def test_open_limit_uses_measured_position_tolerance():
    open_sample = DoorPositionSample(position=1.13, received_at=1.0)
    blocked_sample = DoorPositionSample(position=1.12, received_at=1.0)

    assert MachineDoorMonitor.is_open(open_sample)
    assert not MachineDoorMonitor.is_open(blocked_sample)
