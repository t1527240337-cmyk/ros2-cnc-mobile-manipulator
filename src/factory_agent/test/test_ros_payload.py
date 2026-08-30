from types import SimpleNamespace

from factory_agent.ros_node import FactoryAgentNode


def _factory_result(*, machine_state: int, battery_status: int):
    duration = SimpleNamespace(sec=3, nanosec=500_000_000)
    machine = SimpleNamespace(
        machine_id="machine_1", state=machine_state, door_open=False,
        part_present=True, part_id="part-1", remaining_time=duration,
        fault_code=0,
    )
    battery = SimpleNamespace(
        percentage=0.42, voltage=48.0, current=-3.0,
        power_supply_status=battery_status,
    )
    return SimpleNamespace(
        machines=[machine], raw_part_count=4, finished_part_count=0,
        held_part_id="", battery=battery, active_order_id="",
    )


def test_factory_payload_adds_symbolic_ros_enum_names():
    payload = FactoryAgentNode._factory_payload(
        _factory_result(machine_state=2, battery_status=2)
    )

    assert payload["machines"][0]["state_name"] == "PROCESSING"
    assert payload["battery"]["power_supply_status_name"] == "DISCHARGING"


def test_factory_payload_marks_unknown_ros_enum_values():
    payload = FactoryAgentNode._factory_payload(
        _factory_result(machine_state=99, battery_status=99)
    )

    assert payload["machines"][0]["state_name"] == "UNRECOGNIZED_99"
    assert payload["battery"]["power_supply_status_name"] == "UNRECOGNIZED_99"
