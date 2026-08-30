import unittest

from pydantic import ValidationError

from factory_agent.command_models import (
    FORBIDDEN_LOW_LEVEL_INTERFACES,
    AgentOperation,
)
from factory_agent.mcp_tools import (
    FactoryMcpTools,
    MCP_TOOL_NAMES,
    decode_json_payload,
)


class RecordingExecutor:
    def __init__(self):
        self.calls = []

    def __call__(self, command, source):
        self.calls.append((command, source))
        return {
            "accepted": True,
            "operation": command.operation.value,
            "data": {},
        }


class FactoryMcpToolsTests(unittest.TestCase):
    def setUp(self):
        self.executor = RecordingExecutor()
        self.tools = FactoryMcpTools(self.executor)

    def test_state_query_is_an_immediate_service_operation(self):
        result = self.tools.get_factory_state()

        self.assertTrue(result["accepted"])
        self.assertEqual(len(self.executor.calls), 1)
        command, source = self.executor.calls[0]
        self.assertEqual(command.operation, AgentOperation.GET_FACTORY_STATE)
        self.assertEqual(source, "mcp")

    def test_submit_order_preserves_constraints(self):
        self.tools.submit_order(
            quantity=3,
            allowed_machine_ids=["machine_3", "machine_1"],
            auto_recharge=False,
        )

        command, _ = self.executor.calls[0]
        self.assertEqual(command.quantity, 3)
        self.assertEqual(
            command.allowed_machine_ids,
            ["machine_3", "machine_1"],
        )
        self.assertFalse(command.auto_recharge)

    def test_non_positive_order_never_reaches_ros(self):
        with self.assertRaises(ValidationError):
            self.tools.submit_order(0)
        self.assertEqual(self.executor.calls, [])

    def test_invalid_machine_never_reaches_ros(self):
        with self.assertRaises(ValidationError):
            self.tools.hold_machine("machine_99")
        self.assertEqual(self.executor.calls, [])

    def test_all_public_tools_are_explicitly_allow_listed(self):
        expected = {
            name
            for name in dir(self.tools)
            if not name.startswith("_") and callable(getattr(self.tools, name))
        }
        self.assertEqual(set(MCP_TOOL_NAMES), expected)

    def test_no_low_level_interface_is_exposed_as_a_tool(self):
        normalized_forbidden = {
            interface.strip("/").replace("/", "_")
            for interface in FORBIDDEN_LOW_LEVEL_INTERFACES
        }
        self.assertTrue(set(MCP_TOOL_NAMES).isdisjoint(normalized_forbidden))

    def test_json_payload_decoder_rejects_malformed_ros_data(self):
        self.assertEqual(
            decode_json_payload('{"raw_parts":3}'),
            {"raw_parts": 3},
        )
        with self.assertRaisesRegex(RuntimeError, "invalid data_json"):
            decode_json_payload("{broken")


if __name__ == "__main__":
    unittest.main()
