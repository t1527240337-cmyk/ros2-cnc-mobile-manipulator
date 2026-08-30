import unittest

from factory_agent.mcp_tools import FactoryMcpTools, MCP_TOOL_NAMES
from factory_agent.mcp_wiring import create_mcp_server


class FakeFastMcp:
    def __init__(
        self, name, *, instructions, json_response, stateless_http, host, port
    ):
        self.name = name
        self.instructions = instructions
        self.json_response = json_response
        self.settings = (stateless_http, host, port)
        self.registered = {}

    def tool(self, *, name):
        def register(function):
            self.registered[name] = function
            return function

        return register


class McpWiringTests(unittest.TestCase):
    def test_server_registers_only_the_explicit_allow_list(self):
        def execute(command, source):
            return {
                "accepted": True,
                "operation": command.operation.value,
                "source": source,
            }

        tools = FactoryMcpTools(execute)
        server = create_mcp_server(tools, fast_mcp_type=FakeFastMcp)

        self.assertEqual(server.name, "ROS 2 Factory Operator")
        self.assertTrue(server.json_response)
        self.assertEqual(set(server.registered), set(MCP_TOOL_NAMES))
        self.assertIn("never enter the robot production queue", server.instructions)
        self.assertEqual(server.settings, (True, "127.0.0.1", 8000))

    def test_registered_tool_keeps_schema_validating_handler(self):
        calls = []

        def execute(command, source):
            calls.append((command, source))
            return {"accepted": True}

        tools = FactoryMcpTools(execute)
        server = create_mcp_server(tools, fast_mcp_type=FakeFastMcp)
        result = server.registered["submit_order"](
            quantity=2,
            allowed_machine_ids=["machine_2"],
        )

        self.assertTrue(result["accepted"])
        self.assertEqual(calls[0][0].quantity, 2)
        self.assertEqual(calls[0][0].allowed_machine_ids, ["machine_2"])
        self.assertEqual(calls[0][1], "mcp")

    def test_server_uses_requested_http_endpoint(self):
        def execute(command, source):
            return {
                "accepted": True,
                "operation": command.operation.value,
                "source": source,
            }

        server = create_mcp_server(
            FactoryMcpTools(execute),
            fast_mcp_type=FakeFastMcp,
            host="0.0.0.0",
            port=8123,
        )
        self.assertEqual(server.settings, (True, "0.0.0.0", 8123))


if __name__ == "__main__":
    unittest.main()
