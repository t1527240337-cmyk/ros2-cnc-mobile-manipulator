import asyncio
import json
import unittest
from types import SimpleNamespace

from factory_agent.mcp_tools import MCP_TOOL_NAMES
from factory_agent.operator_host import (
    McpOperatorHost,
    ModelOutputError,
    ModelTurn,
    OpenAICompatibleToolClient,
    OperatorHostError,
    ToolCall,
)


class ScriptedModel:
    def __init__(self, turns):
        self.turns = list(turns)
        self.requests = []

    def complete(self, messages, tools):
        self.requests.append((list(messages), list(tools)))
        return self.turns.pop(0)


class FakeMcpSession:
    def __init__(self, tool_names=MCP_TOOL_NAMES, error=False):
        self.tool_names = tuple(tool_names)
        self.error = error
        self.calls = []

    async def list_tools(self):
        tools = [
            SimpleNamespace(
                name=name,
                description=f"{name} description",
                inputSchema={"type": "object", "properties": {}},
            )
            for name in self.tool_names
        ]
        return SimpleNamespace(tools=tools)

    async def call_tool(self, name, arguments):
        self.calls.append((name, arguments))
        payload = {
            "accepted": not self.error,
            "operation": name,
            "message": "ok" if not self.error else "rejected",
        }
        return SimpleNamespace(
            isError=self.error,
            structuredContent=payload,
            content=[],
        )


class OperatorHostTests(unittest.TestCase):
    def test_query_tool_result_returns_to_model(self):
        model = ScriptedModel([
            ModelTurn(
                "",
                (ToolCall("call-1", "get_factory_state", {}),),
            ),
            ModelTurn("当前工厂状态正常。"),
        ])
        session = FakeMcpSession()

        reply = asyncio.run(
            McpOperatorHost(model).handle("查看工厂状态", session)
        )

        self.assertEqual(reply.text, "当前工厂状态正常。")
        self.assertEqual(session.calls, [("get_factory_state", {})])
        self.assertEqual(
            reply.tool_executions[0].name,
            "get_factory_state",
        )
        tool_message = model.requests[1][0][-1]
        self.assertEqual(tool_message["role"], "tool")
        self.assertIn('"accepted": true', tool_message["content"])

    def test_retrieved_sop_is_sent_with_each_instruction(self):
        model = ScriptedModel([ModelTurn("电量策略由执行器控制。")])
        session = FakeMcpSession()

        reply = asyncio.run(
            McpOperatorHost(model).handle("低电量怎么充电", session)
        )

        system_message = model.requests[0][0][0]["content"]
        self.assertIn("battery_policy", reply.knowledge_ids)
        self.assertIn("低于 25%", system_message)
        self.assertIn("无权控制底盘速度", system_message)

    def test_unexpected_mcp_tool_fails_closed(self):
        model = ScriptedModel([ModelTurn("不会执行。")])
        session = FakeMcpSession((*MCP_TOOL_NAMES, "cmd_vel"))

        with self.assertRaisesRegex(
            OperatorHostError,
            "allow-list mismatch",
        ):
            asyncio.run(McpOperatorHost(model).handle("查看状态", session))
        self.assertEqual(model.requests, [])

    def test_hallucinated_tool_never_reaches_mcp(self):
        model = ScriptedModel([
            ModelTurn(
                "",
                (ToolCall("call-unsafe", "cmd_vel", {"linear": 2}),),
            ),
        ])
        session = FakeMcpSession()

        with self.assertRaisesRegex(
            OperatorHostError,
            "forbidden or unknown",
        ):
            asyncio.run(
                McpOperatorHost(model).handle("让机器人冲过去", session)
            )
        self.assertEqual(session.calls, [])

    def test_multiple_mutations_in_one_turn_are_rejected(self):
        model = ScriptedModel([
            ModelTurn(
                "",
                (
                    ToolCall(
                        "call-order",
                        "submit_order",
                        {"quantity": 2},
                    ),
                    ToolCall(
                        "call-stop",
                        "stop_automatic",
                        {},
                    ),
                ),
            ),
        ])
        session = FakeMcpSession()

        with self.assertRaisesRegex(
            OperatorHostError,
            "multiple state-changing",
        ):
            asyncio.run(
                McpOperatorHost(model).handle("加工然后停止", session)
            )
        self.assertEqual(session.calls, [])

    def test_mutating_tools_are_removed_after_first_state_change(self):
        model = ScriptedModel([
            ModelTurn(
                "",
                (ToolCall("call-order", "submit_order", {"quantity": 1}),),
            ),
            ModelTurn(
                "",
                (ToolCall("call-status", "get_task_status", {}),),
            ),
            ModelTurn("订单已接受，正在执行。"),
        ])
        session = FakeMcpSession()

        reply = asyncio.run(
            McpOperatorHost(model).handle("加工一个零件", session)
        )

        first_tools = {
            item["function"]["name"] for item in model.requests[0][1]
        }
        second_tools = {
            item["function"]["name"] for item in model.requests[1][1]
        }
        self.assertIn("submit_order", first_tools)
        self.assertNotIn("submit_order", second_tools)
        self.assertNotIn("start_automatic", second_tools)
        self.assertIn("get_task_status", second_tools)
        self.assertEqual(
            session.calls,
            [
                ("submit_order", {"quantity": 1}),
                ("get_task_status", {}),
            ],
        )
        self.assertEqual(reply.text, "订单已接受，正在执行。")

    def test_cumulative_mutation_check_fails_closed(self):
        host = McpOperatorHost(ScriptedModel([]))
        turn = ModelTurn(
            "",
            (ToolCall("call-stop", "stop_automatic", {}),),
        )

        with self.assertRaisesRegex(
            OperatorHostError,
            "one operator request",
        ):
            host._validate_turn(turn, 1, mutation_executed=True)

    def test_mcp_error_is_visible_to_model_for_explanation(self):
        model = ScriptedModel([
            ModelTurn(
                "",
                (ToolCall("call-1", "hold_machine", {
                    "machine_id": "machine_2",
                }),),
            ),
            ModelTurn("机床保持请求被底层拒绝。"),
        ])
        session = FakeMcpSession(error=True)

        reply = asyncio.run(
            McpOperatorHost(model).handle("暂停2号机床", session)
        )

        self.assertFalse(
            reply.tool_executions[0].protocol_succeeded
        )
        tool_message = next(
            message
            for message in reversed(model.requests[1][0])
            if message["role"] == "tool"
        )
        self.assertIn(
            '"protocol_succeeded": false',
            tool_message["content"],
        )

    def test_invalid_model_arguments_get_one_repair_attempt(self):
        class RepairingClient(OpenAICompatibleToolClient):
            def __init__(self):
                pass

            def complete(self, messages, tools):
                repairs = [
                    message
                    for message in messages
                    if "上一次工具输出格式无效" in message["content"]
                ]
                if not repairs:
                    raise ModelOutputError("bad arguments")
                return ModelTurn("已修正，不执行危险操作。")

        reply = asyncio.run(
            McpOperatorHost(RepairingClient()).handle(
                "测试格式修复",
                FakeMcpSession(),
            )
        )

        self.assertIn("已修正", reply.text)


class OpenAICompatibleToolClientTests(unittest.TestCase):
    def test_decodes_function_tool_call(self):
        message = {
            "content": None,
            "tool_calls": [{
                "id": "call-7",
                "type": "function",
                "function": {
                    "name": "submit_order",
                    "arguments": json.dumps({
                        "quantity": 2,
                        "allowed_machine_ids": ["machine_2"],
                    }),
                },
            }],
        }

        turn = OpenAICompatibleToolClient._decode_message(message)

        self.assertEqual(turn.tool_calls[0].call_id, "call-7")
        self.assertEqual(turn.tool_calls[0].name, "submit_order")
        self.assertEqual(turn.tool_calls[0].arguments["quantity"], 2)

    def test_rejects_non_object_tool_arguments(self):
        message = {
            "tool_calls": [{
                "id": "call-7",
                "type": "function",
                "function": {
                    "name": "submit_order",
                    "arguments": "[2]",
                },
            }],
        }

        with self.assertRaisesRegex(
            ModelOutputError,
            "must decode to an object",
        ):
            OpenAICompatibleToolClient._decode_message(message)

    def test_rejects_non_function_tool_call(self):
        message = {
            "tool_calls": [{
                "id": "call-7",
                "type": "unknown",
                "function": {
                    "name": "submit_order",
                    "arguments": "{}",
                },
            }],
        }

        with self.assertRaisesRegex(
            ModelOutputError,
            "only function",
        ):
            OpenAICompatibleToolClient._decode_message(message)


if __name__ == "__main__":
    unittest.main()
