from __future__ import annotations

import asyncio
import json
import os
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass
from typing import Any, Protocol

from .knowledge import KnowledgeBase
from .mcp_tools import MCP_TOOL_NAMES


OPERATOR_SYSTEM_PROMPT = """你是 ROS 2 工厂移动机械臂的操作员 Agent。
你只能通过本轮提供的高层工具查询和操作工厂，不能声称自己执行了未调用的动作。
涉及实时状态、库存、电量、任务进度时必须调用查询工具，不能根据对话历史猜测。
提交订单后，只有工具结果 accepted=true 才能告诉用户订单已接受。
同一次用户请求最多执行一个会改变工厂状态的工具；查询工具可以组合使用。
状态变更执行后，只能继续查询结果或向用户回复，不能再请求其他状态变更。
你无权控制底盘速度、机械臂关节、夹爪电机、门电机、PLC 输出、主轴或急停。
用户提出越权或危险要求时，说明权限边界并建议使用正确的安全流程。
请使用简洁中文回答，并保留工具返回的订单 ID、机床 ID 和错误原因。"""

MUTATING_TOOLS = frozenset({
    "submit_order",
    "start_automatic",
    "stop_automatic",
    "pause_task",
    "resume_task",
    "cancel_task",
    "hold_machine",
    "resume_machine",
})


class OperatorHostError(RuntimeError):
    """Raised when model output or the connected MCP server violates policy."""


class ModelOutputError(OperatorHostError):
    """Raised when an OpenAI-compatible response cannot be decoded safely."""


@dataclass(frozen=True)
class ToolCall:
    call_id: str
    name: str
    arguments: dict[str, Any]


@dataclass(frozen=True)
class ModelTurn:
    text: str
    tool_calls: tuple[ToolCall, ...] = ()


@dataclass(frozen=True)
class ToolExecution:
    call_id: str
    name: str
    arguments: dict[str, Any]
    protocol_succeeded: bool
    payload: dict[str, Any]


@dataclass(frozen=True)
class OperatorReply:
    text: str
    knowledge_ids: tuple[str, ...]
    tool_executions: tuple[ToolExecution, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "reply": self.text,
            "knowledge_ids": list(self.knowledge_ids),
            "tool_executions": [
                asdict(execution) for execution in self.tool_executions
            ],
        }


class ToolCallingModel(Protocol):
    def complete(
        self,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]],
    ) -> ModelTurn:
        ...


class OpenAICompatibleToolClient:
    """Small Chat Completions tool-calling client without provider lock-in."""

    def __init__(
        self,
        api_key: str | None = None,
        base_url: str | None = None,
        model: str | None = None,
        timeout: float = 8.0,
    ):
        self.api_key = api_key or os.getenv("OPENAI_API_KEY", "")
        self.base_url = (
            base_url or
            os.getenv("OPENAI_BASE_URL", "https://api.openai.com/v1")
        ).rstrip("/")
        self.model = model or os.getenv(
            "FACTORY_AGENT_MODEL",
            "gpt-4.1-mini",
        )
        self.timeout = timeout

    @property
    def configured(self) -> bool:
        return bool(self.api_key)

    def complete(
        self,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]],
    ) -> ModelTurn:
        if not self.configured:
            raise OperatorHostError("OPENAI_API_KEY is not configured")
        payload = {
            "model": self.model,
            "temperature": 0,
            "messages": messages,
            "tools": tools,
            "tool_choice": "auto",
        }
        response = self._request_json(payload)
        try:
            message = response["choices"][0]["message"]
        except (KeyError, IndexError, TypeError) as exc:
            raise ModelOutputError(
                "model response has no choices[0].message"
            ) from exc
        return self._decode_message(message)

    def _request_json(self, payload: dict[str, Any]) -> dict[str, Any]:
        request = urllib.request.Request(
            f"{self.base_url}/chat/completions",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request,
                timeout=self.timeout,
            ) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")[:500]
            raise OperatorHostError(
                f"LLM HTTP {exc.code}: {detail}"
            ) from exc
        except (urllib.error.URLError, TimeoutError) as exc:
            raise OperatorHostError(f"LLM request failed: {exc}") from exc
        except json.JSONDecodeError as exc:
            raise ModelOutputError(
                "LLM response body is not JSON"
            ) from exc

    @staticmethod
    def _decode_message(message: dict[str, Any]) -> ModelTurn:
        content = message.get("content") or ""
        if not isinstance(content, str):
            raise ModelOutputError("assistant content must be text or null")

        calls = []
        raw_calls = message.get("tool_calls") or []
        if not isinstance(raw_calls, list):
            raise ModelOutputError("assistant tool_calls must be a list")
        for raw_call in raw_calls:
            if raw_call.get("type") != "function":
                raise ModelOutputError(
                    "only function tool calls are supported"
                )
            try:
                call_id = raw_call["id"]
                function = raw_call["function"]
                name = function["name"]
                raw_arguments = function["arguments"]
            except (KeyError, TypeError) as exc:
                raise ModelOutputError(
                    "tool call is missing id, name or arguments"
                ) from exc
            if not isinstance(call_id, str) or not call_id:
                raise ModelOutputError("tool call id must be non-empty text")
            if not isinstance(name, str) or not name:
                raise ModelOutputError("tool name must be non-empty text")
            if not isinstance(raw_arguments, str):
                raise ModelOutputError("tool arguments must be JSON text")
            try:
                arguments = json.loads(raw_arguments)
            except json.JSONDecodeError as exc:
                raise ModelOutputError(
                    f"{name} arguments are not valid JSON"
                ) from exc
            if not isinstance(arguments, dict):
                raise ModelOutputError(
                    f"{name} arguments must decode to an object"
                )
            calls.append(ToolCall(call_id, name, arguments))
        return ModelTurn(content.strip(), tuple(calls))


def mcp_result_is_error(result: Any) -> bool:
    return bool(
        getattr(
            result,
            "isError",
            getattr(result, "is_error", False),
        )
    )


def mcp_result_text(result: Any) -> str:
    return "\n".join(
        item.text
        for item in getattr(result, "content", [])
        if hasattr(item, "text")
    )


def decode_mcp_result(result: Any) -> dict[str, Any]:
    structured = getattr(
        result,
        "structuredContent",
        getattr(result, "structured_content", None),
    )
    if isinstance(structured, dict):
        if set(structured) == {"result"} and isinstance(
            structured["result"], dict
        ):
            return structured["result"]
        return structured

    text = mcp_result_text(result)
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        return {"message": text}
    if isinstance(payload, dict):
        return payload
    return {"result": payload}


class McpOperatorHost:
    """Runs a bounded LLM/tool loop against one allow-listed MCP session."""

    def __init__(
        self,
        model: ToolCallingModel,
        knowledge: KnowledgeBase | None = None,
        max_model_turns: int = 4,
        max_tool_calls: int = 6,
    ):
        if max_model_turns < 1:
            raise ValueError("max_model_turns must be positive")
        if max_tool_calls < 1:
            raise ValueError("max_tool_calls must be positive")
        self.model = model
        self.knowledge = knowledge or KnowledgeBase()
        self.max_model_turns = max_model_turns
        self.max_tool_calls = max_tool_calls

    async def handle(self, text: str, session: Any) -> OperatorReply:
        normalized = text.strip()
        if not normalized:
            raise OperatorHostError("operator instruction cannot be empty")

        tool_definitions = await self._load_tool_definitions(session)
        entries = self.knowledge.retrieve(normalized)
        knowledge_ids = tuple(entry.entry_id for entry in entries)
        knowledge_context = self.knowledge.format_context(entries)
        messages: list[dict[str, Any]] = [
            {
                "role": "system",
                "content": (
                    f"{OPERATOR_SYSTEM_PROMPT}\n\n"
                    f"本轮检索到的 SOP：\n{knowledge_context}"
                ),
            },
            {"role": "user", "content": normalized},
        ]
        executions: list[ToolExecution] = []
        repaired = False

        for _ in range(self.max_model_turns):
            mutation_executed = any(
                execution.name in MUTATING_TOOLS
                for execution in executions
            )
            available_tools = self._available_tools(
                tool_definitions,
                mutation_executed,
            )
            try:
                turn = await asyncio.to_thread(
                    self.model.complete,
                    messages,
                    available_tools,
                )
            except ModelOutputError as exc:
                if repaired:
                    raise
                repaired = True
                messages.append({
                    "role": "system",
                    "content": (
                        f"上一次工具输出格式无效：{exc}。"
                        "请只使用提供的工具 Schema 修正一次。"
                    ),
                })
                continue

            if not turn.tool_calls:
                if not turn.text:
                    raise ModelOutputError(
                        "model returned neither text nor tool calls"
                    )
                return OperatorReply(
                    turn.text,
                    knowledge_ids,
                    tuple(executions),
                )

            self._validate_turn(
                turn,
                len(executions),
                mutation_executed,
            )
            messages.append(self._assistant_tool_message(turn))
            for call in turn.tool_calls:
                result = await session.call_tool(
                    call.name,
                    arguments=call.arguments,
                )
                protocol_succeeded = not mcp_result_is_error(result)
                payload = decode_mcp_result(result)
                execution = ToolExecution(
                    call.call_id,
                    call.name,
                    call.arguments,
                    protocol_succeeded,
                    payload,
                )
                executions.append(execution)
                messages.append({
                    "role": "tool",
                    "tool_call_id": call.call_id,
                    "name": call.name,
                    "content": json.dumps(
                        {
                            "protocol_succeeded": protocol_succeeded,
                            "result": payload,
                        },
                        ensure_ascii=False,
                    ),
                })

            if any(
                call.name in MUTATING_TOOLS
                for call in turn.tool_calls
            ):
                messages.append({
                    "role": "system",
                    "content": (
                        "本次用户请求的状态变更工具预算已使用。"
                        "后续只能查询执行结果或直接回复用户。"
                    ),
                })

        raise OperatorHostError(
            f"model did not produce a final reply in "
            f"{self.max_model_turns} turns"
        )

    async def _load_tool_definitions(
        self,
        session: Any,
    ) -> list[dict[str, Any]]:
        listed = await session.list_tools()
        tools = list(getattr(listed, "tools", []))
        names = {str(tool.name) for tool in tools}
        expected = set(MCP_TOOL_NAMES)
        if names != expected:
            raise OperatorHostError(
                "MCP tool allow-list mismatch: "
                f"expected={sorted(expected)} actual={sorted(names)}"
            )

        definitions = []
        for tool in sorted(tools, key=lambda item: item.name):
            schema = getattr(
                tool,
                "inputSchema",
                getattr(tool, "input_schema", None),
            )
            if not isinstance(schema, dict):
                raise OperatorHostError(
                    f"MCP tool {tool.name} has no JSON input schema"
                )
            definitions.append({
                "type": "function",
                "function": {
                    "name": tool.name,
                    "description": tool.description or "",
                    "parameters": schema,
                },
            })
        return definitions

    def _validate_turn(
        self,
        turn: ModelTurn,
        previous_call_count: int,
        mutation_executed: bool = False,
    ) -> None:
        if previous_call_count + len(turn.tool_calls) > self.max_tool_calls:
            raise OperatorHostError("model exceeded the tool-call budget")

        mutating = [
            call.name
            for call in turn.tool_calls
            if call.name in MUTATING_TOOLS
        ]
        if len(mutating) > 1:
            raise OperatorHostError(
                "model requested multiple state-changing tools in one turn"
            )
        if mutation_executed and mutating:
            raise OperatorHostError(
                "model requested more than one state-changing tool "
                "in one operator request"
            )

        for call in turn.tool_calls:
            if call.name not in MCP_TOOL_NAMES:
                raise OperatorHostError(
                    f"model requested forbidden or unknown tool: {call.name}"
                )

    @staticmethod
    def _available_tools(
        tool_definitions: list[dict[str, Any]],
        mutation_executed: bool,
    ) -> list[dict[str, Any]]:
        if not mutation_executed:
            return tool_definitions
        return [
            definition
            for definition in tool_definitions
            if definition["function"]["name"] not in MUTATING_TOOLS
        ]

    @staticmethod
    def _assistant_tool_message(turn: ModelTurn) -> dict[str, Any]:
        return {
            "role": "assistant",
            "content": turn.text or None,
            "tool_calls": [
                {
                    "id": call.call_id,
                    "type": "function",
                    "function": {
                        "name": call.name,
                        "arguments": json.dumps(
                            call.arguments,
                            ensure_ascii=False,
                        ),
                    },
                }
                for call in turn.tool_calls
            ],
        }
