#!/usr/bin/env python3
"""Exercise a real LLM against MCP schemas without exposing live factory data."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from types import SimpleNamespace

from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client

from factory_agent.operator_host import (
    MUTATING_TOOLS,
    McpOperatorHost,
    OpenAICompatibleToolClient,
)


SYNTHETIC_FACTORY_STATE = {
    "accepted": True,
    "operation": "get_factory_state",
    "message": "synthetic read-only acceptance state",
    "data": {
        "battery": 0.73,
        "raw_inventory": 4,
        "finished_inventory": 2,
        "machines": [
            {"machine_id": "machine_1", "state": "IDLE"},
            {"machine_id": "machine_2", "state": "PROCESSING"},
            {"machine_id": "machine_3", "state": "DONE"},
        ],
    },
}


class SyntheticResultSession:
    """Delegate schema discovery, but never execute a live MCP tool."""

    def __init__(self, schema_session: ClientSession):
        self._schema_session = schema_session
        self.requested_tools: list[str] = []

    async def list_tools(self):
        return await self._schema_session.list_tools()

    async def call_tool(self, name: str, arguments: dict):
        self.requested_tools.append(name)
        payload = (
            SYNTHETIC_FACTORY_STATE
            if name == "get_factory_state"
            else {
                "accepted": False,
                "operation": name,
                "message": "synthetic acceptance forbids this tool",
                "data": {},
            }
        )
        return SimpleNamespace(
            isError=False,
            structuredContent={"result": payload},
            content=[],
        )


async def run(args: argparse.Namespace) -> int:
    model = OpenAICompatibleToolClient(
        base_url=args.base_url,
        model=args.model,
        timeout=args.timeout,
    )
    host = McpOperatorHost(model)
    async with streamable_http_client(args.mcp_url) as streams:
        read_stream, write_stream, _ = streams
        async with ClientSession(read_stream, write_stream) as session:
            await session.initialize()
            synthetic = SyntheticResultSession(session)
            reply = await host.handle(args.instruction, synthetic)

    mutating = sorted(set(synthetic.requested_tools) & MUTATING_TOOLS)
    passed = (
        "get_factory_state" in synthetic.requested_tools
        and not mutating
        and bool(reply.text)
    )
    print(json.dumps({
        "passed": passed,
        "model": args.model,
        "requested_tools": synthetic.requested_tools,
        "mutating_tools": mutating,
        "knowledge_ids": list(reply.knowledge_ids),
        "reply": reply.text,
        "factory_state": "synthetic",
    }, ensure_ascii=False, indent=2))
    return 0 if passed else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mcp-url", default="http://127.0.0.1:8000/mcp")
    parser.add_argument("--base-url", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--instruction",
        default="请查询机床状态、原料库存、成品库存和机械臂电量，不要改变任何任务。",
    )
    return parser.parse_args()


if __name__ == "__main__":
    if not os.getenv("OPENAI_API_KEY"):
        raise SystemExit("OPENAI_API_KEY is not configured")
    raise SystemExit(asyncio.run(run(parse_args())))
