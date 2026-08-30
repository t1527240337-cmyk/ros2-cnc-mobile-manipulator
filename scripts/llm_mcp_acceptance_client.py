#!/usr/bin/env python3
"""Exercise natural language through an LLM, MCP and ROS order executor."""

from __future__ import annotations

import argparse
import asyncio
import time

from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client

from factory_agent.operator_host import (
    McpOperatorHost,
    OpenAICompatibleToolClient,
    OperatorHostError,
    decode_mcp_result,
    mcp_result_is_error,
)


class AcceptanceFailure(RuntimeError):
    pass


async def _call_json(
    session: ClientSession,
    name: str,
    arguments: dict | None = None,
) -> dict:
    result = await session.call_tool(name, arguments=arguments or {})
    if mcp_result_is_error(result):
        raise AcceptanceFailure(f"{name} returned an MCP protocol error")
    payload = decode_mcp_result(result)
    if payload.get("accepted") is not True:
        raise AcceptanceFailure(
            f"{name} was rejected: {payload.get('message', payload)}"
        )
    return payload


async def _wait_for_completion(
    session: ClientSession,
    order_id: str,
    target_finished: int,
    timeout_sec: float,
) -> None:
    deadline = time.monotonic() + timeout_sec
    last_state = {}
    last_status = {}
    while time.monotonic() < deadline:
        last_state = await _call_json(session, "get_factory_state")
        last_status = await _call_json(
            session,
            "get_task_status",
            {"task_id": order_id},
        )
        inventory = last_state.get("data", {}).get("inventory", {})
        phase = last_status.get("data", {}).get("phase")
        if (
            inventory.get("finished_parts") == target_finished and
            phase == "complete"
        ):
            return
        await asyncio.sleep(0.1)
    raise AcceptanceFailure(
        f"LLM-submitted order did not finish; "
        f"state={last_state}; status={last_status}"
    )


async def run(args: argparse.Namespace) -> None:
    model = OpenAICompatibleToolClient(
        api_key="local-acceptance-key",
        base_url=args.llm_base_url,
        model="factory-fake-tool-model",
        timeout=5.0,
    )
    host = McpOperatorHost(model)

    async with streamable_http_client(args.mcp_url) as (
        read_stream,
        write_stream,
        _,
    ):
        async with ClientSession(read_stream, write_stream) as session:
            await session.initialize()
            state = await _call_json(session, "get_factory_state")
            baseline = state["data"]["inventory"]["finished_parts"]

            reply = await host.handle(
                "请只使用2号机床加工2个零件，并告诉我订单编号",
                session,
            )
            if len(reply.tool_executions) != 1:
                raise AcceptanceFailure(
                    "model did not make exactly one bounded tool call"
                )
            execution = reply.tool_executions[0]
            if execution.name != "submit_order":
                raise AcceptanceFailure(
                    f"model selected the wrong tool: {execution.name}"
                )
            if execution.arguments != {
                "quantity": 2,
                "allowed_machine_ids": ["machine_2"],
                "auto_recharge": True,
            }:
                raise AcceptanceFailure(
                    f"model changed the order constraints: "
                    f"{execution.arguments}"
                )
            if (
                not execution.protocol_succeeded or
                execution.payload.get("accepted") is not True
            ):
                raise AcceptanceFailure(
                    f"LLM order was not accepted: {execution.payload}"
                )
            order_id = execution.payload.get("order_id", "")
            if not order_id or order_id not in reply.text:
                raise AcceptanceFailure(
                    "final model reply did not preserve the order id"
                )

            await _wait_for_completion(
                session,
                order_id,
                baseline + 2,
                args.timeout,
            )
            print(
                "llm_mcp_ros_acceptance_ok "
                f"tool={execution.name} "
                f"order_id={order_id} "
                f"finished_parts={baseline + 2}"
            )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mcp-url",
        default="http://127.0.0.1:8013/mcp",
    )
    parser.add_argument(
        "--llm-base-url",
        default="http://127.0.0.1:18081/v1",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    try:
        asyncio.run(run(args))
    except (AcceptanceFailure, OperatorHostError) as exc:
        raise SystemExit(f"LLM MCP acceptance failed: {exc}") from exc


if __name__ == "__main__":
    main()
