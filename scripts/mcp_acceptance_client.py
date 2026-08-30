#!/usr/bin/env python3
"""Protocol-level acceptance client for the factory MCP server."""

from __future__ import annotations

import argparse
import asyncio
import json
import time
from typing import Any

from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client


EXPECTED_TOOLS = {
    "get_factory_state",
    "submit_order",
    "get_task_status",
    "start_automatic",
    "stop_automatic",
    "get_automatic_status",
    "pause_task",
    "resume_task",
    "cancel_task",
    "hold_machine",
    "resume_machine",
    "explain_failure",
    "list_capabilities",
}

FORBIDDEN_TOOL_FRAGMENTS = {
    "cmd_vel",
    "joint",
    "trajectory",
    "spindle",
    "door_motor",
    "emergency_stop",
}


class AcceptanceFailure(RuntimeError):
    """Raised when an MCP protocol or factory-domain assertion fails."""


def _result_is_error(result: Any) -> bool:
    return bool(
        getattr(
            result,
            "isError",
            getattr(result, "is_error", False),
        )
    )


def _text_content(result: Any) -> str:
    texts = [
        item.text
        for item in getattr(result, "content", [])
        if hasattr(item, "text")
    ]
    return "\n".join(texts)


def _result_payload(result: Any) -> dict:
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

    text = _text_content(result)
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise AcceptanceFailure(
            f"tool response is not structured JSON: {text!r}"
        ) from exc
    if not isinstance(payload, dict):
        raise AcceptanceFailure(
            f"tool response must be an object, got {type(payload).__name__}"
        )
    return payload


async def _call_json(
    session: ClientSession,
    tool_name: str,
    arguments: dict | None = None,
) -> dict:
    result = await session.call_tool(tool_name, arguments=arguments or {})
    if _result_is_error(result):
        raise AcceptanceFailure(
            f"{tool_name} returned MCP error: {_text_content(result)}"
        )
    return _result_payload(result)


def _require_accepted(payload: dict, operation: str) -> None:
    if payload.get("accepted") is not True:
        raise AcceptanceFailure(
            f"{operation} was rejected: {payload.get('message', payload)}"
        )


def _finished_count(state_payload: dict) -> int:
    inventory = state_payload.get("data", {}).get("inventory", {})
    value = inventory.get("finished_parts")
    if not isinstance(value, int):
        raise AcceptanceFailure(
            f"factory state has no integer finished_parts: {state_payload}"
        )
    return value


def _verify_public_tool_schema(tools: list[Any]) -> None:
    by_name = {tool.name: tool for tool in tools}
    submit_schema = getattr(
        by_name["submit_order"],
        "inputSchema",
        getattr(by_name["submit_order"], "input_schema", {}),
    )
    quantity_schema = submit_schema.get(
        "properties",
        {},
    ).get("quantity", {})
    if (
        quantity_schema.get("minimum") != 1
        or quantity_schema.get("maximum") != 6
    ):
        raise AcceptanceFailure(
            f"submit_order quantity bounds missing: {quantity_schema}"
        )


async def _verify_invalid_arguments(session: ClientSession) -> None:
    result = await session.call_tool(
        "submit_order",
        arguments={"quantity": 7},
    )
    if not _result_is_error(result):
        raise AcceptanceFailure(
            "quantity=7 escaped MCP/Pydantic validation"
        )


async def _wait_for_order(
    session: ClientSession,
    order_id: str,
    target_finished: int,
    timeout_sec: float,
) -> tuple[dict, dict]:
    deadline = time.monotonic() + timeout_sec
    last_state: dict = {}
    last_status: dict = {}
    while time.monotonic() < deadline:
        last_state = await _call_json(session, "get_factory_state")
        _require_accepted(last_state, "get_factory_state")
        last_status = await _call_json(
            session,
            "get_task_status",
            {"task_id": order_id},
        )
        _require_accepted(last_status, "get_task_status")
        phase = last_status.get("data", {}).get("phase")
        if (
            _finished_count(last_state) >= target_finished
            and phase == "complete"
        ):
            return last_state, last_status
        await asyncio.sleep(0.1)
    raise AcceptanceFailure(
        f"order did not finish within {timeout_sec:.1f}s; "
        f"last state={last_state}; last status={last_status}"
    )


async def run_acceptance(args: argparse.Namespace) -> None:
    async with streamable_http_client(args.url) as (
        read_stream,
        write_stream,
        _,
    ):
        async with ClientSession(read_stream, write_stream) as session:
            await session.initialize()
            await session.send_ping()

            listed = await session.list_tools()
            tool_names = {tool.name for tool in listed.tools}
            if tool_names != EXPECTED_TOOLS:
                raise AcceptanceFailure(
                    "tool allow-list mismatch: "
                    f"expected={sorted(EXPECTED_TOOLS)} "
                    f"actual={sorted(tool_names)}"
                )
            lower_names = " ".join(tool_names).lower()
            leaked = sorted(
                fragment
                for fragment in FORBIDDEN_TOOL_FRAGMENTS
                if fragment in lower_names
            )
            if leaked:
                raise AcceptanceFailure(
                    f"low-level tools leaked through MCP: {leaked}"
                )
            _verify_public_tool_schema(list(listed.tools))

            state = await _call_json(session, "get_factory_state")
            _require_accepted(state, "get_factory_state")
            baseline_finished = _finished_count(state)

            capabilities = await _call_json(
                session,
                "list_capabilities",
            )
            _require_accepted(capabilities, "list_capabilities")
            capability_text = json.dumps(
                capabilities,
                ensure_ascii=False,
            ).lower()
            leaked = sorted(
                fragment
                for fragment in FORBIDDEN_TOOL_FRAGMENTS
                if fragment in capability_text
            )
            if leaked:
                raise AcceptanceFailure(
                    f"low-level interfaces leaked in capabilities: {leaked}"
                )

            if args.smoke_only:
                print(
                    "mcp_protocol_smoke_ok "
                    f"tools={len(tool_names)} "
                    f"finished_parts={baseline_finished}"
                )
                return

            await _verify_invalid_arguments(session)

            order = await _call_json(
                session,
                "submit_order",
                {
                    "quantity": args.quantity,
                    "allowed_machine_ids": [args.machine],
                    "auto_recharge": True,
                },
            )
            _require_accepted(order, "submit_order")
            order_id = order.get("order_id")
            if not isinstance(order_id, str) or not order_id:
                raise AcceptanceFailure(
                    f"submit_order returned no order_id: {order}"
                )

            _, status = await _wait_for_order(
                session,
                order_id,
                baseline_finished + args.quantity,
                args.timeout,
            )

            missing = await _call_json(
                session,
                "get_task_status",
                {"task_id": "missing-task"},
            )
            if missing.get("accepted") is not False:
                raise AcceptanceFailure(
                    f"unknown task id was not rejected: {missing}"
                )

            print(
                "mcp_ros_acceptance_ok "
                f"tools={len(tool_names)} "
                f"order_id={order_id} "
                f"finished_parts={baseline_finished + args.quantity}"
            )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate the factory MCP protocol and ROS execution path"
    )
    parser.add_argument(
        "--url",
        default="http://127.0.0.1:8000/mcp",
    )
    parser.add_argument("--smoke-only", action="store_true")
    parser.add_argument("--quantity", type=int, default=2)
    parser.add_argument("--machine", default="machine_2")
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def _first_acceptance_failure(group: BaseExceptionGroup) -> BaseException:
    for error in group.exceptions:
        if isinstance(error, BaseExceptionGroup):
            nested = _first_acceptance_failure(error)
            if isinstance(nested, AcceptanceFailure):
                return nested
        elif isinstance(error, AcceptanceFailure):
            return error
    return group


def main() -> None:
    args = _parse_args()
    try:
        asyncio.run(run_acceptance(args))
    except (AcceptanceFailure, OSError, TimeoutError) as exc:
        raise SystemExit(f"MCP acceptance failed: {exc}") from exc
    except BaseExceptionGroup as exc:
        cause = _first_acceptance_failure(exc)
        raise SystemExit(
            f"MCP acceptance failed: {cause}"
        ) from cause


if __name__ == "__main__":
    main()
