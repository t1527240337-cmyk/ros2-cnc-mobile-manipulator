from __future__ import annotations

import argparse
import asyncio
import json
import os

from .operator_host import (
    McpOperatorHost,
    OpenAICompatibleToolClient,
    OperatorHostError,
)


async def _run(args: argparse.Namespace) -> None:
    try:
        from mcp import ClientSession
        from mcp.client.streamable_http import streamable_http_client
    except ImportError as exc:
        raise OperatorHostError(
            "MCP SDK is not installed; run scripts/setup_mcp_env.sh"
        ) from exc

    model = OpenAICompatibleToolClient(
        base_url=args.base_url,
        model=args.model,
        timeout=args.llm_timeout,
    )
    host = McpOperatorHost(model)

    async with streamable_http_client(args.mcp_url) as (
        read_stream,
        write_stream,
        _,
    ):
        async with ClientSession(read_stream, write_stream) as session:
            await session.initialize()
            await session.send_ping()
            if args.text:
                reply = await host.handle(" ".join(args.text), session)
                _print_reply(reply, args.json)
                return

            print("工厂 Agent 已连接；输入 quit 或 exit 退出。")
            while True:
                try:
                    text = await asyncio.to_thread(input, "operator> ")
                except EOFError:
                    return
                if text.strip().lower() in {"quit", "exit"}:
                    return
                if not text.strip():
                    continue
                try:
                    reply = await host.handle(text, session)
                    _print_reply(reply, args.json)
                except OperatorHostError as exc:
                    print(f"请求失败：{exc}")


def _print_reply(reply, json_output: bool) -> None:
    if json_output:
        print(json.dumps(reply.to_dict(), ensure_ascii=False, indent=2))
        return
    print(reply.text)
    for execution in reply.tool_executions:
        accepted = execution.payload.get("accepted")
        print(
            f"  tool={execution.name} "
            f"protocol_ok={execution.protocol_succeeded} "
            f"accepted={accepted}"
        )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Operate the ROS 2 factory through an LLM and MCP"
    )
    parser.add_argument("text", nargs="*")
    parser.add_argument(
        "--mcp-url",
        default=os.getenv(
            "FACTORY_MCP_URL",
            "http://127.0.0.1:8000/mcp",
        ),
    )
    parser.add_argument(
        "--base-url",
        default=os.getenv(
            "OPENAI_BASE_URL",
            "https://api.openai.com/v1",
        ),
    )
    parser.add_argument(
        "--model",
        default=os.getenv("FACTORY_AGENT_MODEL", "gpt-4.1-mini"),
    )
    parser.add_argument(
        "--llm-timeout",
        type=float,
        default=float(os.getenv("FACTORY_AGENT_LLM_TIMEOUT", "8.0")),
    )
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def _find_operator_error(group: BaseExceptionGroup) -> BaseException:
    for error in group.exceptions:
        if isinstance(error, BaseExceptionGroup):
            nested = _find_operator_error(error)
            if isinstance(nested, OperatorHostError):
                return nested
        elif isinstance(error, OperatorHostError):
            return error
    return group


def main() -> None:
    args = _parse_args()
    try:
        asyncio.run(_run(args))
    except (OperatorHostError, OSError, TimeoutError) as exc:
        raise SystemExit(f"Factory operator failed: {exc}") from exc
    except BaseExceptionGroup as exc:
        cause = _find_operator_error(exc)
        raise SystemExit(
            f"Factory operator failed: {cause}"
        ) from cause


if __name__ == "__main__":
    main()
