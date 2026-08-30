#!/usr/bin/env python3
"""Deterministic OpenAI-compatible server used by Agent integration tests."""

from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

from factory_agent.mcp_tools import MCP_TOOL_NAMES


def _completion(message: dict[str, Any], finish_reason: str) -> dict:
    return {
        "id": "chatcmpl-factory-test",
        "object": "chat.completion",
        "model": "factory-fake-tool-model",
        "choices": [{
            "index": 0,
            "message": {"role": "assistant", **message},
            "finish_reason": finish_reason,
        }],
        "usage": {
            "prompt_tokens": 1,
            "completion_tokens": 1,
            "total_tokens": 2,
        },
    }


class FakeToolCallingHandler(BaseHTTPRequestHandler):
    server_version = "FactoryFakeLLM/1.0"

    def do_GET(self) -> None:
        if self.path != "/health":
            self.send_error(404)
            return
        self._send_json(200, {"status": "ok"})

    def do_POST(self) -> None:
        if self.path != "/v1/chat/completions":
            self.send_error(404)
            return
        try:
            payload = self._read_payload()
            response = self._respond(payload)
        except (ValueError, KeyError, json.JSONDecodeError) as exc:
            self._send_json(400, {"error": {"message": str(exc)}})
            return
        self._send_json(200, response)

    def _read_payload(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if length <= 0 or length > 1_000_000:
            raise ValueError("request body length is invalid")
        payload = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("request body must be a JSON object")
        return payload

    def _respond(self, payload: dict[str, Any]) -> dict:
        tool_names = {
            item["function"]["name"]
            for item in payload["tools"]
            if item.get("type") == "function"
        }
        expected = set(MCP_TOOL_NAMES)
        if tool_names != expected:
            raise ValueError(
                "model did not receive the exact MCP tool allow-list"
            )

        messages = payload["messages"]
        tool_messages = [
            message
            for message in messages
            if message.get("role") == "tool"
        ]
        if not tool_messages:
            return _completion(
                {
                    "content": None,
                    "tool_calls": [{
                        "id": "call-submit-order",
                        "type": "function",
                        "function": {
                            "name": "submit_order",
                            "arguments": json.dumps({
                                "quantity": 2,
                                "allowed_machine_ids": ["machine_2"],
                                "auto_recharge": True,
                            }),
                        },
                    }],
                },
                "tool_calls",
            )

        tool_result = json.loads(tool_messages[-1]["content"])
        result = tool_result.get("result", {})
        if (
            tool_result.get("protocol_succeeded") is not True or
            result.get("accepted") is not True
        ):
            text = (
                "订单没有被执行器接受："
                f"{result.get('message', '工具调用失败')}"
            )
        else:
            text = (
                "订单已经通过 MCP 提交，"
                f"订单 ID 为 {result.get('order_id')}。"
            )
        return _completion({"content": text}, "stop")

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, _format: str, *_args) -> None:
        return


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18081)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    server = ThreadingHTTPServer(
        (args.host, args.port),
        FakeToolCallingHandler,
    )
    print(
        f"fake_openai_tool_server_ready "
        f"url=http://{args.host}:{args.port}/v1",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
