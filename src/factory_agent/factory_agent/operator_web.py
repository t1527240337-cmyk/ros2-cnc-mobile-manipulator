from __future__ import annotations

import argparse
import asyncio
import json
import os
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
from pathlib import PurePosixPath
from typing import Any, Awaitable, Callable
from urllib.parse import urlsplit

from .operator_host import (
    McpOperatorHost,
    OpenAICompatibleToolClient,
    OperatorHostError,
)


MAX_REQUEST_BYTES = 32_768
ASSET_TYPES = {
    ".css": "text/css; charset=utf-8",
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
}


@dataclass(frozen=True)
class OperatorWebConfig:
    mcp_url: str = "http://127.0.0.1:8000/mcp"
    model: str = "gpt-4.1-mini"
    llm_base_url: str = "https://api.openai.com/v1"
    llm_timeout: float = 8.0


ChatHandler = Callable[[str], Awaitable[dict[str, Any]]]


class OperatorWebApplication:
    """Owns local UI assets and the LLM-to-MCP chat operation."""

    def __init__(
        self,
        config: OperatorWebConfig,
        chat_handler: ChatHandler | None = None,
    ):
        self.config = config
        self._chat_handler = chat_handler

    def health(self) -> dict[str, Any]:
        return {
            "status": "ready",
            "llm_configured": bool(os.getenv("OPENAI_API_KEY")),
            "model": self.config.model,
            "mcp_url": self.config.mcp_url,
        }

    def read_asset(self, name: str) -> tuple[bytes, str]:
        path = PurePosixPath(name)
        if (
            path.name != name
            or path.suffix not in ASSET_TYPES
            or name not in {"index.html", "app.css", "app.js"}
        ):
            raise FileNotFoundError(name)
        resource = files("factory_agent").joinpath("web", name)
        return resource.read_bytes(), ASSET_TYPES[path.suffix]

    async def chat(self, message: str) -> dict[str, Any]:
        normalized = message.strip()
        if not normalized:
            raise ValueError("message cannot be empty")
        if len(normalized) > 2_000:
            raise ValueError("message exceeds 2000 characters")
        if self._chat_handler is not None:
            return await self._chat_handler(normalized)
        return await self._chat_with_mcp(normalized)

    async def _chat_with_mcp(self, message: str) -> dict[str, Any]:
        try:
            from mcp import ClientSession
            from mcp.client.streamable_http import (
                streamable_http_client,
            )
        except ImportError as exc:
            raise OperatorHostError(
                "MCP SDK is not installed; run scripts/setup_mcp_env.sh"
            ) from exc

        model = OpenAICompatibleToolClient(
            base_url=self.config.llm_base_url,
            model=self.config.model,
            timeout=self.config.llm_timeout,
        )
        host = McpOperatorHost(model)
        async with streamable_http_client(self.config.mcp_url) as (
            read_stream,
            write_stream,
            _,
        ):
            async with ClientSession(read_stream, write_stream) as session:
                await session.initialize()
                reply = await host.handle(message, session)
                return reply.to_dict()


class FactoryOperatorHttpHandler(BaseHTTPRequestHandler):
    server_version = "FactoryOperator/1.0"

    @property
    def application(self) -> OperatorWebApplication:
        return self.server.application

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/api/health":
            self._send_json(200, self.application.health())
            return
        asset_name = {
            "/": "index.html",
            "/index.html": "index.html",
            "/app.css": "app.css",
            "/app.js": "app.js",
        }.get(path)
        if asset_name is None:
            self._send_json(404, {"error": "not found"})
            return
        try:
            body, content_type = self.application.read_asset(asset_name)
        except FileNotFoundError:
            self._send_json(404, {"error": "asset not found"})
            return
        self._send(200, body, content_type)

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        if path != "/api/chat":
            self._send_json(404, {"error": "not found"})
            return
        try:
            payload = self._read_json()
            message = payload.get("message")
            if not isinstance(message, str):
                raise ValueError("message must be text")
            result = asyncio.run(self.application.chat(message))
        except (ValueError, json.JSONDecodeError) as exc:
            self._send_json(400, {"error": str(exc)})
            return
        except BaseExceptionGroup as exc:
            error = _find_operator_error(exc)
            self._send_json(502, {"error": str(error)})
            return
        except (OperatorHostError, OSError, TimeoutError) as exc:
            self._send_json(502, {"error": str(exc)})
            return
        self._send_json(200, result)

    def _read_json(self) -> dict[str, Any]:
        content_type = self.headers.get("Content-Type", "")
        if "application/json" not in content_type:
            raise ValueError("Content-Type must be application/json")
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if length <= 0 or length > MAX_REQUEST_BYTES:
            raise ValueError("request body length is invalid")
        payload = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("request body must be a JSON object")
        return payload

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._send(status, body, "application/json; charset=utf-8")

    def _send(
        self,
        status: int,
        body: bytes,
        content_type: str,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; "
            "script-src 'self'; "
            "style-src 'self'; "
            "connect-src 'self'; "
            "img-src 'self' data:; "
            "frame-ancestors 'none'",
        )
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format_string: str, *args) -> None:
        if getattr(self.server, "quiet", False):
            return
        super().log_message(format_string, *args)


class FactoryOperatorHttpServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        address: tuple[str, int],
        application: OperatorWebApplication,
        quiet: bool = False,
    ):
        super().__init__(address, FactoryOperatorHttpHandler)
        self.application = application
        self.quiet = quiet


def _find_operator_error(group: BaseExceptionGroup) -> BaseException:
    for error in group.exceptions:
        if isinstance(error, BaseExceptionGroup):
            nested = _find_operator_error(error)
            if isinstance(nested, OperatorHostError):
                return nested
        elif isinstance(error, OperatorHostError):
            return error
    return group


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve the local factory operator web console"
    )
    parser.add_argument(
        "--host",
        default=os.getenv("FACTORY_OPERATOR_WEB_HOST", "127.0.0.1"),
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.getenv("FACTORY_OPERATOR_WEB_PORT", "8080")),
    )
    parser.add_argument(
        "--allow-remote",
        action="store_true",
        help="allow binding to a non-loopback interface",
    )
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
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.host not in {"127.0.0.1", "localhost", "::1"}:
        if not args.allow_remote:
            raise SystemExit(
                "Refusing a remote bind without --allow-remote"
            )
    config = OperatorWebConfig(
        mcp_url=args.mcp_url,
        model=args.model,
        llm_base_url=args.base_url,
        llm_timeout=args.llm_timeout,
    )
    server = FactoryOperatorHttpServer(
        (args.host, args.port),
        OperatorWebApplication(config),
        quiet=args.quiet,
    )
    print(
        f"factory_operator_web_ready "
        f"url=http://{args.host}:{args.port}",
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
