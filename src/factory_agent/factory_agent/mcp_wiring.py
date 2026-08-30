from __future__ import annotations

from typing import Optional, Type

from .mcp_tools import FactoryMcpTools, MCP_TOOL_NAMES


def create_mcp_server(
    tools: FactoryMcpTools,
    fast_mcp_type: Optional[Type] = None,
    host: str = "127.0.0.1",
    port: int = 8000,
):
    """Register the explicit tool allow-list on a FastMCP server."""

    if fast_mcp_type is None:
        try:
            from mcp.server.fastmcp import FastMCP
        except ImportError as exc:
            raise RuntimeError(
                "MCP SDK is not installed; run scripts/setup_mcp_env.sh"
            ) from exc
        fast_mcp_type = FastMCP

    server = fast_mcp_type(
        "ROS 2 Factory Operator",
        instructions=(
            "Use only these high-level tools. Queries execute immediately and "
            "never enter the robot production queue. Production, task and "
            "machine controls are validated again by the ROS Agent service. "
            "No velocity, joint, PLC, spindle or emergency-stop interface is exposed."
        ),
        json_response=True,
        stateless_http=True,
        host=host,
        port=port,
    )
    for tool_name in MCP_TOOL_NAMES:
        server.tool(name=tool_name)(getattr(tools, tool_name))
    return server
