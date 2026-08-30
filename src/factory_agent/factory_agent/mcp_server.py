from __future__ import annotations

import argparse
import os
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from factory_interfaces.srv import ExecuteAgentCommand

from .command_models import AgentCommand
from .mcp_tools import FactoryMcpTools, decode_json_payload
from .mcp_wiring import create_mcp_server


class RosAgentCommandExecutor:
    """Thread-safe client for the single high-level Agent ROS service."""

    def __init__(
        self,
        service_name: str = "/factory_agent/command",
        timeout_sec: float = 5.0,
    ):
        self._timeout_sec = timeout_sec
        self._lock = threading.Lock()
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(args=[])

        self._node = Node("factory_mcp_gateway")
        self._client = self._node.create_client(
            ExecuteAgentCommand,
            service_name,
        )
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name="factory-mcp-ros",
            daemon=True,
        )
        self._spin_thread.start()

    def __call__(self, command: AgentCommand, source: str) -> dict:
        with self._lock:
            if not self._client.wait_for_service(timeout_sec=self._timeout_sec):
                raise RuntimeError("factory Agent ROS service is unavailable")

            request = ExecuteAgentCommand.Request()
            request.source = source
            request.operation = command.operation.value
            request.quantity = command.quantity or 0
            request.allowed_machine_ids = list(command.allowed_machine_ids)
            request.target_machine_id = command.target_machine_id or ""
            request.task_id = command.task_id or ""
            request.auto_recharge = command.auto_recharge
            request.query = command.query or ""

            future = self._client.call_async(request)
            finished = threading.Event()
            future.add_done_callback(lambda _future: finished.set())
            if not finished.wait(self._timeout_sec):
                future.cancel()
                raise TimeoutError("factory Agent ROS service timed out")
            try:
                response = future.result()
            except Exception as exc:
                raise RuntimeError(
                    f"factory Agent ROS request failed: {exc}"
                ) from exc

        data = decode_json_payload(response.data_json)
        return {
            "accepted": response.accepted,
            "request_id": response.request_id,
            "order_id": response.order_id,
            "operation": response.operation,
            "source": response.parser,
            "message": response.message,
            "data": data,
        }

    def close(self) -> None:
        self._executor.shutdown(timeout_sec=1.0)
        self._spin_thread.join(timeout=1.0)
        self._node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()




def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Expose the constrained ROS 2 factory Agent through MCP"
    )
    parser.add_argument(
        "--transport",
        choices=("stdio", "sse", "streamable-http"),
        default=os.getenv(
            "FACTORY_MCP_TRANSPORT",
            "streamable-http",
        ),
    )
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=float(os.getenv("FACTORY_MCP_SERVICE_TIMEOUT", "5.0")),
    )
    parser.add_argument(
        "--host",
        default=os.getenv("FACTORY_MCP_HOST", "127.0.0.1"),
        help="Streamable HTTP/SSE bind address",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.getenv("FACTORY_MCP_PORT", "8000")),
        help="Streamable HTTP/SSE bind port",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    ros_executor = RosAgentCommandExecutor(timeout_sec=args.service_timeout)
    try:
        tools = FactoryMcpTools(ros_executor)
        server = create_mcp_server(
            tools,
            host=args.host,
            port=args.port,
        )
        server.run(transport=args.transport)
    finally:
        ros_executor.close()


if __name__ == "__main__":
    main()
