#!/usr/bin/env python3
"""Wait for observable factory completion and persist the final ROS state."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node

from factory_interfaces.srv import GetFactoryState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--minimum-finished", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def state_payload(response) -> dict:
    return {
        "raw_parts": response.raw_part_count,
        "finished_parts": response.finished_part_count,
        "held_part_id": response.held_part_id,
        "active_order_id": response.active_order_id,
        "battery_percentage": response.battery.percentage,
        "machines": [
            {
                "machine_id": item.machine_id,
                "state": int(item.state),
                "door_open": item.door_open,
                "part_present": item.part_present,
                "part_id": item.part_id,
                "fault_code": int(item.fault_code),
            }
            for item in response.machines
        ],
    }


def main() -> None:
    args = parse_args()
    if args.minimum_finished < 0 or args.timeout <= 0.0:
        raise SystemExit("invalid completion threshold or timeout")

    rclpy.init()
    node = Node("wait_for_factory_completion")
    client = node.create_client(GetFactoryState, "/factory/get_state")
    deadline = time.monotonic() + args.timeout
    last_payload = None
    try:
        if not client.wait_for_service(timeout_sec=30.0):
            raise SystemExit("/factory/get_state is unavailable")
        while time.monotonic() < deadline:
            future = client.call_async(GetFactoryState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            if future.done() and future.exception() is None:
                response = future.result()
                last_payload = state_payload(response)
                if (
                    response.finished_part_count >= args.minimum_finished
                    and not response.active_order_id
                    and not response.held_part_id
                ):
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    args.output.write_text(
                        json.dumps(last_payload, indent=2) + "\n",
                        encoding="utf-8",
                    )
                    print(json.dumps(last_payload, ensure_ascii=False))
                    return
            time.sleep(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if last_payload is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(last_payload, indent=2) + "\n", encoding="utf-8"
        )
    raise SystemExit("factory completion timed out")


if __name__ == "__main__":
    main()
