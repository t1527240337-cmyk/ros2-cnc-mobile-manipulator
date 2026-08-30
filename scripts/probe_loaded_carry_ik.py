#!/usr/bin/env python3
"""Probe collision-aware IK candidates for the loaded arm carry pose.

This is a small engineering diagnostic, not part of the production task
executor.  It asks MoveIt's ``/compute_ik`` service whether an upright,
side-grasped workpiece can be carried at each requested base-frame position.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass(frozen=True)
class Candidate:
    x: float
    y: float
    z: float


class CarryIkProbe(Node):
    """Call MoveIt's IK service using the robot's current joint state."""

    def __init__(self) -> None:
        super().__init__("factory_loaded_carry_ik_probe")
        self._joint_state: JointState | None = None
        self._joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self._remember_joint_state, 10
        )
        self._client = self.create_client(GetPositionIK, "/compute_ik")

    def _remember_joint_state(self, message: JointState) -> None:
        self._joint_state = message

    def wait_until_ready(self, timeout: float) -> None:
        if not self._client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/compute_ik is not available")

        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self._joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds >= deadline:
                raise RuntimeError("/joint_states was not received")

    def solve(self, candidate: Candidate, timeout: float) -> int:
        assert self._joint_state is not None
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "arm"
        ik.robot_state = RobotState(joint_state=self._joint_state, is_diff=True)
        ik.pose_stamped.header.frame_id = "base_link"
        ik.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik.pose_stamped.pose.position.x = candidate.x
        ik.pose_stamped.pose.position.y = candidate.y
        ik.pose_stamped.pose.position.z = candidate.z

        # Fixed upright side-grasp orientation used by ManipulatePart.
        ik.pose_stamped.pose.orientation.x = 0.5
        ik.pose_stamped.pose.orientation.y = 0.5
        ik.pose_stamped.pose.orientation.z = 0.5
        ik.pose_stamped.pose.orientation.w = 0.5
        ik.avoid_collisions = True
        ik.timeout.sec = int(timeout)
        ik.timeout.nanosec = int((timeout - int(timeout)) * 1e9)

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout + 1.0)
        if not future.done():
            raise RuntimeError(
                f"IK request timed out for x={candidate.x:.2f}, z={candidate.z:.2f}"
            )
        response = future.result()
        if response is None:
            raise RuntimeError("IK service returned no response")
        return int(response.error_code.val)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--x",
        nargs="+",
        type=float,
        default=[0.45, 0.48, 0.50, 0.52, 0.55, 0.58, 0.60, 0.62, 0.65],
        help="base_link x candidates in metres",
    )
    parser.add_argument(
        "--z",
        nargs="+",
        type=float,
        default=[0.70, 0.80],
        help="base_link z candidates in metres",
    )
    parser.add_argument(
        "--y",
        nargs="+",
        type=float,
        default=[0.0],
        help="base_link lateral y candidates in metres",
    )
    parser.add_argument("--timeout", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = _arguments()
    rclpy.init()
    node = CarryIkProbe()
    try:
        node.wait_until_ready(args.timeout + 3.0)
        print("collision-aware loaded-carry IK (SUCCESS=1)")
        for lateral in args.y:
            for height in args.z:
                row = []
                for x_position in args.x:
                    candidate = Candidate(
                        x=x_position, y=lateral, z=height
                    )
                    code = node.solve(candidate, args.timeout)
                    state = (
                        "OK"
                        if code == MoveItErrorCodes.SUCCESS
                        else f"NO({code})"
                    )
                    row.append(f"x={x_position:.2f}:{state}")
                print(
                    f"y={lateral:.2f} z={height:.2f}  " + "  ".join(row)
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
