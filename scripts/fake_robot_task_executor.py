#!/usr/bin/env python3
"""Fast acceptance double for the persistent queue worker."""

import argparse
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from factory_interfaces.action import ExecuteRobotTask


class FakeRobotTaskExecutor(Node):
    def __init__(
        self,
        *,
        fail_first: bool = False,
        hang_first: bool = False,
    ) -> None:
        super().__init__("fake_robot_task_executor")
        self._fail_first = fail_first
        self._hang_first = hang_first
        self._goals_received = 0
        self._server = ActionServer(
            self,
            ExecuteRobotTask,
            "/factory/execute_robot_task",
            execute_callback=self._execute,
        )

    def _execute(self, goal_handle):
        self._goals_received += 1
        feedback = ExecuteRobotTask.Feedback()
        feedback.phase = "accepted"
        feedback.machine_id = goal_handle.request.machine_id
        feedback.detail = goal_handle.request.task_id
        goal_handle.publish_feedback(feedback)

        if self._hang_first and self._goals_received == 1:
            feedback.phase = "physical_transfer"
            feedback.detail = "bilateral contact verified before interruption"
            goal_handle.publish_feedback(feedback)
            self.get_logger().warning(
                "Holding first goal after a persisted physical checkpoint"
            )
            while rclpy.ok():
                time.sleep(0.10)
            # Process shutdown normally prevents this result from reaching
            # the original client; keep the callback type complete.
            return ExecuteRobotTask.Result()

        result = ExecuteRobotTask.Result()
        if self._fail_first and self._goals_received == 1:
            result.success = False
            result.retryable = False
            result.error_code = 900
            result.message = "injected unsafe physical failure"
            goal_handle.abort()
            return result

        result.success = True
        result.retryable = False
        result.error_code = 0
        result.message = "acceptance task completed"
        goal_handle.succeed()
        return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--fail-first",
        action="store_true",
        help="abort the first task with a non-retryable physical result",
    )
    parser.add_argument(
        "--hang-first",
        action="store_true",
        help="hold the first task after publishing a physical checkpoint",
    )
    arguments = parser.parse_args()

    rclpy.init()
    node = FakeRobotTaskExecutor(
        fail_first=arguments.fail_first,
        hang_first=arguments.hang_first,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
