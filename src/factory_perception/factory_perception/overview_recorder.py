"""Record the fixed Gazebo overview camera without a graphical client."""

from __future__ import annotations

import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class OverviewRecorder(Node):
    """Encode ROS images into an atomically finalized MP4 evidence file."""

    def __init__(self) -> None:
        super().__init__("factory_overview_recorder")
        self.declare_parameter("topic", "/demo/overview/image_raw")
        self.declare_parameter("output", "artifacts/demo/factory_overview.mp4")
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("maximum_seconds", 0.0)

        self._topic = str(self.get_parameter("topic").value)
        self._output = Path(str(self.get_parameter("output").value)).expanduser()
        self._fps = float(self.get_parameter("fps").value)
        self._maximum_seconds = float(
            self.get_parameter("maximum_seconds").value
        )
        if self._fps <= 0.0:
            raise ValueError("fps must be positive")
        if self._maximum_seconds < 0.0:
            raise ValueError("maximum_seconds cannot be negative")

        self._output.parent.mkdir(parents=True, exist_ok=True)
        self._partial = self._output.with_suffix(".partial.mp4")
        self._bridge = CvBridge()
        self._writer = None
        self._frame_count = 0
        self._first_frame_time = None
        self.finished = False
        self.create_subscription(
            Image, self._topic, self._on_image, 10
        )
        self.get_logger().info(
            f"Waiting for headless overview frames on {self._topic}"
        )

    def _on_image(self, message: Image) -> None:
        if self.finished:
            return
        frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if self._writer is None:
            height, width = frame.shape[:2]
            self._writer = cv2.VideoWriter(
                str(self._partial),
                cv2.VideoWriter_fourcc(*"mp4v"),
                self._fps,
                (width, height),
            )
            if not self._writer.isOpened():
                raise RuntimeError(f"cannot open video output {self._partial}")
            self._first_frame_time = time.monotonic()
            self.get_logger().info(
                f"Recording {width}x{height} at {self._fps:.1f} fps"
            )

        self._writer.write(frame)
        self._frame_count += 1
        if (
            self._maximum_seconds > 0.0
            and self._frame_count / self._fps >= self._maximum_seconds
        ):
            self.finish()

    def finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            self._partial.replace(self._output)
        elapsed = (
            0.0
            if self._first_frame_time is None
            else time.monotonic() - self._first_frame_time
        )
        metadata = {
            "topic": self._topic,
            "output": str(self._output),
            "frame_count": self._frame_count,
            "encoded_fps": self._fps,
            "encoded_duration_seconds": self._frame_count / self._fps,
            "wall_recording_seconds": elapsed,
        }
        self._output.with_suffix(".json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        self.get_logger().info(
            f"Finalized {self._frame_count} frames at {self._output}"
        )

    def destroy_node(self):
        self.finish()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = OverviewRecorder()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
