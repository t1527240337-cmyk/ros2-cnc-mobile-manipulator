from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import Image

from .slots import Slot, detect_slots


class SlotDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("slot_detector")
        self.declare_parameter("tray_frame", "raw_bin_tag")
        self.declare_parameter("tray_plane_depth", 1.0)
        self.declare_parameter("minimum_height", 0.02)
        self._bridge = CvBridge()
        self._slots = [
            Slot(0, 240, 210, 18, -0.20, -0.12), Slot(1, 320, 210, 18, 0.0, -0.12),
            Slot(2, 400, 210, 18, 0.20, -0.12), Slot(3, 240, 290, 18, -0.20, 0.12),
            Slot(4, 320, 290, 18, 0.0, 0.12), Slot(5, 400, 290, 18, 0.20, 0.12),
        ]
        self._publisher = self.create_publisher(PoseArray, "/perception/occupied_slots", 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self._depth, 10)

    def _depth(self, message: Image) -> None:
        image = self._bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        depth = np.asarray(image, dtype=np.float32)
        if message.encoding == "16UC1":
            depth *= 0.001
        observations = detect_slots(
            depth,
            self._slots,
            float(self.get_parameter("tray_plane_depth").value),
            float(self.get_parameter("minimum_height").value),
        )
        result = PoseArray()
        result.header.stamp = message.header.stamp
        result.header.frame_id = str(self.get_parameter("tray_frame").value)
        for observation in observations:
            if not observation.occupied:
                continue
            pose = Pose()
            pose.position.x = observation.tray_x
            pose.position.y = observation.tray_y
            pose.position.z = observation.height_above_tray / 2.0
            pose.orientation.w = 1.0
            result.poses.append(pose)
        self._publisher.publish(result)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlotDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
