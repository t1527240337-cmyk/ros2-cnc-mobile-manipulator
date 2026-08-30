"""Publish RGB-D workpiece candidates from a configured 3-D work volume."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .sparse_bin import (
    CameraIntrinsics,
    Region3D,
    detect_sparse_parts,
    quaternion_rotation_matrix,
)


class SparseBinDetectorNode(Node):
    """Detect separated upright cylinders in a base-frame work volume."""

    def __init__(self) -> None:
        super().__init__("sparse_bin_detector")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter(
            "output_topic", "/perception/raw_part_candidates"
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/camera_info"
        )
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("region_min", [0.58, -0.42, 0.19])
        self.declare_parameter("region_max", [0.96, 0.42, 0.36])
        self.declare_parameter("minimum_component_pixels", 20)
        self.declare_parameter("maximum_component_span", 0.14)
        self.declare_parameter("maximum_candidates", 6)
        # Zero keeps generic silhouette-midpoint detection. A positive value
        # enables geometric axis fitting for known upright cylinders.
        self.declare_parameter("upright_cylinder_radius", 0.0)
        self.declare_parameter("supported_center_height", 0.0)
        self.declare_parameter("cylinder_radius_tolerance", 0.01)

        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        output_topic = str(self.get_parameter("output_topic").value)
        if not output_topic.startswith("/"):
            raise ValueError("output_topic must be an absolute ROS topic")
        camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        depth_topic = str(self.get_parameter("depth_topic").value)
        if not camera_info_topic.startswith("/"):
            raise ValueError(
                "camera_info_topic must be an absolute ROS topic"
            )
        if not depth_topic.startswith("/"):
            raise ValueError("depth_topic must be an absolute ROS topic")

        self._publisher = self.create_publisher(PoseArray, output_topic, 10)
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._remember_camera_info,
            10,
        )
        self.create_subscription(
            Image, depth_topic, self._detect, 10
        )
        self._next_log_at = 0.0

    def _remember_camera_info(self, message: CameraInfo) -> None:
        self._camera_info = message

    def _detect(self, message: Image) -> None:
        camera_info = self._camera_info
        if camera_info is None:
            return

        source_frame = message.header.frame_id or camera_info.header.frame_id
        output_frame = str(self.get_parameter("output_frame").value)
        try:
            transform = self._tf_buffer.lookup_transform(
                output_frame, source_frame, Time()
            )
        except TransformException as error:
            self._log_throttled(f"Waiting for camera TF: {error}")
            return

        depth = np.asarray(
            self._bridge.imgmsg_to_cv2(
                message, desired_encoding="passthrough"
            ),
            dtype=np.float32,
        )
        if message.encoding == "16UC1":
            depth *= 0.001

        rotation = transform.transform.rotation
        translation = transform.transform.translation
        try:
            candidates = detect_sparse_parts(
                depth,
                CameraIntrinsics(
                    fx=float(camera_info.k[0]),
                    fy=float(camera_info.k[4]),
                    cx=float(camera_info.k[2]),
                    cy=float(camera_info.k[5]),
                ),
                quaternion_rotation_matrix(
                    rotation.x, rotation.y, rotation.z, rotation.w
                ),
                (translation.x, translation.y, translation.z),
                Region3D(
                    minimum=tuple(
                        float(value)
                        for value in self.get_parameter("region_min").value
                    ),
                    maximum=tuple(
                        float(value)
                        for value in self.get_parameter("region_max").value
                    ),
                ),
                minimum_component_pixels=int(
                    self.get_parameter("minimum_component_pixels").value
                ),
                maximum_component_span=float(
                    self.get_parameter("maximum_component_span").value
                ),
                maximum_candidates=int(
                    self.get_parameter("maximum_candidates").value
                ),
                upright_cylinder_radius=(
                    float(
                        self.get_parameter("upright_cylinder_radius").value
                    )
                    or None
                ),
                supported_center_height=(
                    float(
                        self.get_parameter("supported_center_height").value
                    )
                    or None
                ),
                cylinder_radius_tolerance=float(
                    self.get_parameter("cylinder_radius_tolerance").value
                ),
            )
        except ValueError as error:
            self.get_logger().error(
                f"Invalid sparse-bin configuration: {error}"
            )
            return

        result = PoseArray()
        result.header = message.header
        result.header.frame_id = output_frame
        for candidate in candidates:
            pose = Pose()
            pose.position.x = candidate.x
            pose.position.y = candidate.y
            pose.position.z = candidate.z
            pose.orientation.w = 1.0
            result.poses.append(pose)
        self._publisher.publish(result)
        self._log_throttled(
            f"Work-volume perception sees {len(candidates)} candidate(s)"
        )

    def _log_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now < self._next_log_at:
            return
        self._next_log_at = now + 5.0
        self.get_logger().info(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SparseBinDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
