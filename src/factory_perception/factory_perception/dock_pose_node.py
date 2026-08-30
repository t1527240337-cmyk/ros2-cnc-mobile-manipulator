"""Convert visible AprilTag TF frames into the docking pose topic."""

from __future__ import annotations

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformException, TransformListener


def select_target_detection(
    detections,
    known_tag_ids: set[int],
    target_tag_id: int | None,
    minimum_margin: float,
):
    """Return a valid observation of the requested tag, or None."""
    if target_tag_id is None:
        return None

    usable = [
        detection
        for detection in detections
        if detection.id == target_tag_id
        and detection.id in known_tag_ids
        and detection.hamming == 0
        and detection.decision_margin >= minimum_margin
    ]
    return max(usable, key=lambda detection: detection.decision_margin, default=None)


class DockPoseNode(Node):
    """Publish a docking pose only for the explicitly selected station tag."""

    def __init__(self) -> None:
        super().__init__("dock_pose_from_tag")
        self.declare_parameter("tag_ids", [1, 2, 3, 10, 11, 20])
        self.declare_parameter(
            "tag_frames",
            [
                "machine_1_tag",
                "machine_2_tag",
                "machine_3_tag",
                "raw_bin_tag",
                "finished_bin_tag",
                "charge_dock_tag",
            ],
        )
        self.declare_parameter("minimum_decision_margin", 25.0)
        self.declare_parameter("maximum_detection_age", 0.5)

        tag_ids = [int(value) for value in self.get_parameter("tag_ids").value]
        tag_frames = [str(value) for value in self.get_parameter("tag_frames").value]
        if len(tag_ids) != len(tag_frames) or len(set(tag_ids)) != len(tag_ids):
            raise ValueError("tag_ids and tag_frames must be unique, equal-length lists")
        self._frames_by_id = dict(zip(tag_ids, tag_frames, strict=True))
        self._target_tag_id: int | None = None

        self._minimum_margin = float(
            self.get_parameter("minimum_decision_margin").value
        )
        self._maximum_age = Duration(
            seconds=float(self.get_parameter("maximum_detection_age").value)
        )
        self._camera_frame: str | None = None
        self._tag_frame: str | None = None
        self._detection_time = self.get_clock().now()

        self._tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(PoseStamped, "/detected_dock_pose", 10)
        self.create_subscription(
            AprilTagDetectionArray, "/detections", self._on_detections, 10
        )
        target_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Int32,
            "/perception/target_tag_id",
            self._on_target_tag,
            target_qos,
        )
        self.create_timer(0.05, self._publish_pose)

    def _on_target_tag(self, message: Int32) -> None:
        requested_tag_id = int(message.data)
        if requested_tag_id == -1:
            target_tag_id = None
        elif requested_tag_id in self._frames_by_id:
            target_tag_id = requested_tag_id
        else:
            self.get_logger().warning(
                f"Ignoring unknown target AprilTag {requested_tag_id}"
            )
            return

        if target_tag_id == self._target_tag_id:
            return
        self._target_tag_id = target_tag_id
        self._camera_frame = None
        self._tag_frame = None
        description = "disabled" if target_tag_id is None else target_tag_id
        self.get_logger().info(f"Perception target changed to {description}")

    def _on_detections(self, message: AprilTagDetectionArray) -> None:
        detection = select_target_detection(
            detections=message.detections,
            known_tag_ids=set(self._frames_by_id),
            target_tag_id=self._target_tag_id,
            minimum_margin=self._minimum_margin,
        )
        if detection is None:
            return

        self._camera_frame = message.header.frame_id
        self._tag_frame = self._frames_by_id[detection.id]
        self._detection_time = self.get_clock().now()

    def _publish_pose(self) -> None:
        if self._camera_frame is None or self._tag_frame is None:
            return
        if self.get_clock().now() - self._detection_time > self._maximum_age:
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._camera_frame, self._tag_frame, Time()
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Waiting for {self._camera_frame} <- {self._tag_frame}: {error}",
                throttle_duration_sec=2.0,
            )
            return

        pose = PoseStamped()
        pose.header.frame_id = transform.header.frame_id
        pose.header.stamp = self._detection_time.to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self._publisher.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
