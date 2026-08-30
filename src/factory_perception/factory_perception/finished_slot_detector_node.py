"""Publish typed RGB-D occupancy for the taught finished-bin slots."""

from __future__ import annotations

import math
import time

from cv_bridge import CvBridge
from factory_interfaces.msg import TrayOccupancy, TraySlotState
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .slots import (
    PinholeIntrinsics,
    SlotObservation,
    SlotTarget,
    detect_slots_in_point_cloud,
)
from .sparse_bin import quaternion_rotation_matrix


def summarize_slots(message: TrayOccupancy) -> str:
    """Build a concise operator-facing summary without hiding unknown slots."""

    states = []
    for slot in message.slots:
        state = "unknown"
        if slot.observable:
            state = "occupied" if slot.occupied else "empty"
        states.append(f"{slot.slot_id}:{state}")
    return ", ".join(states)


def configured_slot_targets(
    slot_ids: tuple[int, ...],
    centers_xy: tuple[float, ...],
    *,
    sample_z: float,
) -> tuple[SlotTarget, ...]:
    """Parse flat ROS parameters into explicit, validated slot targets."""

    if not slot_ids or any(slot_id < 1 for slot_id in slot_ids):
        raise ValueError("slot_ids must contain positive ids")
    if len(set(slot_ids)) != len(slot_ids):
        raise ValueError("slot_ids cannot contain duplicates")
    if len(centers_xy) != 2 * len(slot_ids):
        raise ValueError("slot_centers_xy must contain x/y for every slot")
    if not math.isfinite(sample_z):
        raise ValueError("sample_z must be finite")
    return tuple(
        SlotTarget(
            slot_id=slot_id,
            x=float(centers_xy[2 * index]),
            y=float(centers_xy[2 * index + 1]),
            sample_z=sample_z,
        )
        for index, slot_id in enumerate(slot_ids)
    )


class FinishedSlotDetectorNode(Node):
    """Project taught slots into the current depth image and classify each."""

    def __init__(self) -> None:
        super().__init__("finished_slot_detector")
        self.declare_parameter("tray_id", "finished_bin")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("slot_ids", [1, 2, 3, 4])
        self.declare_parameter(
            "slot_centers_xy",
            [
                0.72, -0.20, 0.72, 0.05, 0.72, 0.30, 0.72, -0.32,
            ],
        )
        # DART settles the docked base_link at world z=0.38 m; the
        # finished table top is world z=0.554 m. Point-cloud crops therefore
        # start at z=0.174 m and accept the 120 mm workpiece volume above it.
        self.declare_parameter("surface_z", 0.174)
        self.declare_parameter("sample_z", 0.294)
        self.declare_parameter("slot_half_size", 0.065)
        self.declare_parameter("minimum_height", 0.025)
        self.declare_parameter("maximum_height", 0.18)
        self.declare_parameter("minimum_region_points", 30)
        self.declare_parameter("minimum_object_points", 12)

        self._targets = configured_slot_targets(
            tuple(int(value) for value in self.get_parameter("slot_ids").value),
            tuple(
                float(value)
                for value in self.get_parameter("slot_centers_xy").value
            ),
            sample_z=float(self.get_parameter("sample_z").value),
        )
        self._validate_parameters()
        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            TrayOccupancy, "/perception/finished_bin_slots", 10
        )
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._remember_camera_info, 10
        )
        self.create_subscription(
            Image, "/camera/depth/image_raw", self._detect, 10
        )
        self._next_log_at = 0.0

    def _validate_parameters(self) -> None:
        if float(self.get_parameter("slot_half_size").value) <= 0.0:
            raise ValueError("slot_half_size must be positive")
        minimum_height = float(self.get_parameter("minimum_height").value)
        maximum_height = float(self.get_parameter("maximum_height").value)
        if not 0.0 < minimum_height < maximum_height:
            raise ValueError("height limits must satisfy 0 < minimum < maximum")
        for name in ("minimum_region_points", "minimum_object_points"):
            if int(self.get_parameter(name).value) < 1:
                raise ValueError(f"{name} must be positive")

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
                source_frame, output_frame, Time()
            )
        except TransformException as error:
            self._log_throttled(f"Waiting for finished-bin camera TF: {error}")
            return

        depth = np.asarray(
            self._bridge.imgmsg_to_cv2(
                message, desired_encoding="passthrough"
            ),
            dtype=np.float32,
        )
        if message.encoding == "16UC1":
            depth *= 0.001

        rotation_message = transform.transform.rotation
        rotation = quaternion_rotation_matrix(
            rotation_message.x,
            rotation_message.y,
            rotation_message.z,
            rotation_message.w,
        )
        translation_message = transform.transform.translation
        translation = (
            translation_message.x,
            translation_message.y,
            translation_message.z,
        )
        intrinsics = PinholeIntrinsics(
            fx=float(camera_info.k[0]),
            fy=float(camera_info.k[4]),
            cx=float(camera_info.k[2]),
            cy=float(camera_info.k[5]),
        )
        observations = {
            observation.slot_id: observation
            for observation in detect_slots_in_point_cloud(
                depth,
                intrinsics,
                rotation,
                translation,
                self._targets,
                surface_z=float(self.get_parameter("surface_z").value),
                slot_half_size=float(
                    self.get_parameter("slot_half_size").value
                ),
                minimum_height=float(
                    self.get_parameter("minimum_height").value
                ),
                maximum_height=float(
                    self.get_parameter("maximum_height").value
                ),
                minimum_region_points=int(
                    self.get_parameter("minimum_region_points").value
                ),
                minimum_object_points=int(
                    self.get_parameter("minimum_object_points").value
                ),
            )
        }
        self._publish(message, observations)

    def _publish(
        self,
        image: Image,
        observations: dict[int, SlotObservation],
    ) -> None:
        result = TrayOccupancy()
        result.header = image.header
        result.header.frame_id = str(self.get_parameter("output_frame").value)
        result.tray_id = str(self.get_parameter("tray_id").value)
        for target in self._targets:
            observation = observations.get(target.slot_id)
            state = TraySlotState()
            state.slot_id = target.slot_id
            state.observable = bool(
                observation is not None and observation.observable
            )
            state.occupied = bool(
                state.observable
                and observation is not None
                and observation.occupied
            )
            state.median_depth = (
                float(observation.median_depth)
                if observation is not None
                else float("nan")
            )
            state.height_above_surface = (
                float(observation.height_above_tray)
                if observation is not None
                else 0.0
            )
            state.valid_fraction = (
                float(observation.valid_fraction)
                if observation is not None
                else 0.0
            )
            result.slots.append(state)
        self._publisher.publish(result)
        summary = summarize_slots(result)
        evidence = ", ".join(
            (
                f"{slot.slot_id}:h={slot.height_above_surface:.3f}m"
                f"/valid={slot.valid_fraction:.2f}"
            )
            for slot in result.slots
        )
        self._log_throttled(
            f"Finished-bin slots: {summary}; evidence: {evidence}"
        )

    def _log_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now < self._next_log_at:
            return
        self._next_log_at = now + 5.0
        self.get_logger().info(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FinishedSlotDetectorNode()
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
