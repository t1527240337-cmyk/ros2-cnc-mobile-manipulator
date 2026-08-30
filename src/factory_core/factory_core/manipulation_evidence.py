"""Sensor evidence used by the manipulation controller.

This module deliberately has no subscription to Gazebo pose publishers.  The
runtime controller may use joint-state-derived TF, RGB-D detections and
simulated contact sensors, exactly as a hardware controller would use robot
kinematics, cameras and tactile switches.  Gazebo entity pose is reserved for
black-box acceptance tests outside the controller process.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time

from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class Point3:
    """Cartesian point in metres."""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Quaternion:
    """Quaternion in ROS x/y/z/w order."""

    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True)
class FingerContactCheck:
    """Fresh tactile evidence from both gripper fingers."""

    left: bool
    right: bool

    @property
    def accepted(self) -> bool:
        return self.left and self.right

    def describe(self) -> str:
        return f"left={self.left}, right={self.right}"


@dataclass(frozen=True)
class FingerContactSelection:
    """The one physical workpiece contacted by both fingertips."""

    physical_part_id: str | None
    contact: FingerContactCheck
    ambiguous: bool = False

    @property
    def accepted(self) -> bool:
        return (
            self.physical_part_id is not None
            and self.contact.accepted
            and not self.ambiguous
        )

    def describe(self) -> str:
        identity = self.physical_part_id or "none"
        suffix = ", ambiguous=true" if self.ambiguous else ""
        return f"physical_part={identity}, {self.contact.describe()}{suffix}"


def contact_is_current(
    sample_time: float | None,
    *,
    now: float,
    maximum_age: float,
    received_after: float,
) -> bool:
    """Return whether a contact belongs to the current operation window."""

    return (
        sample_time is not None
        and sample_time >= received_after
        and now - sample_time <= maximum_age
    )


def collision_belongs_to_part(collision_name: str, part_id: str) -> bool:
    """Match a Gazebo collision name to one requested workpiece."""

    return (
        collision_name == part_id
        or collision_name.startswith(f"{part_id}::")
        or f"::{part_id}::" in collision_name
    )


def support_collision_matches_station(
    collision_name: str, station_id: str
) -> bool:
    """Match only load-bearing geometry belonging to the requested station."""

    if station_id.startswith("machine_"):
        # Only the zero-point pallet proves that stock is seated in the CNC.
        # Contact with the work table, enclosure or spindle is a failed
        # insertion and must never be promoted to a successful placement.
        return (
            station_id in collision_name
            and "fixture_base_collision" in collision_name
        )
    if station_id == "finished_bin":
        return "finished_bin" in collision_name
    if station_id == "raw_bin":
        return "raw_bin" in collision_name
    return False


def grasp_hold_is_valid(
    contact: FingerContactCheck,
    measured_position: float | None,
    reference_position: float | None,
    *,
    minimum_total_closure: float,
    maximum_position_change: float,
) -> bool:
    """Validate bilateral contact and total jaw closure without entity pose."""

    if minimum_total_closure < 0.0:
        raise ValueError("minimum_total_closure cannot be negative")
    if maximum_position_change <= 0.0:
        raise ValueError("maximum_position_change must be positive")
    return (
        contact.accepted
        and measured_position is not None
        and reference_position is not None
        and measured_position >= minimum_total_closure
        and abs(measured_position - reference_position)
        <= maximum_position_change
    )


def proof_lift_reseat_is_valid(
    contact: FingerContactCheck,
    measured_position: float | None,
    *,
    minimum_total_closure: float,
    maximum_total_closure: float,
) -> bool:
    """Validate a new stable aperture while the first load is applied.

    V-shaped compliant pads can centre an initially off-axis cylinder during
    the proof lift. Fresh bilateral identity proves continued ownership; the
    bounded aperture rejects both an open/drop state and the closed stops.
    """
    if minimum_total_closure < 0.0:
        raise ValueError("minimum_total_closure cannot be negative")
    if maximum_total_closure <= minimum_total_closure:
        raise ValueError("maximum_total_closure must exceed the minimum")
    return (
        contact.accepted
        and measured_position is not None
        and minimum_total_closure <= measured_position
        <= maximum_total_closure
    )


def loaded_hold_reseat_is_valid(
    contact: FingerContactCheck,
    measured_position: float | None,
    reference_position: float | None,
    *,
    minimum_total_closure: float,
    maximum_total_closure: float,
    maximum_reseat_change: float,
) -> bool:
    """Validate one bounded compliant reseat during loaded transport.

    A cylinder can move deeper into the V pads when its load direction changes.
    This is not ordinary aperture drift: callers may use this transition only
    once, after bilateral identity and low-velocity samples remain stable.
    """
    if maximum_reseat_change <= 0.0:
        raise ValueError("maximum_reseat_change must be positive")
    if reference_position is None:
        return False
    return (
        proof_lift_reseat_is_valid(
            contact,
            measured_position,
            minimum_total_closure=minimum_total_closure,
            maximum_total_closure=maximum_total_closure,
        )
        and measured_position is not None
        and abs(measured_position - reference_position)
        <= maximum_reseat_change
    )


class ManipulationEvidence:
    """Collect fail-closed kinematic, tactile and support evidence."""

    def __init__(
        self,
        node: Node,
        part_ids: tuple[str, ...],
        *,
        maximum_sample_age: float = 1.0,
    ) -> None:
        if maximum_sample_age <= 0.0:
            raise ValueError("maximum_sample_age must be positive")
        self._part_ids = part_ids
        self._maximum_sample_age = maximum_sample_age
        self._lock = threading.Lock()
        self._finger_contact_times: dict[str, dict[str, float]] = {
            "left": {},
            "right": {},
        }
        self._support_contact_times: dict[tuple[str, str], float] = {}
        self._contact_pairs: dict[str, dict[tuple[str, str], float]] = {
            part_id: {} for part_id in part_ids
        }
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._workpiece_contacts = node.create_subscription(
            Contacts,
            "/factory/workpieces/contacts",
            self._remember_contacts,
            10,
        )
        self._cnc_contacts = node.create_subscription(
            Contacts,
            "/factory/cnc/contacts",
            self._remember_contacts,
            10,
        )

    def point_in_base(
        self, source_frame: str, point: Point3, *, timeout_sec: float
    ) -> Point3:
        """Transform a calibrated station landmark into ``base_link``."""

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    "base_link", source_frame, Time()
                )
            except TransformException:
                time.sleep(0.02)
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            rotated = _rotate(point, Quaternion(
                rotation.x, rotation.y, rotation.z, rotation.w
            ))
            return Point3(
                translation.x + rotated.x,
                translation.y + rotated.y,
                translation.z + rotated.z,
            )
        raise RuntimeError(f"no current base_link to {source_frame} transform")

    def current_tcp_in_base(
        self, *, timeout_sec: float
    ) -> tuple[Point3, Quaternion]:
        """Read the joint-state-derived tool pose, never simulator truth."""

        return self._frame_pose_in_base("gripper_tcp", timeout_sec=timeout_sec)

    def kinematic_pad_positions_in_base(
        self, *, timeout_sec: float
    ) -> tuple[Point3, Point3]:
        """Read both joint-state-derived fingertip positions."""

        left, _ = self._frame_pose_in_base(
            "gripper_left_finger_tip_link", timeout_sec=timeout_sec
        )
        right, _ = self._frame_pose_in_base(
            "gripper_right_finger_tip_link", timeout_sec=timeout_sec
        )
        return left, right

    def begin_finger_contact_window(self, part_id: str) -> float:
        """Clear stale fingertip contacts and return the attempt start time."""

        self._require_part(part_id)
        with self._lock:
            started_at = time.monotonic()
            self._finger_contact_times["left"].pop(part_id, None)
            self._finger_contact_times["right"].pop(part_id, None)
        return started_at

    def begin_any_finger_contact_window(self) -> float:
        """Clear all stale contacts before an anonymous raw-bin grasp."""

        with self._lock:
            started_at = time.monotonic()
            for contacts in self._finger_contact_times.values():
                contacts.clear()
        return started_at

    def check_any_two_finger_contact(
        self, *, timeout_sec: float, received_after: float
    ) -> FingerContactSelection:
        """Wait until exactly one configured workpiece touches both pads."""

        deadline = time.monotonic() + timeout_sec
        latest = FingerContactSelection(None, FingerContactCheck(False, False))
        while time.monotonic() < deadline:
            latest = self.recent_any_two_finger_contact(
                received_after=received_after
            )
            if latest.accepted or latest.ambiguous:
                return latest
            time.sleep(0.02)
        return latest

    def recent_any_two_finger_contact(
        self, *, received_after: float = 0.0
    ) -> FingerContactSelection:
        """Identify a raw workpiece from tactile evidence, never Gazebo pose."""

        matches = tuple(
            (part_id, self.recent_two_finger_contact(
                part_id, received_after=received_after
            ))
            for part_id in self._part_ids
        )
        accepted = tuple(item for item in matches if item[1].accepted)
        if len(accepted) == 1:
            return FingerContactSelection(accepted[0][0], accepted[0][1])
        if len(accepted) > 1:
            return FingerContactSelection(
                None, FingerContactCheck(True, True), ambiguous=True
            )
        left = any(check.left for _, check in matches)
        right = any(check.right for _, check in matches)
        return FingerContactSelection(None, FingerContactCheck(left, right))

    def check_two_finger_contact(
        self,
        part_id: str,
        *,
        timeout_sec: float,
        received_after: float,
    ) -> FingerContactCheck:
        """Wait for fresh bilateral workpiece contact."""

        self._require_part(part_id)
        deadline = time.monotonic() + timeout_sec
        latest = FingerContactCheck(False, False)
        while time.monotonic() < deadline:
            latest = self.recent_two_finger_contact(
                part_id, received_after=received_after
            )
            if latest.accepted:
                return latest
            time.sleep(0.02)
        return latest

    def recent_two_finger_contact(
        self, part_id: str, *, received_after: float = 0.0
    ) -> FingerContactCheck:
        """Return current tactile state without blocking."""

        self._require_part(part_id)
        now = time.monotonic()
        with self._lock:
            left = self._finger_contact_times["left"].get(part_id)
            right = self._finger_contact_times["right"].get(part_id)
        return FingerContactCheck(
            left=contact_is_current(
                left,
                now=now,
                maximum_age=self._maximum_sample_age,
                received_after=received_after,
            ),
            right=contact_is_current(
                right,
                now=now,
                maximum_age=self._maximum_sample_age,
                received_after=received_after,
            ),
        )

    def begin_support_contact_window(
        self, part_id: str, station_id: str
    ) -> float:
        """Clear stale target-support evidence before final insertion."""

        self._require_part(part_id)
        with self._lock:
            self._support_contact_times.pop((part_id, station_id), None)
        return time.monotonic()

    def wait_for_support_contact(
        self,
        part_id: str,
        station_id: str,
        *,
        received_after: float,
        timeout_sec: float,
    ) -> bool:
        """Require fresh contact with the selected fixture or destination."""

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.recent_support_contact(
                part_id, station_id, received_after=received_after
            ):
                return True
            time.sleep(0.02)
        return False

    def recent_support_contact(
        self,
        part_id: str,
        station_id: str,
        *,
        received_after: float,
    ) -> bool:
        """Return fresh target-support evidence without blocking motion."""

        self._require_part(part_id)
        now = time.monotonic()
        with self._lock:
            received_at = self._support_contact_times.get(
                (part_id, station_id)
            )
        return contact_is_current(
            received_at,
            now=now,
            maximum_age=self._maximum_sample_age,
            received_after=received_after,
        )

    def recent_contact_summary(
        self, part_id: str, *, maximum_age: float = 2.0
    ) -> str:
        """Describe recent collision pairs for failure diagnostics."""

        self._require_part(part_id)
        now = time.monotonic()
        with self._lock:
            recent = tuple(
                pair
                for pair, received_at in self._contact_pairs[part_id].items()
                if now - received_at <= maximum_age
            )
        if not recent:
            return "contacts=none"
        return "contacts=" + "; ".join(
            sorted(" <-> ".join(pair) for pair in recent)
        )

    def _frame_pose_in_base(
        self, child_frame: str, *, timeout_sec: float
    ) -> tuple[Point3, Quaternion]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    "base_link", child_frame, Time()
                )
            except TransformException:
                time.sleep(0.02)
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return (
                Point3(translation.x, translation.y, translation.z),
                Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
            )
        raise RuntimeError(f"no current base_link to {child_frame} transform")

    def _remember_contacts(self, message: Contacts) -> None:
        received_at = time.monotonic()
        with self._lock:
            for contact in message.contacts:
                pair = (contact.collision1.name, contact.collision2.name)
                for part_id in self._part_ids:
                    if not any(
                        collision_belongs_to_part(name, part_id)
                        for name in pair
                    ):
                        continue
                    self._contact_pairs[part_id][pair] = received_at
                    for name in pair:
                        if "gripper_left_finger_tip_link" in name:
                            self._finger_contact_times["left"][part_id] = received_at
                        if "gripper_right_finger_tip_link" in name:
                            self._finger_contact_times["right"][part_id] = received_at
                    other_names = tuple(
                        name
                        for name in pair
                        if not collision_belongs_to_part(name, part_id)
                    )
                    for station_id in (
                        "raw_bin",
                        "finished_bin",
                        "machine_1",
                        "machine_2",
                        "machine_3",
                    ):
                        if any(
                            support_collision_matches_station(name, station_id)
                            for name in other_names
                        ):
                            self._support_contact_times[(part_id, station_id)] = (
                                received_at
                            )

    def _require_part(self, part_id: str) -> None:
        if part_id not in self._part_ids:
            raise ValueError(f"unknown workpiece: {part_id}")


def _rotate(point: Point3, quaternion: Quaternion) -> Point3:
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm <= 1.0e-12:
        raise ValueError("transform quaternion must be non-zero")
    qx = quaternion.x / norm
    qy = quaternion.y / norm
    qz = quaternion.z / norm
    qw = quaternion.w / norm
    dot = qx * point.x + qy * point.y + qz * point.z
    cross_x = qy * point.z - qz * point.y
    cross_y = qz * point.x - qx * point.z
    cross_z = qx * point.y - qy * point.x
    scale = qw * qw - qx * qx - qy * qy - qz * qz
    return Point3(
        scale * point.x + 2.0 * dot * qx + 2.0 * qw * cross_x,
        scale * point.y + 2.0 * dot * qy + 2.0 * qw * cross_y,
        scale * point.z + 2.0 * dot * qz + 2.0 * qw * cross_z,
    )
