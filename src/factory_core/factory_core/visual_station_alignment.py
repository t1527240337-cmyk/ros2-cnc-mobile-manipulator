"""Bounded AprilTag servo for the final centimetres of station alignment."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


# Station refinement stays slower than Nav2 transit, but it must not spend
# tens of seconds correcting a few centimetres of residual error.
VISUAL_APPROACH_MAX_LINEAR_SPEED = 0.15
VISUAL_APPROACH_PROPORTIONAL_GAIN = 2.0
VISUAL_SIDE_STEP_LINEAR_SPEED = 0.12
ODOMETRY_CORRECTION_MIN_LINEAR_SPEED = 0.05
ODOMETRY_CORRECTION_MAX_LINEAR_SPEED = 0.14
# A cached detection must never drive the base for a full tag-loss timeout.
# Hold one command for slightly longer than a 5 Hz camera period, then stop
# until a new image has been processed.
VISUAL_COMMAND_MAX_OBSERVATION_AGE = 0.30


@dataclass(frozen=True)
class TagAlignmentTarget:
    """Desired pose of a station tag in the robot base frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class AlignmentError:
    """Planar error measured directly from the visible station tag."""

    longitudinal: float
    lateral: float
    heading: float

    def within(self, *, position: float, heading: float) -> bool:
        return (
            abs(self.longitudinal) <= position
            and abs(self.lateral) <= position
            and abs(self.heading) <= heading
        )


class ObservationFreshnessClock:
    """Measure image progress in the clock domain that drives the camera.

    Gazebo camera and AprilTag stamps advance in simulation time. On a slow
    WSL host, several wall-clock seconds can pass between two 5 Hz simulated
    images even though the sensor has not missed a frame in simulation. A
    wall-clock freshness test would therefore invent a tag loss and command a
    needless search turn. The physical stack enables ``use_sim_time``; a
    system-time node keeps the ordinary monotonic-clock behaviour.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        try:
            self._uses_simulation_time = bool(
                node.get_parameter("use_sim_time").value
            )
        except Exception:  # pragma: no cover - defensive for test doubles
            self._uses_simulation_time = False

    def now(self) -> float:
        """Return seconds in the active sensor-progress clock."""
        if self._uses_simulation_time:
            return self._node.get_clock().now().nanoseconds * 1e-9
        return time.monotonic()


def wrap_angle(angle: float) -> float:
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def observation_may_drive(age: float) -> bool:
    """Return whether a cached tag observation may still command motion."""
    return 0.0 <= age <= VISUAL_COMMAND_MAX_OBSERVATION_AGE


def tag_position_in_odometry(
    base_pose: "OdometryPose", tag_x: float, tag_y: float
) -> tuple[float, float]:
    """Project a fresh base-frame tag observation into odometry.

    The result is retained only to aim the camera during a bounded
    reacquisition manoeuvre. It is never accepted as a current alignment
    measurement after the TF stamp has stopped advancing.
    """
    cosine = math.cos(base_pose.yaw)
    sine = math.sin(base_pose.yaw)
    return (
        base_pose.x + cosine * tag_x - sine * tag_y,
        base_pose.y + sine * tag_x + cosine * tag_y,
    )


def tag_reacquisition_heading(
    base_pose: "OdometryPose",
    tag_position: tuple[float, float],
    target: TagAlignmentTarget,
    *,
    search_offset: float = 0.0,
) -> float:
    """Aim the camera at the last physically observed tag position."""
    tag_bearing = math.atan2(
        tag_position[1] - base_pose.y,
        tag_position[0] - base_pose.x,
    )
    calibrated_bearing = math.atan2(target.y, target.x)
    return wrap_angle(tag_bearing - calibrated_bearing + search_offset)


def planar_heading_from_tag_normal(
    qx: float, qy: float, qz: float, qw: float
) -> float:
    """Measure station heading from a vertical tag without Euler singularity.

    The visible face normal is the tag frame's +Z axis. At the calibrated bin
    pose it points along the robot's -X axis, so its horizontal deflection is
    the base heading error even when the tag pitch is close to 90 degrees.
    """
    normal_x = 2.0 * (qx * qz + qw * qy)
    normal_y = 2.0 * (qy * qz - qw * qx)
    if math.hypot(normal_x, normal_y) < 0.5:
        raise ValueError("tag normal is too close to vertical")
    return math.atan2(normal_y, -normal_x)


def alignment_error(
    tag_x: float,
    tag_y: float,
    tag_yaw: float,
    target: TagAlignmentTarget,
) -> AlignmentError:
    """Compare a perceived tag pose with its calibrated docking pose."""
    return AlignmentError(
        longitudinal=tag_x - target.x,
        lateral=tag_y - target.y,
        heading=wrap_angle(tag_yaw - target.yaw),
    )


def velocity_for_alignment(error: AlignmentError) -> tuple[float, float]:
    """Compute conservative differential-drive commands from tag error."""
    linear = max(-0.08, min(0.12, 1.2 * error.longitudinal))
    if abs(error.longitudinal) < 0.05:
        # Commands below 20 mm/s can be swallowed by the simulated contact
        # and velocity pipeline. The alignment loop stops before publishing
        # once the calibrated position tolerance is satisfied.
        linear = math.copysign(max(0.02, abs(linear) * 0.5), linear)
    angular = max(
        -0.35,
        min(0.35, 2.0 * error.lateral + 1.5 * error.heading),
    )
    return linear, angular


def select_odometry_correction(
    error: "BasePoseError",
    *,
    lateral_tolerance: float,
    heading_tolerance: float,
    heading_trigger_margin: float,
    heading_available: bool = True,
    lateral_available: bool = True,
) -> str | None:
    """Choose one observable coarse correction in a stable order.

    A differential-drive side step is defined in the station heading frame.
    Executing it while the base still has a large heading residual rotates
    the correction direction and can push the station Tag out of view.
    """
    if abs(error.heading) > heading_tolerance + heading_trigger_margin:
        return "heading" if heading_available else None
    if abs(error.lateral) > lateral_tolerance and lateral_available:
        return "lateral"
    return None


class VisualStationAligner:
    """Close the map-to-station residual after Nav2 coarse docking."""

    # The side step is executed from continuous odometry and is followed by a
    # fresh Tag measurement, so the marker need not remain visible mid-move.
    # A 45 degree diagonal corrects a 20 cm cross-track miss in 27 cm instead
    # of the former 56 cm shallow diagonal. The base turns in place inside a
    # surveyed 0.65 m station corridor, then returns to station heading before
    # requesting a new image; no stale Tag observation controls this motion.
    _SIDE_STEP_ANGLE = math.radians(45.0)
    _SIDE_STEP_RESIDUAL = 0.008
    _MAX_SIDE_STEP_DISTANCE = 0.60
    _ODOM_HEADING_TOLERANCE = 0.025
    # Odometry correction keeps enough minimum torque to move the loaded base,
    # then reduces angular speed near the target. Do not use it for a small
    # residual which the continuously observed visual controller can remove
    # without risking that the tag leaves the camera field of view.
    _ODOM_HEADING_TRIGGER_MARGIN = 0.020
    _HEADING_REACQUISITION_WAIT = 2.0
    _REACQUISITION_SEARCH_OFFSETS = (
        0.0,
        math.radians(12.0),
        math.radians(-12.0),
    )

    def __init__(
        self,
        node: Node,
        callback_wait: Callable[[], None] | None = None,
    ) -> None:
        self._node = node
        self._callback_wait = callback_wait
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._velocity_publisher = node.create_publisher(
            Twist, "/cmd_vel_docking", 10
        )

    def align(
        self,
        tag_frame: str,
        target: TagAlignmentTarget,
        *,
        timeout_sec: float,
        position_tolerance: float = 0.015,
        lateral_tolerance: float | None = None,
        heading_tolerance: float = 0.035,
        observation_timeout: float = 1.5,
        longitudinal_only: bool = False,
        control_lateral: bool = True,
        heading_from_tag_normal: bool = False,
        use_odometry_corrections: bool = True,
    ) -> AlignmentError:
        """Servo to a visible tag and require several consecutive good samples."""
        if lateral_tolerance is None:
            lateral_tolerance = position_tolerance
        if lateral_tolerance <= 0.0:
            raise ValueError("lateral_tolerance must be positive")
        deadline = time.monotonic() + timeout_sec
        next_log_time = time.monotonic()
        observation_clock = ObservationFreshnessClock(self._node)
        stable_samples = 0
        last_error: AlignmentError | None = None
        last_observation_age: float | None = None
        last_observation_stamp: int | None = None
        last_stamp_change_time = observation_clock.now()
        required_new_stamp: int | None = None
        fresh_observation_wait_started: float | None = None
        heading_recovery_yaw: float | None = None
        last_visible_tag_position: tuple[float, float] | None = None
        reacquisition_attempt = 0
        completed_odometry_corrections: set[str] = set()
        pose_controller = None
        if not longitudinal_only:
            pose_controller = PoseAlignmentController(
                lateral_tolerance,
                control_lateral=control_lateral,
            )

        try:
            while time.monotonic() < deadline:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        "base_link", tag_frame, Time()
                    )
                except TransformException:
                    self._stop()
                    self._spin_once()
                    continue

                # Compare TF stamp progression with a monotonic wall clock.
                # CLI nodes may use system time while Gazebo publishes /clock,
                # so subtracting the two ROS epochs would reject every sample.
                observation_stamp = Time.from_msg(transform.header.stamp).nanoseconds
                if required_new_stamp is not None:
                    if observation_stamp == required_new_stamp:
                        self._stop()
                        now = observation_clock.now()
                        if fresh_observation_wait_started is None:
                            fresh_observation_wait_started = now
                        if (
                            now - fresh_observation_wait_started
                            >= self._HEADING_REACQUISITION_WAIT
                        ):
                            if last_visible_tag_position is None:
                                raise RuntimeError(
                                    f"cannot reacquire {tag_frame}: no fresh "
                                    "tag position was observed"
                                )
                            if reacquisition_attempt >= len(
                                self._REACQUISITION_SEARCH_OFFSETS
                            ):
                                raise RuntimeError(
                                    f"could not reacquire {tag_frame} after "
                                    f"{reacquisition_attempt} bounded camera "
                                    "headings"
                                )
                            self._node.get_logger().warning(
                                f"Actively reacquiring {tag_frame} from the "
                                "last fresh observation "
                                f"(attempt {reacquisition_attempt + 1})"
                            )
                            self._turn_camera_toward_tag(
                                last_visible_tag_position,
                                target,
                                reacquisition_attempt,
                                deadline,
                            )
                            reacquisition_attempt += 1
                            heading_recovery_yaw = None
                            fresh_observation_wait_started = observation_clock.now()
                        if time.monotonic() >= next_log_time:
                            self._node.get_logger().info(
                                f"Waiting for a new {tag_frame} observation "
                                "after the odometry correction"
                            )
                            next_log_time = time.monotonic() + 1.0
                        self._spin_once()
                        continue
                    required_new_stamp = None
                    fresh_observation_wait_started = None
                    heading_recovery_yaw = None
                    last_observation_stamp = None
                    reacquisition_attempt = 0
                stamp_changed = observation_stamp != last_observation_stamp
                if stamp_changed:
                    last_observation_stamp = observation_stamp
                    last_stamp_change_time = observation_clock.now()
                last_observation_age = max(
                    0.0, observation_clock.now() - last_stamp_change_time
                )
                if last_observation_age > observation_timeout:
                    stable_samples = 0
                    self._stop()
                    if last_visible_tag_position is not None:
                        self._node.get_logger().warning(
                            f"{tag_frame} left the camera view; actively "
                            "returning toward its last fresh odometry position"
                        )
                        self._turn_camera_toward_tag(
                            last_visible_tag_position,
                            target,
                            reacquisition_attempt,
                            deadline,
                        )
                        reacquisition_attempt += 1
                        required_new_stamp = observation_stamp
                        fresh_observation_wait_started = observation_clock.now()
                        stable_samples = 0
                        if not longitudinal_only:
                            pose_controller = PoseAlignmentController(
                                lateral_tolerance,
                                control_lateral=control_lateral,
                            )
                        continue
                    if time.monotonic() >= next_log_time:
                        self._node.get_logger().warning(
                            f"Waiting for a fresh {tag_frame} observation; "
                            f"TF stamp unchanged for {last_observation_age:.2f} s"
                        )
                        next_log_time = time.monotonic() + 1.0
                    self._spin_once()
                    continue

                if not stamp_changed:
                    # Do not integrate the same delayed detection repeatedly.
                    # Holding a command for one camera period preserves motion;
                    # after that the base stops until a new image is processed.
                    # Stable acceptance therefore uses independent images.
                    if not observation_may_drive(last_observation_age):
                        self._stop()
                    self._spin_once()
                    continue

                translation = transform.transform.translation
                rotation = transform.transform.rotation
                if stamp_changed:
                    last_visible_tag_position = tag_position_in_odometry(
                        self._lookup_odometry_pose(),
                        translation.x,
                        translation.y,
                    )
                if heading_from_tag_normal:
                    tag_yaw = -planar_heading_from_tag_normal(
                        rotation.x, rotation.y, rotation.z, rotation.w
                    )
                else:
                    tag_yaw = math.atan2(
                        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                        1.0
                        - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
                    )
                last_error = alignment_error(
                    translation.x, translation.y, tag_yaw, target
                )
                pose_error = (
                    None
                    if longitudinal_only
                    else base_pose_error(last_error, target)
                )
                if (
                    pose_error is not None
                    and control_lateral
                    and use_odometry_corrections
                ):
                    correction = select_odometry_correction(
                        pose_error,
                        lateral_tolerance=lateral_tolerance,
                        heading_tolerance=heading_tolerance,
                        heading_trigger_margin=(
                            self._ODOM_HEADING_TRIGGER_MARGIN
                        ),
                        heading_available=(
                            "heading" not in completed_odometry_corrections
                        ),
                        lateral_available="lateral" not in completed_odometry_corrections,
                    )
                    if correction == "lateral":
                        self._node.get_logger().info(
                            "Starting bounded odometry side step: "
                            f"base_dx={pose_error.longitudinal:.3f} m, "
                            f"base_dy={pose_error.lateral:.3f} m, "
                            f"base_dyaw={pose_error.heading:.3f} rad"
                        )
                        self._execute_lateral_correction(pose_error, deadline)
                        completed_odometry_corrections.add("lateral")
                        required_new_stamp = observation_stamp
                        fresh_observation_wait_started = observation_clock.now()
                        heading_recovery_yaw = None
                        stable_samples = 0
                        pose_controller = PoseAlignmentController(
                            lateral_tolerance,
                            control_lateral=control_lateral,
                        )
                        continue
                    if correction == "heading":
                        self._node.get_logger().info(
                            "Starting bounded odometry heading correction: "
                            f"base_dyaw={pose_error.heading:.3f} rad"
                        )
                        heading_recovery_yaw = (
                            self._lookup_odometry_pose().yaw
                        )
                        self._execute_heading_correction(
                            pose_error, deadline
                        )
                        completed_odometry_corrections.add("heading")
                        required_new_stamp = observation_stamp
                        fresh_observation_wait_started = observation_clock.now()
                        stable_samples = 0
                        pose_controller = PoseAlignmentController(
                            lateral_tolerance,
                            control_lateral=control_lateral,
                        )
                        continue
                if longitudinal_only:
                    is_aligned = abs(last_error.longitudinal) <= position_tolerance
                    control_error = AlignmentError(
                        longitudinal=last_error.longitudinal,
                        lateral=0.0,
                        heading=0.0,
                    )
                else:
                    assert pose_error is not None
                    # Control and acceptance must use the same station-goal
                    # frame.  Raw tag x/y components are coupled to heading by
                    # the tag's non-zero lever arm from the base target.
                    is_aligned = pose_error.within(
                        position=position_tolerance,
                        lateral_position=(
                            lateral_tolerance if control_lateral else math.inf
                        ),
                        heading=heading_tolerance,
                    )
                    control_error = last_error
                if is_aligned:
                    stable_samples += 1
                    self._stop()
                    if stable_samples >= 3:
                        return last_error
                else:
                    stable_samples = 0
                    if longitudinal_only:
                        linear, angular = velocity_for_alignment(control_error)
                    else:
                        assert pose_controller is not None
                        linear, angular = pose_controller.command(control_error, target)
                    command = Twist()
                    command.linear.x = linear
                    command.angular.z = angular
                    self._velocity_publisher.publish(command)
                if time.monotonic() >= next_log_time:
                    self._node.get_logger().info(
                        "Visual alignment sample: "
                        f"dx={last_error.longitudinal:.3f} m, "
                        f"dy={last_error.lateral:.3f} m, "
                        f"dyaw={last_error.heading:.3f} rad, "
                        f"phase={'longitudinal' if pose_controller is None else pose_controller.phase}, "
                        f"age={last_observation_age:.2f} s"
                    )
                    next_log_time = time.monotonic() + 1.0
                self._spin_once()
        finally:
            self._stop()
            for _ in range(3):
                self._spin_once()

        detail = "tag transform unavailable"
        if last_observation_age is not None and last_observation_age > observation_timeout:
            detail = (
                f"tag TF stamp unchanged for {last_observation_age:.2f} s"
            )
        elif last_error is not None:
            detail = (
                f"longitudinal={last_error.longitudinal:.3f} m, "
                f"lateral={last_error.lateral:.3f} m, "
                f"heading={last_error.heading:.3f} rad"
            )
        raise RuntimeError(f"visual station alignment timed out: {detail}")

    def _turn_camera_toward_tag(
        self,
        tag_position: tuple[float, float],
        target: TagAlignmentTarget,
        attempt: int,
        deadline: float,
    ) -> None:
        """Execute one bounded view-recovery heading from odometry."""
        pose = self._lookup_odometry_pose()
        target_yaw = tag_reacquisition_heading(
            pose,
            tag_position,
            target,
            search_offset=self._REACQUISITION_SEARCH_OFFSETS[attempt],
        )
        self._rotate_to_heading(target_yaw, deadline)
        self._stop()

    def _execute_heading_correction(
        self,
        error: BasePoseError,
        deadline: float,
    ) -> None:
        """Finish a visually measured turn using continuous odometry."""
        initial_pose = self._lookup_odometry_pose()
        station_heading = wrap_angle(initial_pose.yaw - error.heading)
        self._rotate_to_heading(station_heading, deadline)
        self._stop()
        self._node.get_logger().info(
            "Odometry heading correction complete; "
            "requesting a fresh tag observation"
        )

    def _execute_lateral_correction(
        self,
        error: BasePoseError,
        deadline: float,
    ) -> None:
        """Correct lateral error without requiring the tag to remain visible.

        A differential-drive base cannot translate sideways.  The bounded
        manoeuvre therefore turns away from the station, reverses along a
        measured odometry segment, and turns back to the calibrated station
        heading.  Visual control resumes only after a new tag frame arrives.
        """
        initial_pose = self._lookup_odometry_pose()
        station_heading = wrap_angle(initial_pose.yaw - error.heading)
        side = math.copysign(1.0, error.lateral)
        side_step_heading = wrap_angle(
            station_heading + side * self._SIDE_STEP_ANGLE
        )
        distance = (
            abs(error.lateral) - self._SIDE_STEP_RESIDUAL
        ) / math.sin(self._SIDE_STEP_ANGLE)
        if distance > self._MAX_SIDE_STEP_DISTANCE:
            raise RuntimeError(
                "lateral docking error is outside the bounded correction "
                f"domain: required={distance:.3f} m, "
                f"limit={self._MAX_SIDE_STEP_DISTANCE:.3f} m"
            )

        self._rotate_to_heading(side_step_heading, deadline)
        self._reverse_by_odometry(distance, side_step_heading, deadline)
        self._rotate_to_heading(station_heading, deadline)
        self._stop()
        self._node.get_logger().info(
            "Odometry side step complete; requesting a fresh tag observation"
        )

    def _rotate_to_heading(self, target_yaw: float, deadline: float) -> None:
        stable_samples = 0
        while time.monotonic() < deadline:
            pose = self._lookup_odometry_pose()
            error = wrap_angle(target_yaw - pose.yaw)
            if abs(error) <= self._ODOM_HEADING_TOLERANCE:
                stable_samples += 1
                self._stop()
                if stable_samples >= 2:
                    return
            else:
                stable_samples = 0
                command = Twist()
                # Use the proven 0.50 rad/s envelope for large errors, but
                # taper near the target to avoid repeated correction overshoot.
                # The 0.18 rad/s floor still overcomes loaded-base friction.
                command.angular.z = self._bounded_command(
                    2.0 * error, minimum=0.18, maximum=0.50
                )
                self._velocity_publisher.publish(command)
            self._spin_once()
        raise RuntimeError("odometry side-step rotation timed out")

    def _reverse_by_odometry(
        self,
        distance: float,
        target_yaw: float,
        deadline: float,
    ) -> None:
        start = self._lookup_odometry_pose()
        while time.monotonic() < deadline:
            pose = self._lookup_odometry_pose()
            travelled = math.hypot(pose.x - start.x, pose.y - start.y)
            remaining = distance - travelled
            if remaining <= 0.004:
                self._stop()
                return

            command = Twist()
            command.linear.x = -min(
                ODOMETRY_CORRECTION_MAX_LINEAR_SPEED,
                max(ODOMETRY_CORRECTION_MIN_LINEAR_SPEED, remaining),
            )
            heading_error = wrap_angle(target_yaw - pose.yaw)
            command.angular.z = max(
                -0.20, min(0.20, 1.5 * heading_error)
            )
            self._velocity_publisher.publish(command)
            self._spin_once()
        raise RuntimeError("odometry side-step translation timed out")

    def _lookup_odometry_pose(self) -> OdometryPose:
        try:
            transform = self._tf_buffer.lookup_transform(
                "odom", "base_link", Time()
            )
        except TransformException as exc:
            raise RuntimeError(
                "odom to base_link transform unavailable during side step"
            ) from exc

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return OdometryPose(x=translation.x, y=translation.y, yaw=yaw)

    @staticmethod
    def _bounded_command(value: float, *, minimum: float, maximum: float) -> float:
        magnitude = min(maximum, max(minimum, abs(value)))
        return math.copysign(magnitude, value)

    def _stop(self) -> None:
        self._velocity_publisher.publish(Twist())

    def _spin_once(self) -> None:
        if self._callback_wait is not None:
            self._callback_wait()
            return
        rclpy.spin_once(self._node, timeout_sec=0.05)


@dataclass(frozen=True)
class OdometryPose:
    """Planar robot pose in the continuous odometry frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class BasePoseError:
    """Current robot pose expressed in the calibrated station-goal frame."""

    longitudinal: float
    lateral: float
    heading: float

    def within(
        self,
        *,
        position: float,
        heading: float,
        lateral_position: float | None = None,
    ) -> bool:
        lateral_limit = position if lateral_position is None else lateral_position
        return (
            abs(self.longitudinal) <= position
            and abs(self.lateral) <= lateral_limit
            and abs(self.heading) <= heading
        )


def base_pose_error(
    error: AlignmentError,
    target: TagAlignmentTarget,
) -> BasePoseError:
    """Recover the robot pose error from the observed station-tag error.

    ``alignment_error`` is expressed in the moving robot frame.  A lateral
    station-tag residual therefore also changes when the robot rotates.  This
    SE(2) composition converts it to the robot pose in the fixed, calibrated
    docking frame, which makes a bounded non-holonomic correction possible.
    """
    measured_x = target.x + error.longitudinal
    measured_y = target.y + error.lateral
    heading = error.heading
    cosine = math.cos(heading)
    sine = math.sin(heading)

    # T(base, goal) = T(base, tag) * inverse(T(goal, tag)).
    goal_x = measured_x - (cosine * target.x - sine * target.y)
    goal_y = measured_y - (sine * target.x + cosine * target.y)

    # Invert T(base, goal) to express the current base in the goal frame.
    return BasePoseError(
        longitudinal=-(cosine * goal_x + sine * goal_y),
        lateral=-(-sine * goal_x + cosine * goal_y),
        heading=-heading,
    )


class PoseAlignmentController:
    """Stateful final-pose controller for a non-holonomic mobile base."""

    # Physical camera evidence shows the close-range machine Tag leaves view
    # above roughly 6.5 degrees. A 5 degree arc preserves observation while
    # the longer reverse segment removes the final centimetre cross-track.
    _SIDE_STEP_HEADING = math.radians(5.0)
    _HEADING_TOLERANCE = 0.020
    _SIDE_STEP_HEADING_TOLERANCE = 0.020
    _SIDE_STEP_FINISH = 0.008
    # While the goal is still ahead, differential drive can remove a small
    # cross-track residual with a shallow observed arc. Turning in place for
    # a side step at that point amplifies the Tag lever-arm error and can move
    # the marker outside the camera field of view.
    _SIDE_STEP_LONGITUDINAL_TRIGGER = 0.05

    def __init__(
        self,
        position_tolerance: float,
        *,
        control_lateral: bool = True,
    ) -> None:
        self._position_tolerance = position_tolerance
        self._control_lateral = control_lateral
        self._phase = "straighten"
        self._side = 0.0

    @property
    def phase(self) -> str:
        """Human-readable phase for diagnostics and test evidence."""
        return self._phase

    def command(
        self,
        error: AlignmentError,
        target: TagAlignmentTarget,
    ) -> tuple[float, float]:
        """Advance the parking state machine and return ``(linear, angular)``."""
        pose = base_pose_error(error, target)

        # Several phase changes can require no physical command.  Iterating
        # locally keeps the external servo loop simple and publishes only the
        # command belonging to the resulting phase.
        for _ in range(5):
            if self._phase == "straighten":
                if abs(pose.heading) > self._HEADING_TOLERANCE:
                    return 0.0, self._heading_command(-pose.heading)
                if (
                    self._control_lateral
                    and abs(pose.lateral) > self._position_tolerance
                    and abs(pose.longitudinal) <= self._SIDE_STEP_LONGITUDINAL_TRIGGER
                ):
                    self._side = math.copysign(1.0, pose.lateral)
                    self._phase = "turn_for_side_step"
                else:
                    self._phase = "approach"
                continue

            if self._phase == "turn_for_side_step":
                desired_heading = self._side * self._SIDE_STEP_HEADING
                heading_error = wrap_angle(desired_heading - pose.heading)
                if abs(heading_error) > self._SIDE_STEP_HEADING_TOLERANCE:
                    return 0.0, self._heading_command(heading_error)
                self._phase = "side_step"
                continue

            if self._phase == "side_step":
                if pose.lateral * self._side <= self._SIDE_STEP_FINISH:
                    self._phase = "straighten_after_side_step"
                    continue
                desired_heading = self._side * self._SIDE_STEP_HEADING
                heading_error = wrap_angle(desired_heading - pose.heading)
                return (
                    -VISUAL_SIDE_STEP_LINEAR_SPEED,
                    self._heading_command(heading_error),
                )

            if self._phase == "straighten_after_side_step":
                if abs(pose.heading) > self._HEADING_TOLERANCE:
                    return 0.0, self._heading_command(-pose.heading)
                self._phase = "approach"
                continue

            if self._phase == "approach":
                # Re-enter a deliberate side step only after the robot is
                # straight.  This hysteresis prevents noisy phase chattering.
                if (
                    self._control_lateral
                    and abs(pose.lateral) > self._position_tolerance
                    and abs(pose.longitudinal) <= self._SIDE_STEP_LONGITUDINAL_TRIGGER
                ):
                    self._phase = "straighten"
                    continue
                linear = max(
                    -VISUAL_APPROACH_MAX_LINEAR_SPEED,
                    min(
                        VISUAL_APPROACH_MAX_LINEAR_SPEED,
                        -VISUAL_APPROACH_PROPORTIONAL_GAIN * pose.longitudinal,
                    ),
                )
                # Couple the currently observed lateral Tag residual into the
                # forward approach. This is a shallow visual arc, not a blind
                # side-step, and remains bounded by the same angular envelope.
                angular = max(
                    -0.20, min(0.20, 2.0 * error.lateral + 1.5 * error.heading)
                )
                return linear, angular

        raise RuntimeError(f"invalid pose-alignment phase: {self._phase}")

    @staticmethod
    def _heading_command(error: float) -> float:
        proportional = max(-0.40, min(0.40, 1.8 * error))
        if abs(error) <= 1e-6:
            return 0.0
        return math.copysign(max(0.15, abs(proportional)), proportional)
