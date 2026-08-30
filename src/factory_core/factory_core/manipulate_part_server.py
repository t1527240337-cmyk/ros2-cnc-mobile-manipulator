"""ROS action server for perceived or calibrated pick-and-place sequences."""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time
from typing import Callable

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from factory_interfaces.action import ManipulatePart
from moveit_msgs.msg import MoveItErrorCodes
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .fixture_clamp_client import FixtureClampClient
from .ik_reachability import CollisionAwareIk, candidate_screening_poses
from .finished_slot_perception import FinishedSlotPerception
from .manipulation_evidence import (
    ManipulationEvidence,
    Point3,
    Quaternion,
    grasp_hold_is_valid,
    loaded_hold_reseat_is_valid,
    proof_lift_reseat_is_valid,
)
from .machine_door_monitor import MachineDoorMonitor
from .gripper_client import GripperClient
from .manipulation_config import (
    CartesianPose,
    PICK_OPERATION,
    load_manipulation_stations,
    resolve_manipulation_request,
)
from .motion_client import (
    ARM_JOINTS,
    BIN_STATION_TRANSIT,
    MANIPULATION_READY,
    JointTarget,
    MoveGroupClient,
    STOWED,
    moveit_succeeded,
)
from .planning_scene_client import PlanningSceneClient, WORKPIECE_HEIGHT
from .pose_motion_client import PoseMoveGroupClient, PoseTarget
from .raw_part_perception import RawPartPerception

APPROACH_TRANSIT_HEIGHT = 0.80
# The CNC opening is clear above 1.10 m in base_link. A 1.25 m TCP target put
# the UR5e at the edge of its vertical reach after docking refinement: Pilz
# PTP could reject that exact pose while OMPL found a one-way wrist branch.
# Keeping the corridor at 1.20 m still clears the fixture and held stock, and
# gives entry and reverse egress one repeatable IK branch.
MACHINE_APPROACH_TRANSIT_HEIGHT = 1.20
MACHINE_DOOR_CLEAR_X = 0.55
MACHINE_FOLD_CLEARANCE_Y = 0.20
MACHINE_FOLD_CLEARANCE_Z = 1.20
MAX_MACHINE_PART_ALIGNMENT = 0.15
RAW_BIN_PREGRASP_HEIGHT = 0.45
GRASP_FORCE_SETTLE_TIME = 0.80
GRASP_HOLD_CONFIRMATION_TIMEOUT = 0.80
GRASP_HOLD_CONFIRMATION_SAMPLES = 3
# A round part can roll a few millimetres between compliant pads as load is
# transferred from the fixture to the gripper.  Accept that settling only
# while bilateral contact remains fresh and the measured joints are nearly
# stationary; a 10 mm aperture loss still fails as physical slip.
MAX_GRASP_SETTLING_POSITION_CHANGE = 0.008
MAX_LOADED_RESEAT_POSITION_CHANGE = 0.012
MAX_LOADED_RESEATS = 1
MAX_GRASP_SETTLING_VELOCITY = 0.020
GRIPPER_CONTACT_SEARCH_TIMEOUT = 15.0
PROOF_LIFT_DISTANCE = 0.03
# Contact compliance means the workpiece does not reproduce every millimetre
# of commanded TCP travel. Requiring a clear majority of the unassisted
# 30 mm proof lift still rejects table vibration and bouncing.
PROOF_LIFT_MINIMUM_RATIO = 0.65
PROOF_ATTACHMENT_DISTANCE_TOLERANCE = 0.008
# Carry above the CNC's 1.16 m jaw top. ``base_link`` is 0.34 m above the
# The fingertip collision model consists of two 6 x 25.5 mm faces pitched at
# 45 degrees. Their vertical projection, not just their nominal length, must
# clear the full 120 mm-tall upright workpiece during a supported release.
FINGERTIP_V_FACE_THICKNESS = 0.006
FINGERTIP_V_FACE_LENGTH = 0.0255
FINGERTIP_V_FACE_PITCH = math.radians(45.0)
SUPPORTED_RELEASE_CLEARANCE = 0.010
NOMINAL_VERTICAL_GRASP_OFFSET = 0.005


def supported_release_separation_distance(vertical_grasp_offset: float) -> float:
    """Return the axial lift that clears an upright part from both V pads.

    ``vertical_grasp_offset`` is the TCP height above the workpiece centre.
    The calculation uses the complete physical envelopes: workpiece half
    height, projected fingertip half-height and an explicit free-space margin.
    """
    if not math.isfinite(vertical_grasp_offset):
        raise ValueError("vertical grasp offset must be finite")
    if vertical_grasp_offset < 0.0:
        raise ValueError("vertical grasp offset cannot be negative")
    projected_fingertip_half_height = 0.5 * (
        FINGERTIP_V_FACE_LENGTH * math.cos(FINGERTIP_V_FACE_PITCH)
        + FINGERTIP_V_FACE_THICKNESS * math.sin(FINGERTIP_V_FACE_PITCH)
    )
    return max(
        SUPPORTED_RELEASE_CLEARANCE,
        0.5 * WORKPIECE_HEIGHT
        + projected_fingertip_half_height
        + SUPPORTED_RELEASE_CLEARANCE
        - vertical_grasp_offset,
    )


def released_part_retention_is_valid(
    station_id: str,
    *,
    support_contact: bool,
    fixture_clamped: bool | None,
) -> bool:
    """Select the physical retention evidence appropriate to a station.

    A CNC clamp carries the workpiece after release and can remove microscopic
    pallet contact in a rigid-body solver. An unclamped destination has no
    such constraint and must retain fresh load-bearing support contact.
    """

    if station_id.startswith("machine_"):
        return fixture_clamped is True
    return support_contact


SUPPORTED_RELEASE_SEPARATION_DISTANCE = supported_release_separation_distance(
    NOMINAL_VERTICAL_GRASP_OFFSET
)
SUPPORTED_RELEASE_CLEAR_TIMEOUT = 3.0
# floor, so a 1.05 m TCP target keeps both fingers and 120 mm stock clear while
# the base performs local docking. Lower carry poses put the workpiece at jaw
# height and can physically block the docking controller before it reaches its
# visual target.
LOADED_CARRY_HEIGHT = 1.05
# Keep the arm in the front-left quarter of the deck without folding the
# upright stock through the forearm. The 0.40 m reach remains outside the
# camera-to-station sightline and has 0.10 m more self-collision clearance
# than the former over-folded pose.
LOADED_CARRY_X = 0.40
LOADED_CARRY_Y = 0.45
MAX_TCP_POSITION_ERROR = 0.025
MAX_TCP_ORIENTATION_ERROR = math.radians(5.0)
DEFAULT_MAX_ANCHOR_ALIGNMENT = 0.120
# AprilTag corrects the residual left by base docking; it must not replace the
# surveyed CNC fixture pose. A 30 mm radial bound covers the physical docking
# acceptance while rejecting noisy depth estimates that push the arm toward
# the sensor mast or outside the taught machine corridor.
MAX_MACHINE_FIXTURE_REFINEMENT = 0.030
# A surveyed fixture height is the safe start of placement, not proof of
# support. Search downward by collision-checked segments and stop each active
# trajectory on fresh workpiece-to-fixture contact. One 60 mm workpiece
# half-height bounds the search without allowing motion through the fixture.
MACHINE_SEATING_STEP = 0.010
MAX_MACHINE_SEATING_DEPTH = 0.060
# The part can settle vertically inside the compliant V pads during transport.
# Search at most one 120 mm workpiece half-height, retaining bilateral grasp
# checks and stopping only on fresh part-to-bin support contact.
FINISHED_BIN_SEATING_STEP = 0.004
MAX_FINISHED_BIN_SEATING_DEPTH = 0.060
SUPPORT_CONTACT_SAMPLE_TIMEOUT = 0.45
# A guarded placement may stop above its surveyed TCP target when the held
# workpiece reaches the support first. This uses the same grasp-compliance
# envelope as the bounded seating search without allowing a distant contact
# to authorize release.
MAX_GUARDED_SUPPORT_HORIZONTAL_ERROR = 0.020
MAX_GUARDED_SUPPORT_VERTICAL_ERROR = 0.030

_EARLY_MOTION_COMPLETION = object()
ARM_REPLAN_SETTLE_TOLERANCE = 0.002
ARM_REPLAN_SETTLE_SAMPLES = 3
ARM_REPLAN_SETTLE_TIMEOUT = 4.0



def machine_seating_depths() -> tuple[float, ...]:
    """Return the bounded sequence used by contact-guided CNC placement."""

    count = round(MAX_MACHINE_SEATING_DEPTH / MACHINE_SEATING_STEP)
    return tuple(MACHINE_SEATING_STEP * index for index in range(1, count + 1))


def finished_bin_seating_depths() -> tuple[float, ...]:
    """Return a bounded table-contact search for mobile-base height variation."""
    count = round(MAX_FINISHED_BIN_SEATING_DEPTH / FINISHED_BIN_SEATING_STEP)
    return tuple(FINISHED_BIN_SEATING_STEP * index for index in range(1, count + 1))

def guarded_support_pose_is_safe(
    measured: Point3,
    target: tuple[float, float, float],
) -> bool:
    """Bound the TCP pose accepted after a contact-triggered stop."""

    horizontal_error = math.hypot(
        measured.x - target[0], measured.y - target[1]
    )
    vertical_error = abs(measured.z - target[2])
    return (
        horizontal_error <= MAX_GUARDED_SUPPORT_HORIZONTAL_ERROR
        and vertical_error <= MAX_GUARDED_SUPPORT_VERTICAL_ERROR
    )


def supported_part_center_from_tcp(
    measured_tcp: Point3, vertical_grasp_offset: float
) -> tuple[float, float, float]:
    """Recover a supported workpiece centre from the measured release TCP."""
    if not math.isfinite(vertical_grasp_offset):
        raise ValueError("vertical grasp offset must be finite")
    if vertical_grasp_offset < 0.0:
        raise ValueError("vertical grasp offset cannot be negative")
    return (
        measured_tcp.x,
        measured_tcp.y,
        measured_tcp.z - vertical_grasp_offset,
    )

def arm_joint_sample_is_stable(
    previous: tuple[float, ...],
    current: tuple[float, ...],
    *,
    tolerance: float = ARM_REPLAN_SETTLE_TOLERANCE,
) -> bool:
    """Return whether every arm joint is stationary across two fresh samples."""
    if tolerance <= 0.0:
        raise ValueError("arm settle tolerance must be positive")
    if len(previous) != len(current) or not previous:
        raise ValueError("arm samples must be non-empty and equally sized")
    return max(abs(now - before) for before, now in zip(previous, current)) <= tolerance


def uses_direct_bin_place_transit(station_name: str) -> bool:
    """Return whether loaded placement owns a safe bin-specific first leg.

    The carried-part path already enters ``BIN_STATION_TRANSIT``. Sending it
    through the empty-arm ready posture first would add motion and can sweep
    the held cylinder toward the mast.
    """
    return station_name in {"raw_bin", "finished_bin"}


def uses_direct_pick_transit(_station_name: str, station_role: str) -> bool:
    """Return whether a pick may skip the collision-clearing unfold.

    Raw-bin picks unfold above the deck before entering their taught branch.
    CNC picks already start through a high, collision-checked open-door
    corridor and may safely skip that generic waypoint.
    """
    return station_role == "machine"


def tcp_position_error(
    actual: Point3, target: tuple[float, float, float]
) -> float:
    """Return Euclidean TCP position error in metres."""
    return math.dist((actual.x, actual.y, actual.z), target)


def tcp_orientation_error(
    actual: Quaternion, target: tuple[float, float, float, float]
) -> float:
    """Return the shortest quaternion angular distance in radians."""
    dot = abs(
        actual.x * target[0]
        + actual.y * target[1]
        + actual.z * target[2]
        + actual.w * target[3]
    )
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def resolve_machine_target_position(
    perceived_target: Point3,
    configured_position: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Validate and use the observed CNC datum in the horizontal plane.

    The AprilTag-to-loading-datum transform is a mechanical calibration, not
    an optional correction.  A disagreement larger than the calibration
    tolerance means the base or marker observation is unsafe; silently
    clipping that disagreement would command a repeatably wrong grasp.  The
    fixture height remains the surveyed value because the CNC is floor-fixed.
    """
    horizontal_error = math.hypot(
        perceived_target.x - configured_position[0],
        perceived_target.y - configured_position[1],
    )
    if horizontal_error > MAX_MACHINE_FIXTURE_REFINEMENT:
        raise ValueError(
            "observed CNC loading datum differs from calibration by "
            f"{horizontal_error:.3f} m: observed=({perceived_target.x:.3f}, "
            f"{perceived_target.y:.3f}), calibrated=({configured_position[0]:.3f}, "
            f"{configured_position[1]:.3f}), limit={MAX_MACHINE_FIXTURE_REFINEMENT:.3f} m"
        )

    return (
        perceived_target.x,
        perceived_target.y,
        configured_position[2],
    )


def resolve_observed_machine_part_center(
    perceived: Point3,
    configured_position: tuple[float, float, float],
    *,
    maximum_vertical_distance: float,
) -> tuple[float, float, float]:
    """Use a measured CNC workpiece centre after a bounded sanity check."""
    if maximum_vertical_distance <= 0.0:
        raise ValueError("maximum vertical distance must be positive")
    vertical_error = abs(perceived.z - configured_position[2])
    if vertical_error > maximum_vertical_distance:
        raise ValueError(
            "observed CNC workpiece height differs from its safe work volume "
            f"by {vertical_error:.3f} m: observed={perceived.z:.3f}, "
            f"nominal={configured_position[2]:.3f}, "
            f"limit={maximum_vertical_distance:.3f} m"
        )
    return (perceived.x, perceived.y, perceived.z)


def aligned_grasp_poses_from_measured_tcp(
    measured_position: Point3,
    measured_orientation: Quaternion,
    correction: Point3,
    approach: CartesianPose,
    grasp: CartesianPose,
) -> tuple[CartesianPose, CartesianPose]:
    """Move a measured TCP without asking IK to select a new wrist branch.

    MoveIt permits a small arrival tolerance, so the physical TCP can differ
    slightly from the commanded approach. An independent Gazebo correction
    must be applied to that measured pose, not added to the old command. The
    grasp keeps the same approach-to-grasp vector and measured orientation.
    """
    if approach.frame_id != "base_link" or grasp.frame_id != "base_link":
        raise ValueError("physical anchor alignment requires base_link poses")

    approach_to_grasp = tuple(
        grasp_axis - approach_axis
        for grasp_axis, approach_axis in zip(
            grasp.position, approach.position, strict=True
        )
    )
    aligned_position = (
        measured_position.x + correction.x,
        measured_position.y + correction.y,
        measured_position.z + correction.z,
    )
    orientation = (
        measured_orientation.x,
        measured_orientation.y,
        measured_orientation.z,
        measured_orientation.w,
    )
    aligned_approach = CartesianPose(
        frame_id="base_link",
        position=aligned_position,
        orientation=orientation,
    )
    aligned_grasp = CartesianPose(
        frame_id="base_link",
        position=tuple(
            axis + delta
            for axis, delta in zip(
                aligned_position, approach_to_grasp, strict=True
            )
        ),
        orientation=orientation,
    )
    return aligned_approach, aligned_grasp


def planner_attempts(planner: str) -> tuple[tuple[int, str], ...]:
    """Return bounded collision-checked planner candidates for one move."""
    if planner == "bin_approach_lin":
        # Preserve the station-transit wrist branch. The second PTP attempt
        # avoids a sampled multi-turn joint solution which is
        # valid while empty but cannot retract after the grasp.
        return ((1, "lin"), (2, "ptp"))
    if planner == "ptp":
        # A sampled OMPL plan can recover when the seed-local PTP IK fails.
        return ((1, "ptp"), (2, "ompl"))
    if planner == "loaded_transport_lin":
        # Prefer the direct, orientation-preserving carry corridor. A
        # collision or seed-local IK failure may select the PTP candidate
        # because no failed candidate trajectory was executed.
        return ((1, "loaded_transport_lin"), (2, "loaded_ptp"))
    if planner == "loaded_egress_lin":
        # The TCP is still inside the CNC door. A joint-space alternative can
        # intersect the lintel or side posts, so retry only the safe Cartesian
        # corridor from the latest measured state.
        return ((1, "loaded_egress_lin"), (2, "loaded_egress_lin"))
    if planner == "loaded_ompl":
        # The carry pose has a validated IK branch. Pilz PTP provides a
        # deterministic second candidate when OMPL cannot sample that narrow branch.
        return ((1, "loaded_ompl"), (2, "loaded_ptp"))
    if planner == "loaded_ptp":
        return ((1, "loaded_ptp"), (2, "loaded_ompl"))
    return ((1, planner), (2, planner))


def machine_egress_free_space_planner(door_planner: str) -> str:
    """Choose a joint-stable planner after the TCP clears the CNC door.

    Cartesian LIN is required while crossing the confined door plane. Once
    outside, a sampled or PTP joint path avoids discontinuous Cartesian IK
    branches during the lateral fold toward the mobile base.
    """
    if door_planner in {"lin", "fixture_empty_lin"}:
        return "ptp"
    if door_planner in {
        "loaded_lin",
        "fixture_loaded_lin",
        "loaded_transport_lin",
    }:
        return "loaded_ptp"
    return door_planner


def machine_clearance_waypoints(
    transit: CartesianPose,
) -> tuple[CartesianPose, CartesianPose, CartesianPose]:
    """Return the shared door, fold, and travel poses for CNC motion.

    Entry and exit must use the same Cartesian corridor in opposite
    directions. Keeping the geometry in one helper prevents a tested egress
    path from silently drifting away from the loading path.
    """
    x, y, z = transit.position
    door_clear = CartesianPose(
        frame_id=transit.frame_id,
        position=(min(x, MACHINE_DOOR_CLEAR_X), y, z),
        orientation=transit.orientation,
    )
    fold_clearance = CartesianPose(
        frame_id=transit.frame_id,
        position=(
            MACHINE_DOOR_CLEAR_X,
            MACHINE_FOLD_CLEARANCE_Y,
            MACHINE_FOLD_CLEARANCE_Z,
        ),
        orientation=transit.orientation,
    )
    travel = CartesianPose(
        frame_id=transit.frame_id,
        position=(LOADED_CARRY_X, LOADED_CARRY_Y, LOADED_CARRY_HEIGHT),
        orientation=transit.orientation,
    )
    return door_clear, fold_clearance, travel


def _default_config_path() -> str:
    share = Path(get_package_share_directory("factory_core"))
    return str(share / "config" / "manipulation.yaml")


class ExecutionFailure(RuntimeError):
    """An expected action failure with a stable public error code."""

    def __init__(
        self,
        code: int,
        message: str,
        *,
        safe_to_retry: bool = True,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.safe_to_retry = safe_to_retry


class ManipulatePartServer(Node):
    """Execute one readable manipulation sequence at a time."""

    def __init__(self) -> None:
        super().__init__("factory_manipulate_part")
        self.declare_parameter("config", _default_config_path())
        config_path = str(self.get_parameter("config").value)
        self._stations = load_manipulation_stations(config_path)

        physical_part_ids = tuple(
            str(value)
            for value in self.declare_parameter(
                "physical_workpiece_ids",
                [f"raw_part_{index}" for index in range(1, 7)],
            ).value
        )
        if not physical_part_ids or len(set(physical_part_ids)) != len(
            physical_part_ids
        ):
            raise ValueError(
                "physical_workpiece_ids must be non-empty and unique"
            )
        self._pose_motion = PoseMoveGroupClient(self)
        self._joint_motion = MoveGroupClient(self)
        self._gripper = GripperClient(self)
        self._fixture_clamp = FixtureClampClient(self, physical_part_ids)
        machine_ids = tuple(
            station.name
            for station in self._stations.values()
            if station.role == "machine"
        )
        self._machine_doors = MachineDoorMonitor(self, machine_ids)
        self._arm_positions: tuple[float, ...] | None = None
        self._arm_state_sample: tuple[float, tuple[float, ...]] | None = None
        self._arm_state_subscription = self.create_subscription(
            JointState, "/joint_states", self._remember_arm_state, 10
        )
        self._planning_scene = PlanningSceneClient(self)
        self._ik_reachability = CollisionAwareIk(self)
        self._evidence = ManipulationEvidence(self, physical_part_ids)
        self._raw_part_perception = RawPartPerception(
            self,
            additional_topics=("/perception/raw_part_candidates_aux",),
        )
        self._machine_part_perception = RawPartPerception(
            self, topic="/perception/machine_part_candidates"
        )
        self._finished_slots = FinishedSlotPerception(self)
        self._raw_part_perception_timeout = float(
            self.declare_parameter("raw_part_perception_timeout", 8.0).value
        )
        self._raw_part_candidate_distance = float(
            self.declare_parameter(
                "raw_part_selection_radius", 0.50
            ).value
        )
        self._machine_part_perception_timeout = float(
            self.declare_parameter(
                "machine_part_perception_timeout", 8.0
            ).value
        )
        self._machine_part_candidate_distance = float(
            self.declare_parameter(
                "machine_part_candidate_distance", 0.08
            ).value
        )
        self._machine_part_vertical_distance = float(
            self.declare_parameter(
                "machine_part_vertical_distance", 0.08
            ).value
        )
        if not bool(
            self.declare_parameter("use_finished_slot_perception", True).value
        ):
            raise ValueError(
                "finished-bin placement requires RGB-D occupancy perception"
            )
        self._finished_slot_confirmation_timeout = float(
            self.declare_parameter(
                "finished_slot_confirmation_timeout", 10.0
            ).value
        )
        if (
            self._raw_part_perception_timeout <= 0.0
            or self._raw_part_candidate_distance <= 0.0
            or self._machine_part_perception_timeout <= 0.0
            or self._machine_part_candidate_distance <= 0.0
            or self._machine_part_vertical_distance <= 0.0
            or self._finished_slot_confirmation_timeout <= 0.0
        ):
            raise ValueError("perception timeouts and limits must be positive")
        if not bool(
            self.declare_parameter("use_raw_part_perception", True).value
        ):
            raise ValueError(
                "raw-bin manipulation requires RGB-D perception; "
                "a calibrated-position fallback is not supported"
            )
        if not bool(
            self.declare_parameter("use_machine_part_perception", True).value
        ):
            raise ValueError(
                "machine unloading requires RGB-D workpiece perception; "
                "a nominal-fixture fallback is not supported"
            )
        self._motion_execution_timeout = float(
            self.declare_parameter("motion_execution_timeout", 120.0).value
        )
        if self._motion_execution_timeout <= 0.0:
            raise ValueError("motion_execution_timeout must be positive")
        # Logical IDs belong to orders; physical IDs belong only to the
        # Gazebo contact / detachable-joint adapter. The mapping is created
        # after bilateral tactile contact identifies the selected entity.
        self._logical_to_physical: dict[str, str] = {}
        self._raw_scene_candidates: tuple[
            tuple[float, float, float], ...
        ] = ()
        self._held_part_id: str | None = None
        self._held_physical_part_id: str | None = None
        self._held_part_vertical_offset: float | None = None
        self._held_gripper_position: float | None = None
        self._loaded_reseat_count = 0
        self._gripper_effort_active = False
        self._last_grasp_monitor_log = 0.0
        self._execution_lock = threading.Lock()
        self._server = ActionServer(
            self,
            ManipulatePart,
            "/manipulate_part",
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    def _accept_goal(self, request: ManipulatePart.Goal) -> GoalResponse:
        try:
            resolve_manipulation_request(
                self._stations,
                int(request.operation),
                request.station_id,
                request.part_id,
                int(request.placement_slot_id),
            )
        except ValueError as error:
            self.get_logger().warning(f"Rejecting manipulation request: {error}")
            return GoalResponse.REJECT

        if not self._execution_lock.acquire(blocking=False):
            self.get_logger().warning("Rejecting manipulation request: robot is busy")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> ManipulatePart.Result:
        request = goal_handle.request
        try:
            station, target = resolve_manipulation_request(
                self._stations,
                int(request.operation),
                request.station_id,
                request.part_id,
                int(request.placement_slot_id),
            )
            self._require_dependencies()
            if station.role == "machine":
                self._require_machine_door_open(goal_handle, station.name)
            if int(request.operation) == PICK_OPERATION:
                self._pick(goal_handle, station, target, request.part_id)
            else:
                self._place(goal_handle, station, target, request.part_id)

            goal_handle.succeed()
            return self._result(
                ManipulatePart.Result.OK,
                f"{request.part_id} completed at {request.station_id}",
                physical_part_id=self._logical_to_physical.get(
                    request.part_id, ""
                ),
            )
        except ExecutionFailure as error:
            if error.code == ManipulatePart.Result.CANCELLED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return self._result(error.code, str(error))
        except Exception as error:
            self.get_logger().error(f"Unexpected manipulation failure: {error}")
            goal_handle.abort()
            return self._result(
                ManipulatePart.Result.INVALID_REQUEST, str(error)
            )
        finally:
            self._execution_lock.release()

    def _pick(self, goal_handle, station, target, part_id: str) -> None:
        if self._held_part_id is not None:
            raise ExecutionFailure(
                ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
                f"Cannot pick {part_id}; gripper already holds {self._held_part_id}",
            )

        raw_pick = station.name == "raw_bin"
        expected_physical_part_id = self._logical_to_physical.get(part_id)
        if raw_pick and expected_physical_part_id is not None:
            raise ExecutionFailure(
                ManipulatePart.Result.INVALID_REQUEST,
                f"Logical workpiece {part_id} was already bound to "
                f"{expected_physical_part_id}",
            )
        if station.role == "machine" and expected_physical_part_id is None:
            raise ExecutionFailure(
                ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
                f"No physical identity is bound to {part_id}",
            )

        # Every station model is expressed in base_link at its observed dock.
        # Remove the previous station before perception/IK at the new dock so
        # collision screening cannot see world-fixed geometry from the prior
        # production phase. No robot motion occurs until the newly observed
        # station geometry has been applied below.
        self._feedback(
            goal_handle,
            "CLEAR_PREVIOUS_STATION_SCENE",
            "Removing the previous dock's collision geometry from MoveIt",
        )
        self._apply_planning_scene(
            goal_handle, self._planning_scene.clear_station_geometry()
        )

        self._feedback(
            goal_handle,
            "RESOLVE_GRASP_TARGET",
            f"Resolving a sensor-derived target for {part_id}",
        )
        part_center_pose = self._resolve_station_target(
            station, target.pose
        )
        if station.name == "raw_bin":
            self._feedback(
                goal_handle,
                "PERCEIVE_RAW_PART",
                f"Selecting a fresh RGB-D candidate for {part_id}",
            )
            part_center_pose = self._resolve_perceived_raw_target(
                goal_handle,
                part_center_pose,
                station.grasp_offset,
                station.approach_offset,
            )
        elif station.role == "machine":
            self._feedback(
                goal_handle,
                "PERCEIVE_MACHINE_PART",
                f"Locating {part_id} inside {station.name} from RGB-D",
            )
            part_center_pose = self._resolve_perceived_machine_target(
                part_center_pose
            )

        grasp_pose = part_center_pose.translated(station.grasp_offset)
        approach = grasp_pose.translated(station.approach_offset)
        self._feedback(
            goal_handle,
            "PREPARE_COLLISION_SCENE",
            f"Adding {station.name} and its observed workpieces to MoveIt",
        )
        self._prepare_pick_scene(
            goal_handle, station, target, part_id, part_center_pose
        )
        if not uses_direct_pick_transit(station.name, station.role):
            self._move_to_manipulation_ready(goal_handle)
        if station.clearance_pose is not None:
            self._move(
                goal_handle, station.clearance_pose, "MOVE_TO_CLEARANCE",
                planner="ptp",
            )

        self._move_to_approach(
            goal_handle,
            approach,
            station.name,
            part_to_open=part_id,
        )

        self._allow_target_contact(goal_handle, part_id)
        self._move(goal_handle, grasp_pose, "MOVE_TO_GRASP", planner="lin")
        contact_window = (
            self._evidence.begin_any_finger_contact_window()
            if raw_pick
            else self._evidence.begin_finger_contact_window(
                expected_physical_part_id
            )
        )
        self._feedback(goal_handle, "CLOSE_GRIPPER", f"Closing on {part_id}")
        self._feedback(
            goal_handle,
            "VERIFY_TWO_FINGER_CONTACT",
            "Requiring fresh contact at both fingertips",
        )
        physical_part_id = self._close_until_two_finger_contact(
            goal_handle,
            expected_physical_part_id=expected_physical_part_id,
            received_after=contact_window,
        )
        if raw_pick:
            existing_logical = next(
                (
                    logical
                    for logical, physical in self._logical_to_physical.items()
                    if physical == physical_part_id
                ),
                None,
            )
            if existing_logical is not None:
                raise ExecutionFailure(
                    ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
                    f"{physical_part_id} is already owned by {existing_logical}",
                )
            self._logical_to_physical[part_id] = physical_part_id
        self._confirm_settled_grasp(
            goal_handle, physical_part_id, received_after=contact_window
        )

        measured_position, _ = self._gripper.measured_state()
        if measured_position is None:
            raise ExecutionFailure(
                ManipulatePart.Result.GRIPPER_FAILED,
                "No measured gripper aperture after bilateral contact",
            )
        self._held_gripper_position = measured_position
        self._loaded_reseat_count = 0
        if station.role in {"source", "machine"}:
            fixture_name = (
                "source-tray locator" if station.role == "source" else "CNC vise"
            )
            self._feedback(
                goal_handle,
                "RELEASE_FIXTURE_CLAMP",
                f"Releasing the {fixture_name} after tactile ownership",
            )
            self._release_fixture_clamp(goal_handle, physical_part_id)

        calibrated_offset = abs(float(station.grasp_offset[2]))
        if calibrated_offset <= 0.0:
            raise ExecutionFailure(
                ManipulatePart.Result.INVALID_REQUEST,
                f"Station {station.name} has no calibrated TCP-to-part offset",
            )
        self._feedback(
            goal_handle,
            "VERIFY_LOAD_BEARING_GRASP",
            f"Proof-lifting {part_id} with contact physics only",
        )
        proof_lift = self._current_relative_lift_pose(PROOF_LIFT_DISTANCE)
        proof_lift_started = time.monotonic()
        self._move(goal_handle, proof_lift, "PROOF_LIFT", planner="proof_lin")
        self._require_grasp_hold(
            physical_part_id,
            received_after=proof_lift_started,
            allow_proof_lift_reseat=True,
        )
        self._held_part_id = part_id
        self._held_physical_part_id = physical_part_id
        self._held_part_vertical_offset = calibrated_offset

        self._apply_planning_scene(
            goal_handle,
            self._planning_scene.attach_carried_workpiece_geometry(
                part_id,
                part_center_tool_y=-calibrated_offset,
            ),
        )
        self._move_to_loaded_carry(
            goal_handle,
            part_id=physical_part_id,
            reference_distance=calibrated_offset,
            exit_machine=station.role == "machine",
        )

    def _place(self, goal_handle, station, target, part_id: str) -> None:
        if self._held_part_id != part_id:
            held = self._held_part_id or "nothing"
            raise ExecutionFailure(
                ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
                f"Cannot place {part_id}; gripper currently holds {held}",
            )
        physical_part_id = self._logical_to_physical.get(part_id)
        if (
            physical_part_id is None
            or self._held_physical_part_id != physical_part_id
        ):
            raise ExecutionFailure(
                ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
                f"Cannot place {part_id}; its physical grasp identity is lost",
            )
        if self._held_part_vertical_offset is None:
            raise ExecutionFailure(
                ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
                f"Cannot place {part_id}; its calibrated grasp offset is unknown",
            )

        station_target = self._resolve_station_target(station, target.pose)
        if station.role == "machine":
            self._feedback(
                goal_handle,
                "PREPARE_COLLISION_SCENE",
                f"Adding the open {station.name} aperture and fixture to MoveIt",
            )
            self._apply_planning_scene(
                goal_handle,
                self._planning_scene.prepare_machine_station(
                    station.name,
                    {},
                    station_offset=(
                        station_target.position[0] - target.pose.position[0],
                        station_target.position[1] - target.pose.position[1],
                        0.0,
                    ),
                ),
            )
        elif station.name == "finished_bin":
            self._feedback(
                goal_handle,
                "PREPARE_COLLISION_SCENE",
                "Adding the finished table and marker support to MoveIt",
            )
            self._apply_planning_scene(
                goal_handle, self._planning_scene.prepare_finished_station()
            )

        if station.role == "machine":
            self._feedback(
                goal_handle,
                "KEEP_LOADED_CARRY",
                "Keeping the collision-checked loaded pose for CNC entry",
            )
        elif not uses_direct_bin_place_transit(station.name):
            self._move_to_manipulation_ready(goal_handle)
        if station.clearance_pose is not None:
            self._move(
                goal_handle, station.clearance_pose, "MOVE_TO_CLEARANCE",
                planner="ptp",
            )

        place_pose = station_target.translated(
            (0.0, 0.0, self._held_part_vertical_offset)
        )
        approach = place_pose.translated(station.approach_offset)
        self._move_to_approach(goal_handle, approach, station.name)
        self._require_grasp_hold(physical_part_id)
        if station.role == "machine":
            self._allow_machine_fixture_contact(goal_handle, part_id)
        elif station.name == "finished_bin":
            self._allow_finished_bin_support_contact(goal_handle, part_id)

        support_window = self._evidence.begin_support_contact_window(
            physical_part_id, station.name
        )
        place_planner = (
            "fixture_loaded_lin" if station.role == "machine" else "loaded_lin"
        )
        self._move(
            goal_handle,
            place_pose,
            "MOVE_TO_PLACE",
            planner=place_planner,
            early_completion_condition=lambda: (
                self._evidence.recent_support_contact(
                    physical_part_id,
                    station.name,
                    received_after=support_window,
                )
            ),
            early_completion_validator=lambda: (
                self._require_guarded_support_stop(
                    physical_part_id,
                    station.name,
                    place_pose,
                )
            ),
        )
        self._verify_or_seek_support(
            goal_handle,
            physical_part_id,
            station.name,
            station.role,
            place_pose,
            received_after=support_window,
        )

        try:
            support_tcp, _orientation = self._evidence.current_tcp_in_base(
                timeout_sec=3.0
            )
        except RuntimeError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                f"Cannot measure supported {part_id} centre: {error}",
            ) from error
        released_part_center = supported_part_center_from_tcp(
            support_tcp, self._held_part_vertical_offset
        )
        self.get_logger().info(
            f"Measured supported centre for {part_id} at {station.name}: "
            f"({released_part_center[0]:.3f}, "
            f"{released_part_center[1]:.3f}, "
            f"{released_part_center[2]:.3f})"
        )

        if station.role == "machine":
            self._feedback(
                goal_handle,
                "SECURE_FIXTURE_CLAMP",
                f"Clamping physically supported {part_id} before release",
            )
            self._secure_fixture_clamp(goal_handle, physical_part_id)

        self._feedback(goal_handle, "OPEN_GRIPPER", f"Releasing {part_id}")
        # A supported cylinder may remain tangent to one fully opened pad.
        # Both the CNC fixture and finished table permit a collision-checked
        # vertical separation. Verify tactile silence only after that motion,
        # rather than declaring failure while the open fingers still straddle
        # the physically secured part.
        fingers_clear = self._open_to_safe_aperture(
            goal_handle,
            physical_part_id,
            allow_supported_separation=(
                station.role == "machine" or station.name == "finished_bin"
            ),
        )
        if not fingers_clear:
            self._separate_open_gripper_from_supported_part(
                goal_handle,
                physical_part_id,
                station_id=station.name,
                support_received_after=support_window,
            )
        self._held_part_id = None
        self._held_physical_part_id = None
        self._held_part_vertical_offset = None
        self._held_gripper_position = None
        self._loaded_reseat_count = 0

        if station.role == "machine":
            self._move_to_machine_egress(goal_handle, approach)
        elif station.name == "finished_bin":
            self._move_to_bin_egress(goal_handle, approach)
            # Start only after the gripper has released and cleared the slot;
            # neither a carried part nor a finger may satisfy placement proof.
            occupancy_window = self._finished_slots.begin_observation_window()
        else:
            self._move(goal_handle, approach, "RETREAT", planner="lin")

        if station.name == "finished_bin":
            self._feedback(
                goal_handle,
                "VERIFY_DESTINATION_OCCUPIED",
                "Confirming finished-bin placement from post-release RGB-D frames",
            )
            try:
                self._finished_slots.wait_for_occupied(
                    int(target.placement_slot_id),
                    requested_at=occupancy_window,
                    timeout_sec=self._finished_slot_confirmation_timeout,
                )
            except RuntimeError as error:
                raise ExecutionFailure(
                    ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED, str(error)
                ) from error

        self._apply_planning_scene(
            goal_handle,
            self._planning_scene.place_released_workpiece_geometry(
                part_id,
                frame_id=station_target.frame_id,
                position=released_part_center,
            ),
        )
        if station.role == "machine":
            self._open_gripper_after_machine_exit(goal_handle, part_id)
        else:
            self._move_to_transport(goal_handle, "RETURN_TO_TRANSPORT")

    def _allow_target_contact(self, goal_handle, part_id: str) -> None:
        """Remove only the selected target from MoveIt's world geometry."""
        self._feedback(
            goal_handle,
            "ALLOW_TARGET_CONTACT",
            f"Allowing the gripper to contact only {part_id}",
        )
        self._apply_planning_scene(
            goal_handle, self._planning_scene.remove_world_object(part_id)
        )

    def _allow_finished_bin_support_contact(
        self, goal_handle, part_id: str
    ) -> None:
        """Allow only the held part to make its intended table contact."""
        self._feedback(
            goal_handle,
            "ALLOW_DESTINATION_CONTACT",
            f"Allowing only {part_id} to contact the finished table",
        )
        response = self._wait_future(
            goal_handle,
            self._planning_scene.get_allowed_collision_matrix(),
            timeout_sec=5.0,
            timeout_code=ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
            timeout_message="Reading MoveIt's collision matrix timed out",
        )
        try:
            update = self._planning_scene.allow_finished_bin_support_contact(
                part_id,
                response.scene.allowed_collision_matrix,
            )
        except ValueError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                f"MoveIt returned an invalid collision matrix: {error}",
            ) from error
        self._apply_planning_scene(goal_handle, update)

    def _allow_machine_fixture_contact(self, goal_handle, part_id: str) -> None:
        """Append one process allowance to MoveIt's existing collision matrix."""
        self._feedback(
            goal_handle,
            "ALLOW_FIXTURE_CONTACT",
            f"Allowing only {part_id} to contact the CNC vise",
        )
        response = self._wait_future(
            goal_handle,
            self._planning_scene.get_allowed_collision_matrix(),
            timeout_sec=5.0,
            timeout_code=ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
            timeout_message="Reading MoveIt's collision matrix timed out",
        )
        try:
            update = self._planning_scene.allow_machine_fixture_contact(
                part_id,
                response.scene.allowed_collision_matrix,
            )
        except ValueError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                f"MoveIt returned an invalid collision matrix: {error}",
            ) from error
        self._apply_planning_scene(goal_handle, update)

    def _resolve_station_target(
        self, station, configured_pose: CartesianPose
    ) -> CartesianPose:
        """Use a perceived CNC frame for final manipulation coordinates."""
        reference = station.fixture_reference
        if reference is None:
            return configured_pose

        try:
            position = self._evidence.point_in_base(
                reference.frame_id,
                Point3(*reference.position),
                timeout_sec=3.0,
            )
        except RuntimeError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                f"Cannot resolve {station.name} fixture from "
                f"{reference.frame_id}: {error}",
            ) from error

        try:
            target_position = resolve_machine_target_position(
                position,
                configured_pose.position,
            )
        except ValueError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.PERCEPTION_FAILED,
                f"Unsafe {station.name} loading datum: {error}",
            ) from error

        target = CartesianPose(
            frame_id="base_link",
            position=target_position,
            orientation=configured_pose.orientation,
        )
        self.get_logger().info(
            f"Resolved {station.name} fixture from {reference.frame_id}: "
            f"observed=({position.x:.3f}, {position.y:.3f}, "
            f"{position.z:.3f}), target=({target.position[0]:.3f}, "
            f"{target.position[1]:.3f}, {target.position[2]:.3f})"
        )
        return target

    def _prepare_pick_scene(
        self,
        goal_handle,
        station,
        target,
        part_id: str,
        measured_center: CartesianPose,
    ) -> None:
        """Apply collision geometry before entering a source or CNC fixture."""
        if station.role == "machine":
            # A second visual dock can stop a few centimetres from the first
            # one while still satisfying Nav2's local controller. The clamped
            # part is a fixed fixture landmark, so use its horizontal
            # displacement to express the whole CNC collision model in the
            # robot's current base_link frame. Vertical part settlement is
            # deliberately excluded: the machine remains fixed to the floor.
            nominal = target.pose.position
            measured = measured_center.position
            station_offset = (
                measured[0] - nominal[0],
                measured[1] - nominal[1],
                0.0,
            )
            self._apply_planning_scene(
                goal_handle,
                self._planning_scene.prepare_machine_station(
                    station.name,
                    {part_id: measured_center.position},
                    station_offset=station_offset,
                ),
            )
            return

        if station.name != "raw_bin":
            return

        # RGB-D owns every source-part position. The selected candidate uses
        # the order's logical ID so only that collision object is removed for
        # the final grasp; all remaining candidates stay in the scene.
        workpieces = {part_id: measured_center.position}
        obstacle_index = 0
        for candidate in self._raw_scene_candidates:
            if math.dist(candidate, measured_center.position) <= 0.03:
                continue
            obstacle_index += 1
            workpieces[f"raw_obstacle_{obstacle_index}"] = candidate

        self._apply_planning_scene(
            goal_handle,
            self._planning_scene.prepare_source_station(
                station.name, workpieces
            ),
        )

    def _resolve_perceived_raw_target(
        self,
        goal_handle,
        configured_target: CartesianPose,
        grasp_offset: tuple[float, float, float],
        approach_offset: tuple[float, float, float],
    ) -> CartesianPose:
        """Choose the first fresh candidate with collision-free approach IK."""
        try:
            selection = self._raw_part_perception.wait_for_selection(
                configured_target.position,
                maximum_horizontal_distance=self._raw_part_candidate_distance,
                timeout_sec=self._raw_part_perception_timeout,
            )
        except RuntimeError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE, str(error)
            ) from error
        self._raw_scene_candidates = tuple(
            (candidate.x, candidate.y, candidate.z)
            for candidate in selection.candidates
        )

        rejected: list[str] = []
        # Prefer the observation that leaves the shortest horizontal arm
        # reach after applying the calibrated side-approach offset.  This is
        # a general manipulability policy; collision-aware IK still decides
        # whether a candidate is admissible.
        ordered_candidates = sorted(
            selection.stable_candidates,
            key=lambda item: math.hypot(
                item.x + grasp_offset[0] + approach_offset[0],
                item.y + grasp_offset[1] + approach_offset[1],
            ),
        )
        for perceived in ordered_candidates:
            target = CartesianPose(
                frame_id="base_link",
                position=(perceived.x, perceived.y, perceived.z),
                orientation=configured_target.orientation,
            )
            candidate_accepted = True
            for pose_name, screening_pose in candidate_screening_poses(
                target, grasp_offset, approach_offset
            ):
                try:
                    future = self._ik_reachability.solve_async(
                        screening_pose, BIN_STATION_TRANSIT
                    )
                except RuntimeError as error:
                    raise ExecutionFailure(
                        ManipulatePart.Result.DEPENDENCY_UNAVAILABLE, str(error)
                    ) from error
                response = self._wait_future(
                    goal_handle,
                    future,
                    timeout_sec=2.0,
                    timeout_code=(
                        ManipulatePart.Result.DEPENDENCY_UNAVAILABLE
                    ),
                    timeout_message=(
                        "MoveIt collision-aware IK screening timed out"
                    ),
                )
                code = int(response.error_code.val)
                if code != MoveItErrorCodes.SUCCESS:
                    rejected.append(
                        f"({perceived.x:.3f},{perceived.y:.3f},"
                        f"{perceived.z:.3f}):{pose_name}={code}"
                    )
                    candidate_accepted = False
                    break
            if candidate_accepted:
                self.get_logger().info(
                    "Raw RGB-D target selected at "
                    f"({perceived.x:.3f}, {perceived.y:.3f}, "
                    f"{perceived.z:.3f}) after approach and grasp IK screening"
                )
                return target

        raise ExecutionFailure(
            ManipulatePart.Result.MOTION_FAILED,
            "no fresh raw-part candidate has collision-free approach and grasp IK; "
            f"tested={len(ordered_candidates)}, "
            f"MoveIt codes={rejected}",
        )

    def _resolve_perceived_machine_target(
        self, configured_target: CartesianPose
    ) -> CartesianPose:
        """Use a fresh fixture-volume RGB-D three-dimensional centre.

        Contact-guided insertion may settle at different support depths.
        Therefore unloading must measure x, y and z; the taught pose is only
        a bounded work-volume sanity reference, never a height fallback.
        """

        try:
            perceived = self._machine_part_perception.wait_for_nearest(
                configured_target.position,
                maximum_horizontal_distance=(
                    self._machine_part_candidate_distance
                ),
                timeout_sec=self._machine_part_perception_timeout,
            )
        except RuntimeError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE, str(error)
            ) from error
        try:
            measured_center = resolve_observed_machine_part_center(
                perceived,
                configured_target.position,
                maximum_vertical_distance=(
                    self._machine_part_vertical_distance
                ),
            )
        except ValueError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.PERCEPTION_FAILED, str(error)
            ) from error
        target = CartesianPose(
            frame_id="base_link",
            position=measured_center,
            orientation=configured_target.orientation,
        )
        self.get_logger().info(
            "Machine RGB-D target selected at "
            f"({perceived.x:.3f}, {perceived.y:.3f}, {perceived.z:.3f}); "
            f"nominal_z={configured_target.position[2]:.3f}"
        )
        return target

    def _close_until_two_finger_contact(
        self,
        goal_handle,
        *,
        expected_physical_part_id: str | None,
        received_after: float,
    ) -> str:
        """Close normally and return the one tactile-identified workpiece."""

        last_description = "contact=none"

        def observe(timeout_sec: float) -> tuple[str | None, bool, bool, str]:
            if expected_physical_part_id is None:
                selection = self._evidence.check_any_two_finger_contact(
                    timeout_sec=timeout_sec,
                    received_after=received_after,
                )
                return (
                    selection.physical_part_id,
                    selection.accepted,
                    selection.ambiguous,
                    selection.describe(),
                )
            check = self._evidence.check_two_finger_contact(
                expected_physical_part_id,
                timeout_sec=timeout_sec,
                received_after=received_after,
            )
            return (
                expected_physical_part_id,
                check.accepted,
                False,
                check.describe(),
            )

        try:
            # A raw-bin pick has no preselected simulator identity. The pads
            # discover it from bilateral contact; a CNC unload instead checks
            # the physical identity previously bound to the logical part.
            deadline = time.monotonic() + GRIPPER_CONTACT_SEARCH_TIMEOUT
            target = self._gripper.CLOSED_POSITION
            self._start_gripper_close(goal_handle, target)
            while time.monotonic() < deadline:
                self._check_cancel(goal_handle)
                physical_part_id, accepted, ambiguous, last_description = (
                    observe(0.05)
                )
                if ambiguous:
                    raise ExecutionFailure(
                        ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
                        "Bilateral contact matched more than one physical "
                        "workpiece; refusing an ambiguous grasp",
                    )
                finger_pair = self._gripper.measured_positions()
                if (
                    not accepted
                    and finger_pair is not None
                    and min(finger_pair)
                    >= self._gripper.CLOSED_POSITION - 0.001
                ):
                    raise ExecutionFailure(
                        ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
                        "The jaws reached their physical closure limit "
                        "without fresh bilateral workpiece contact; "
                        f"fingers=({finger_pair[0]:.3f},"
                        f"{finger_pair[1]:.3f}), {last_description}",
                    )
                if accepted and physical_part_id is not None:
                    position, _ = self._gripper.measured_state()
                    if position is None:
                        raise ExecutionFailure(
                            ManipulatePart.Result.GRIPPER_FAILED,
                            "No gripper position available at contact",
                        )
                    hold_target = self._start_gripper_hold(
                        goal_handle, measured_position=position
                    )
                    self.get_logger().info(
                        f"Two-finger contact for {physical_part_id}: "
                        f"{last_description}, knuckle={position:.3f}, "
                        f"search_target={target:.3f}, "
                        f"contact_positions=({hold_target[0]:.3f},"
                        f"{hold_target[1]:.3f}) with bounded force hold"
                    )
                    return physical_part_id
        except Exception:
            self._stop_gripper_effort()
            raise

        self._stop_gripper_effort()
        position, velocity = self._gripper.measured_state()
        state = (
            "unknown"
            if position is None
            else f"position={position:.3f}, velocity={velocity or 0.0:.4f}"
        )
        pad_state = "pad_tf=unavailable"
        try:
            left, right = self._evidence.kinematic_pad_positions_in_base(
                timeout_sec=1.0
            )
            pad_state = (
                f"pad_tf=left({left.x:.3f},{left.y:.3f},{left.z:.3f}),"
                f"right({right.x:.3f},{right.y:.3f},{right.z:.3f})"
            )
        except RuntimeError:
            pass
        raise ExecutionFailure(
            ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
            "Two-finger contact missing; "
            f"{last_description}, knuckle={state}, {pad_state}",
        )

    def _stop_gripper_effort(self) -> None:
        """Remove only the jaw effort owned by this manipulation request."""
        if self._gripper_effort_active:
            self._gripper.stop()
            self._gripper_effort_active = False

    def _confirm_settled_grasp(
        self, goal_handle, part_id: str, *, received_after: float
    ) -> None:
        """Require bilateral contact after the arm and pads have settled."""
        # Let the measured-width hold goal settle before accepting contact.
        self._wait_grasp_settle(goal_handle)
        check = self._evidence.check_two_finger_contact(
            part_id,
            timeout_sec=0.4,
            received_after=received_after,
        )
        if not check.accepted:
            raise ExecutionFailure(
                ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
                f"Bilateral contact did not persist for {part_id}; "
                f"{check.describe()}",
            )
        position, _ = self._gripper.measured_state()
        measured = "unknown" if position is None else f"{position:.3f}"
        pad_text = "pad_tf=unavailable"
        try:
            left, right = self._evidence.kinematic_pad_positions_in_base(
                timeout_sec=1.0
            )
            pad_text = (
                f"pad_tf=left({left.x:.3f},{left.y:.3f},{left.z:.3f}),"
                f"right({right.x:.3f},{right.y:.3f},{right.z:.3f})"
            )
        except RuntimeError:
            pass
        self.get_logger().info(
            f"Settled two-finger grasp for {part_id}: "
            f"{check.describe()}, knuckle={measured}, {pad_text}, "
            f"{self._evidence.recent_contact_summary(part_id)}"
        )

    @staticmethod
    def _wait_grasp_settle(goal_handle) -> None:
        """Let the loaded fingertip contacts settle without blocking cancel."""
        settle_deadline = time.monotonic() + GRASP_FORCE_SETTLE_TIME
        while time.monotonic() < settle_deadline:
            ManipulatePartServer._check_cancel(goal_handle)
            time.sleep(0.02)

    def _current_relative_lift_pose(self, distance: float) -> CartesianPose:
        """Lift from the latest measured TCP pose by one positive distance."""
        if distance <= 0.0:
            raise ValueError("relative lift distance must be positive")
        try:
            position, orientation = self._evidence.current_tcp_in_base(
                timeout_sec=3.0
            )
        except RuntimeError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE, str(error)
            ) from error
        return CartesianPose(
            frame_id="base_link",
            position=(position.x, position.y, position.z + distance),
            orientation=(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )

    def _require_grasp_hold(
        self,
        part_id: str,
        *,
        received_after: float = 0.0,
        allow_proof_lift_reseat: bool = False,
    ) -> None:
        """Require three fresh tactile/joint samples from a stable hold."""

        deadline = time.monotonic() + GRASP_HOLD_CONFIRMATION_TIMEOUT
        consecutive = 0
        previous_sample_time: float | None = None
        check = self._evidence.recent_two_finger_contact(
            part_id, received_after=received_after
        )
        position: float | None = None
        velocity: float | None = None
        reference = self._held_gripper_position

        while time.monotonic() < deadline:
            check = self._evidence.recent_two_finger_contact(
                part_id, received_after=received_after
            )
            position, velocity, sample_time = self._gripper.measured_sample()
            if sample_time is None or sample_time == previous_sample_time:
                time.sleep(0.02)
                continue
            previous_sample_time = sample_time
            preserved_aperture = (
                grasp_hold_is_valid(
                    check,
                    position,
                    reference,
                    minimum_total_closure=(
                        self._gripper.MIN_VERIFIED_TOTAL_CLOSURE
                    ),
                    maximum_position_change=(
                        MAX_GRASP_SETTLING_POSITION_CHANGE
                    ),
                )
            )
            proof_reseat = (
                allow_proof_lift_reseat
                and proof_lift_reseat_is_valid(
                    check,
                    position,
                    minimum_total_closure=(
                        self._gripper.MIN_VERIFIED_TOTAL_CLOSURE
                    ),
                    maximum_total_closure=(
                        self._gripper.MAX_VERIFIED_TOTAL_CLOSURE
                    ),
                )
            )
            loaded_reseat = (
                not preserved_aperture
                and self._held_physical_part_id == part_id
                and self._loaded_reseat_count < MAX_LOADED_RESEATS
                and loaded_hold_reseat_is_valid(
                    check,
                    position,
                    reference,
                    minimum_total_closure=(
                        self._gripper.MIN_VERIFIED_TOTAL_CLOSURE
                    ),
                    maximum_total_closure=(
                        self._gripper.MAX_VERIFIED_TOTAL_CLOSURE
                    ),
                    maximum_reseat_change=MAX_LOADED_RESEAT_POSITION_CHANGE,
                )
            )
            stable = (
                (preserved_aperture or proof_reseat or loaded_reseat)
                and velocity is not None
                and abs(velocity) <= MAX_GRASP_SETTLING_VELOCITY
            )
            consecutive = consecutive + 1 if stable else 0
            if consecutive >= GRASP_HOLD_CONFIRMATION_SAMPLES:
                # Rebase only after tactile identity, aperture and velocity
                # have all remained valid for consecutive fresh samples.
                # Future checks therefore detect new slip rather than
                # accumulating legitimate compliant settling forever.
                if loaded_reseat and not preserved_aperture:
                    self._loaded_reseat_count += 1
                mode = "preserved"
                if proof_reseat:
                    mode = "proof_reseat"
                elif loaded_reseat:
                    mode = "loaded_reseat"
                self._held_gripper_position = position
                self.get_logger().info(
                    f"Grasp evidence for {part_id}: {check.describe()}, "
                    f"total_closure={position:.3f}, "
                    f"velocity={velocity or 0.0:.4f}, "
                    f"mode={mode}, "
                    f"loaded_reseats={self._loaded_reseat_count}/"
                    f"{MAX_LOADED_RESEATS}, "
                    f"samples={consecutive}"
                )
                return
            time.sleep(0.02)

        measured = "unknown" if position is None else f"{position:.3f}"
        expected = "unknown" if reference is None else f"{reference:.3f}"
        finger_pair = self._gripper.measured_positions()
        pair_text = (
            "unavailable"
            if finger_pair is None
            else f"left={finger_pair[0]:.3f},right={finger_pair[1]:.3f}"
        )
        raise ExecutionFailure(
            ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
            f"Load-bearing grasp evidence lost for {part_id}; "
            f"{check.describe()}, total_closure={measured}, "
            f"reference={expected}, fingers=({pair_text}), "
            f"consecutive_samples={consecutive}",
        )

    def _verify_or_seek_support(
        self,
        goal_handle,
        part_id: str,
        station_id: str,
        station_role: str,
        nominal_pose: CartesianPose,
        *,
        received_after: float,
    ) -> None:
        """Verify support, or perform a bounded contact-guided descent.

        The first pose comes from surveyed fixture geometry.  Small downward
        motions compensate only for physical grasp compliance and simulation
        contact settling.  They never turn missing contact into success: every
        accepted placement still needs a fresh collision-sensor observation.
        """

        if self._support_contact_observed(
            part_id, station_id, received_after=received_after
        ):
            self.get_logger().info(
                f"Support contact confirmed for {part_id} at {station_id} "
                "at the nominal pose"
            )
            return

        if station_role == "machine":
            seating_depths = machine_seating_depths()
            seating_planner = "fixture_loaded_lin"
            maximum_depth = MAX_MACHINE_SEATING_DEPTH
        elif station_id == "finished_bin":
            seating_depths = finished_bin_seating_depths()
            seating_planner = "loaded_lin"
            maximum_depth = MAX_FINISHED_BIN_SEATING_DEPTH
        else:
            self._raise_missing_support(part_id, station_id)
            return

        for step_index, depth in enumerate(seating_depths, start=1):
            self._require_grasp_hold(part_id)
            self._feedback(
                goal_handle,
                "SEEK_FIXTURE_SUPPORT",
                f"Contact-guided seating step {step_index}: "
                f"{depth * 1000.0:.0f} mm below nominal",
                attempt=step_index,
            )
            seating_pose = nominal_pose.translated((0.0, 0.0, -depth))
            self._move(
                goal_handle,
                seating_pose,
                "SEEK_FIXTURE_SUPPORT",
                planner=seating_planner,
                early_completion_condition=lambda: (
                    self._evidence.recent_support_contact(
                        part_id,
                        station_id,
                        received_after=received_after,
                    )
                ),
                early_completion_validator=lambda: (
                    self._require_guarded_support_stop(
                        part_id,
                        station_id,
                        seating_pose,
                    )
                ),
            )
            if self._support_contact_observed(
                part_id, station_id, received_after=received_after
            ):
                self._require_grasp_hold(part_id)
                self.get_logger().info(
                    f"Support contact confirmed for {part_id} at "
                    f"{station_id} after {depth * 1000.0:.0f} mm "
                    "guarded descent"
                )
                return

        self._raise_missing_support(
            part_id,
            station_id,
            detail=(
                f" after {maximum_depth * 1000.0:.0f} mm "
                "bounded guarded descent"
            ),
        )

    def _support_contact_observed(
        self,
        part_id: str,
        station_id: str,
        *,
        received_after: float,
    ) -> bool:
        """Return only fresh contact evidence from the requested support."""

        return self._evidence.wait_for_support_contact(
            part_id,
            station_id,
            received_after=received_after,
            timeout_sec=SUPPORT_CONTACT_SAMPLE_TIMEOUT,
        )

    def _require_guarded_support_stop(
        self,
        part_id: str,
        station_id: str,
        target: CartesianPose,
    ) -> None:
        """Validate the stopped pose after contact-triggered cancellation.

        Fresh support was already observed by the early-completion condition.
        Cancelling MoveIt can outlive the contact freshness window, so checking
        the same transient event again creates a race.  Here we prove that the
        arm stopped inside the placement envelope with a load-bearing grasp.
        The caller then requires current support before opening the gripper.
        """
        self._require_grasp_hold(part_id)
        try:
            position, orientation = self._evidence.current_tcp_in_base(
                timeout_sec=3.0
            )
        except RuntimeError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                str(error),
                safe_to_retry=False,
            ) from error

        horizontal_error = math.hypot(
            position.x - target.position[0],
            position.y - target.position[1],
        )
        vertical_error = position.z - target.position[2]

        orientation_error = tcp_orientation_error(
            orientation, target.orientation
        )
        if (
            not guarded_support_pose_is_safe(position, target.position)
            or orientation_error > MAX_TCP_ORIENTATION_ERROR
        ):
            raise ExecutionFailure(
                ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
                f"Guarded support stop at {station_id} is outside the "
                "placement envelope: "
                f"horizontal_error={horizontal_error:.4f} m, "
                f"vertical_error={vertical_error:+.4f} m, "
                f"orientation_error={orientation_error:.4f} rad, "
                f"measured=({position.x:.4f}, {position.y:.4f}, "
                f"{position.z:.4f}), target=({target.position[0]:.4f}, "
                f"{target.position[1]:.4f}, {target.position[2]:.4f}); "
                f"{self._evidence.recent_contact_summary(part_id)}",
                safe_to_retry=False,
            )
        self.get_logger().info(
            f"Guarded placement stopped safely after support contact for "
            f"{part_id} at {station_id}; measured TCP="
            f"({position.x:.3f}, {position.y:.3f}, {position.z:.3f})"
        )

    def _raise_missing_support(
        self, part_id: str, station_id: str, *, detail: str = ""
    ) -> None:
        raise ExecutionFailure(
            ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
            f"No fresh support contact for {part_id} at {station_id}{detail}; "
            f"{self._evidence.recent_contact_summary(part_id)}",
        )

    def _require_machine_door_open(
        self, goal_handle, machine_id: str
    ) -> None:
        """Block all arm motion until measured door feedback is safely open."""
        self._feedback(
            goal_handle,
            "VERIFY_MACHINE_DOOR_OPEN",
            f"Waiting for the physical {machine_id} door-open limit",
        )
        deadline = time.monotonic() + 12.0
        latest = None
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            latest = self._machine_doors.latest(machine_id)
            if latest is not None and self._machine_doors.is_open(latest):
                self.get_logger().info(
                    f"{machine_id} physical door is open at "
                    f"{latest.position:.3f} m"
                )
                return
            time.sleep(0.05)

        if latest is None:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                f"No physical door feedback received from {machine_id}",
            )
        raise ExecutionFailure(
            ManipulatePart.Result.MOTION_FAILED,
            f"{machine_id} door did not reach its open limit; "
            f"measured {latest.position:.3f} m",
        )

    def _require_dependencies(self) -> None:
        if not self._pose_motion.wait_until_ready(timeout_sec=5.0):
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                "MoveIt /move_action is unavailable",
            )
        if not self._joint_motion.wait_until_ready(timeout_sec=5.0):
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                "MoveIt joint motion interface is unavailable",
            )
        if not self._gripper.wait_until_ready(timeout_sec=5.0):
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                "Gripper effort controller is unavailable",
            )

        if not self._planning_scene.wait_until_ready(timeout_sec=5.0):
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                "MoveIt planning scene service is unavailable",
            )
        if not self._ik_reachability.wait_until_ready(timeout_sec=5.0):
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                "MoveIt collision-aware IK service or joint state is unavailable",
            )

    def _apply_planning_scene(self, goal_handle, future) -> None:
        response = self._wait_future(
            goal_handle,
            future,
            timeout_sec=5.0,
            timeout_code=ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
            timeout_message="MoveIt planning scene update timed out",
        )
        if not response.success:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE,
                "MoveIt rejected the workpiece planning scene update",
            )

    def _move(
        self,
        goal_handle,
        pose: CartesianPose,
        phase: str,
        *,
        planner: str = "ompl",
        progress_callback: Callable[[], None] | None = None,
        early_completion_condition: Callable[[], bool] | None = None,
        early_completion_validator: Callable[[], None] | None = None,
    ) -> None:
        if (early_completion_condition is None) != (
            early_completion_validator is None
        ):
            raise ValueError(
                "early motion completion requires a condition and validator"
            )
        target = PoseTarget(
            name=phase.lower(),
            frame_id=pose.frame_id,
            position=pose.position,
            orientation=pose.orientation,
        )
        self.get_logger().info(
            f"{phase} target in {pose.frame_id}: "
            f"position=({pose.position[0]:.3f}, "
            f"{pose.position[1]:.3f}, {pose.position[2]:.3f}), "
            f"planner={planner}"
        )

        # PTP needs one KDL IK solve from the current seed; when that fails
        # (NO_IK_SOLUTION) OMPL's multi-seeded sampling usually still finds a
        # plan, so PTP transit moves fall back to OMPL instead of aborting.
        attempts = planner_attempts(planner)
        for index, (attempt, attempt_planner) in enumerate(attempts):
            self._feedback(
                goal_handle,
                phase,
                f"Target frame: {pose.frame_id} (planner: {attempt_planner})",
                attempt=attempt,
            )
            try:
                reached_target = self._execute_motion(
                    goal_handle,
                    self._pose_motion.send_pose_target(
                        target,
                        planner=attempt_planner,
                        position_tolerance=(
                            None if attempt_planner == "lin" else 0.020
                        ),
                    ),
                    phase,
                    progress_callback=progress_callback,
                    early_completion_condition=early_completion_condition,
                )
                if reached_target:
                    self._require_tcp_pose(pose, phase)
                else:
                    early_completion_validator()
                return
            except ExecutionFailure as error:
                # MoveIt can report TIMED_OUT while the trajectory controller
                # reaches its goal during cancellation. The action status is
                # then stale, but the physical goal contract is still
                # observable. Accept only a fresh TCP measurement inside the
                # same arrival envelope used after an ordinary SUCCESS; a
                # merely planned or partially executed motion still fails.
                if (
                    error.code == ManipulatePart.Result.MOTION_FAILED
                    and pose.frame_id == "base_link"
                ):
                    try:
                        self._require_tcp_pose(pose, phase)
                    except ExecutionFailure:
                        pass
                    else:
                        self.get_logger().warning(
                            f"{phase} action failed, but independent TCP "
                            "measurement proves physical arrival"
                        )
                        return
                if (
                    error.code != ManipulatePart.Result.MOTION_FAILED
                    or index == len(attempts) - 1
                    or not error.safe_to_retry
                ):
                    raise
                next_planner = attempts[index + 1][1]
                self.get_logger().warning(
                    f"{phase} with {attempt_planner} failed once; retrying "
                    f"with {next_planner} from the latest robot state"
                )
                self._wait_for_arm_replan_settle(goal_handle)

    def _require_tcp_pose(self, target: CartesianPose, phase: str) -> None:
        """Independently verify Cartesian arrival after controller success."""
        if target.frame_id != "base_link":
            self.get_logger().warning(
                f"Skipping TCP arrival check for unsupported frame "
                f"{target.frame_id}"
            )
            return
        try:
            position, orientation = self._evidence.current_tcp_in_base(
                timeout_sec=3.0
            )
        except RuntimeError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE, str(error)
            ) from error

        position_error = tcp_position_error(position, target.position)
        orientation_error = tcp_orientation_error(
            orientation, target.orientation
        )
        self.get_logger().info(
            f"{phase} measured TCP error: "
            f"position={position_error:.4f} m, "
            f"orientation={orientation_error:.4f} rad"
        )
        if (
            position_error > MAX_TCP_POSITION_ERROR
            or orientation_error > MAX_TCP_ORIENTATION_ERROR
        ):
            raise ExecutionFailure(
                ManipulatePart.Result.MOTION_FAILED,
                f"{phase} TCP did not settle within the physical arrival "
                f"limits (position={position_error:.4f} m, "
                f"orientation={orientation_error:.4f} rad)",
            )

    def _remember_arm_state(self, message: JointState) -> None:
        by_name = dict(zip(message.name, message.position))
        if all(name in by_name for name in ARM_JOINTS):
            self._arm_positions = tuple(by_name[name] for name in ARM_JOINTS)
            self._arm_state_sample = (
                time.monotonic(), self._arm_positions
            )

    def _wait_for_arm_replan_settle(self, goal_handle) -> None:
        """Wait for measured joints to stop before sampling a retry start."""
        deadline = time.monotonic() + ARM_REPLAN_SETTLE_TIMEOUT
        previous: tuple[float, ...] | None = None
        previous_time = -1.0
        stable_samples = 0
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            sample = self._arm_state_sample
            if sample is None or sample[0] == previous_time:
                time.sleep(0.01)
                continue
            previous_time, positions = sample
            if previous is not None and arm_joint_sample_is_stable(
                previous, positions
            ):
                stable_samples += 1
                if stable_samples >= ARM_REPLAN_SETTLE_SAMPLES:
                    self.get_logger().info(
                        "Measured arm joints settled before MoveIt retry"
                    )
                    return
            else:
                stable_samples = 0
            previous = positions
            time.sleep(0.01)
        raise ExecutionFailure(
            ManipulatePart.Result.MOTION_FAILED,
            "Measured arm joints did not settle before MoveIt retry",
            safe_to_retry=False,
        )

    def _log_arm_waypoint(self, name: str) -> None:
        if self._arm_positions is None:
            self.get_logger().warning(f"No joint state available at {name}")
            return
        values = ", ".join(f"{value:.6f}" for value in self._arm_positions)
        self.get_logger().info(f"{name} joints=({values})")

    def _move_to_approach(
        self,
        goal_handle,
        approach: CartesianPose,
        station_name: str,
        *,
        part_to_open: str | None = None,
    ) -> None:
        """Reach a station, optionally opening above a CNC before descent."""
        if station_name in {"raw_bin", "finished_bin"}:
            station_label = station_name.upper()
            phase = f"MOVE_TO_{station_label}_TRANSIT"
            self._feedback(
                goal_handle,
                phase,
                f"Using the collision-checked {station_name} IK branch",
            )
            self._move_joint_target(
                goal_handle, BIN_STATION_TRANSIT, phase
            )
            self._log_arm_waypoint(f"{station_name} transit")
            x, y, z = approach.position
            upper_approach = CartesianPose(
                frame_id=approach.frame_id,
                position=(x, y, max(z, RAW_BIN_PREGRASP_HEIGHT)),
                orientation=approach.orientation,
            )
            # Select the wrist branch in open space, then preserve it during
            # the low fixture descent. The final station insertion is a second
            # LIN segment after independent Gazebo anchor calibration.
            self._move(
                goal_handle,
                upper_approach,
                "MOVE_TO_UPPER_APPROACH",
                planner="ptp",
            )
            if part_to_open is not None:
                self._open_gripper_for_pick(
                    goal_handle, part_to_open, confined=False
                )
            self._move(
                goal_handle,
                approach,
                "MOVE_TO_APPROACH",
                planner="bin_approach_lin",
            )
            self._log_arm_waypoint(f"{station_name} approach")
            return

        transit_height = (
            MACHINE_APPROACH_TRANSIT_HEIGHT
            if station_name.startswith("machine_")
            else APPROACH_TRANSIT_HEIGHT
        )
        x, y, z = approach.position
        transit = CartesianPose(
            frame_id=approach.frame_id,
            position=(x, y, max(z, transit_height)),
            orientation=approach.orientation,
        )
        machine_transit = station_name.startswith("machine_")
        # The loaded arm enters through the same waypoints as the independently
        # verified CNC egress corridor. A single OMPL goal all the way into
        # the machine can select a long alternate IK branch and exceed the
        # execution deadline even though both endpoints are valid.
        if machine_transit and self._held_part_id is not None:
            self._move_to_machine_transit(goal_handle, transit)
        else:
            self._move(
                goal_handle,
                transit,
                "MOVE_TO_APPROACH_TRANSIT",
                planner="ptp" if machine_transit else "ompl",
            )
        if part_to_open is not None:
            self._open_gripper_for_pick(
                goal_handle, part_to_open, confined=True
            )
        if machine_transit:
            # Transit and approach share x/y and orientation. A vertical LIN
            # descent preserves the validated IK branch for both loading and
            # unloading. A held workpiece only changes the speed limits.
            approach_planner = (
                "fixture_loaded_lin"
                if self._held_part_id is not None
                else "fixture_empty_lin"
            )
        else:
            approach_planner = (
                "loaded_ptp" if self._held_part_id is not None else "ptp"
            )
        self._move(
            goal_handle,
            approach,
            "MOVE_TO_APPROACH",
            planner=approach_planner,
        )

    def _move_to_machine_transit(
        self, goal_handle, transit: CartesianPose
    ) -> None:
        """Select the CNC IK branch outside, then reverse its proven egress."""
        door_clear, fold_clearance, _travel = (
            machine_clearance_waypoints(transit)
        )
        # The raw-bin carry and CNC corridor use the same TCP orientation but
        # may occupy different joint-space branches. Select the nearest valid
        # branch with deterministic PTP at each open-space waypoint. A Pilz
        # LIN request across this horizontal span can cross a wrist/elbow
        # singularity and generate a discontinuous IK solution even though
        # both endpoint poses are valid. Cartesian LIN is reserved for the
        # final vertical fixture approach below.
        self._move(
            goal_handle,
            fold_clearance,
            "MOVE_TO_MACHINE_FOLD_CLEARANCE",
            planner="loaded_ptp",
        )
        self._move(
            goal_handle,
            door_clear,
            "MOVE_TO_MACHINE_DOOR_CLEARANCE",
            planner="loaded_ptp",
        )
        self._move(
            goal_handle,
            transit,
            "MOVE_TO_APPROACH_TRANSIT",
            planner="loaded_ptp",
        )

    def _move_to_bin_egress(
        self, goal_handle, approach: CartesianPose
    ) -> None:
        """Rise above a placed bin part before folding the empty arm."""
        x, y, z = approach.position
        upper_approach = CartesianPose(
            frame_id=approach.frame_id,
            position=(x, y, max(z, RAW_BIN_PREGRASP_HEIGHT)),
            orientation=approach.orientation,
        )
        if upper_approach.position[2] <= z + 0.001:
            return
        self._move(
            goal_handle,
            upper_approach,
            "MOVE_TO_UPPER_RETREAT",
            planner="lin",
        )

    def _move_to_machine_egress(
        self, goal_handle, approach: CartesianPose
    ) -> None:
        """Reverse CNC entry, then park clear of the sliding-door pocket."""
        x, y, z = approach.position
        transit = CartesianPose(
            frame_id=approach.frame_id,
            position=(x, y, max(z, MACHINE_APPROACH_TRANSIT_HEIGHT)),
            orientation=approach.orientation,
        )
        # This is an empty-gripper move, but it starts at the CNC reach
        # boundary. The general empty LIN acceleration caused Pilz to exceed
        # the elbow deceleration limit on the longer merged retreat. Use the
        # dedicated fixture envelope while retaining a single continuous rise.
        self._move(
            goal_handle,
            transit,
            "MOVE_TO_MACHINE_EGRESS",
            planner="fixture_empty_lin",
        )
        self._move_from_machine_transit(
            goal_handle,
            transit,
            planner="lin",
        )

    def _move_from_machine_transit(
        self,
        goal_handle,
        transit: CartesianPose,
        *,
        planner: str,
        progress_callback: Callable[[], None] | None = None,
    ) -> None:
        """Cross the CNC door plane, then fold through reachable waypoints."""
        door_clear, fold_clearance, travel = (
            machine_clearance_waypoints(transit)
        )
        free_space_planner = machine_egress_free_space_planner(planner)
        door_planner = (
            "loaded_egress_lin"
            if planner == "loaded_transport_lin"
            else planner
        )
        self._move(
            goal_handle,
            door_clear,
            "MOVE_CLEAR_OF_MACHINE_DOOR",
            planner=door_planner,
            progress_callback=progress_callback,
        )
        self._move(
            goal_handle,
            fold_clearance,
            "MOVE_TO_MACHINE_FOLD_CLEARANCE",
            planner=free_space_planner,
            progress_callback=progress_callback,
        )
        self._move(
            goal_handle,
            travel,
            "MOVE_TO_MACHINE_TRAVEL",
            planner=free_space_planner,
            progress_callback=progress_callback,
        )

    def _move_to_manipulation_ready(self, goal_handle) -> None:
        """Unfold through a known-safe joint posture before station motion."""
        phase = "MOVE_TO_MANIPULATION_READY"
        self._feedback(
            goal_handle,
            phase,
            "Unfolding arm above the mobile-base collision envelope",
        )
        self._move_joint_target(
            goal_handle, MANIPULATION_READY, phase
        )

    def _move_to_loaded_carry(
        self,
        goal_handle,
        *,
        part_id: str,
        reference_distance: float,
        exit_machine: bool = False,
    ) -> None:
        """Raise and retract a held part along a station-safe path."""
        try:
            position, orientation = self._evidence.current_tcp_in_base(
                timeout_sec=3.0
            )
        except RuntimeError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.DEPENDENCY_UNAVAILABLE, str(error)
            ) from error
        orientation_tuple = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        clearance = CartesianPose(
            frame_id="base_link",
            position=(
                position.x,
                position.y,
                max(
                    position.z,
                    MACHINE_APPROACH_TRANSIT_HEIGHT
                    if exit_machine
                    else LOADED_CARRY_HEIGHT,
                ),
            ),
            orientation=orientation_tuple,
        )
        carry = CartesianPose(
            frame_id="base_link",
            position=(
                LOADED_CARRY_X,
                LOADED_CARRY_Y,
                clearance.position[2],
            ),
            orientation=orientation_tuple,
        )
        # The proof lift established bilateral tactile ownership. Every
        # subsequent segment rechecks both contacts and the measured aperture.
        self._require_grasp_hold(part_id)
        self.get_logger().info(
            f"Keeping tactile-verified hold target for {part_id}"
        )
        if clearance.position[2] > position.z + 0.01:
            self._move(
                goal_handle,
                clearance,
                "MOVE_TO_LOADED_CLEARANCE",
                planner="loaded_transport_lin",
                progress_callback=lambda: self._log_loaded_grasp_sample(
                    part_id, reference_distance
                ),
            )
            self._require_grasp_hold(part_id)

        grasp_monitor = lambda: self._log_loaded_grasp_sample(
            part_id, reference_distance
        )
        if exit_machine:
            self._move_from_machine_transit(
                goal_handle,
                clearance,
                planner="loaded_transport_lin",
                progress_callback=grasp_monitor,
            )
        else:
            self._move(
                goal_handle,
                carry,
                "MOVE_TO_CARRY",
                planner="loaded_transport_lin",
                progress_callback=grasp_monitor,
            )
        self._require_grasp_hold(part_id)
        self._log_arm_waypoint("loaded carry")
        self.get_logger().info(
            f"Physical grasp preserved at carry pose for {part_id}"
        )

    def _open_to_safe_aperture(
        self,
        goal_handle,
        part_id: str,
        *,
        allow_supported_separation: bool = False,
    ) -> bool:
        """Open enough to clear the known cylindrical workpiece.

        This also handles a supported part that remains in light fingertip
        contact; finger clearance plus fixture or RGB-D evidence proves release.
        """
        # Remove the grasping preload, then apply bounded opening effort until
        # the measured V-jaw clearance is safe inside the CNC fixture.
        self._stop_gripper_effort()
        time.sleep(0.10)
        self._evidence.begin_finger_contact_window(part_id)
        self._gripper.release_contact()
        self._gripper_effort_active = True
        deadline = time.monotonic() + 15.0
        safe_since: float | None = None
        open_since: float | None = None
        last_contact = self._evidence.recent_two_finger_contact(part_id)
        last_position: float | None = None
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            now = time.monotonic()
            position, _ = self._gripper.measured_state()
            contact = self._evidence.recent_two_finger_contact(part_id)
            last_contact = contact
            last_position = position
            opened_enough = (
                self._gripper.both_fingers_at_or_below(
                    self._gripper.RELEASE_OPEN_POSITION
                )
            )
            if opened_enough:
                if open_since is None:
                    open_since = now
            else:
                open_since = None
            # Joint width alone cannot prove release.  Retreating while even
            # one pad still touches a fixture-clamped part side-loads the
            # parallel rails and can jam the next approach.
            fingers_clear = not contact.left and not contact.right
            if opened_enough and fingers_clear:
                if safe_since is None:
                    safe_since = now
                elif now - safe_since >= 0.25:
                    self._stop_gripper_effort()
                    self.get_logger().info(
                        f"Verified safe gripper aperture for {part_id}: "
                        f"knuckle={position:.3f}, {contact.describe()}"
                    )
                    return True
            else:
                safe_since = None
            if (
                allow_supported_separation
                and opened_enough
                and open_since is not None
                and now - open_since >= 0.75
            ):
                self._stop_gripper_effort()
                self.get_logger().info(
                    f"Full release aperture reached for {part_id} with "
                    f"residual contact ({contact.describe()}); performing "
                    "a slow supported vertical separation"
                )
                return False
            time.sleep(0.02)
        self._stop_gripper_effort()
        position_text = "unknown" if last_position is None else f"{last_position:.3f}"
        raise ExecutionFailure(
            ManipulatePart.Result.GRIPPER_FAILED,
            f"Safe gripper opening missing for {part_id}; knuckle={position_text}, "
            f"last contact sample: {last_contact.describe()}",
        )

    def _separate_open_gripper_from_supported_part(
        self,
        goal_handle,
        part_id: str,
        *,
        station_id: str,
        support_received_after: float,
    ) -> None:
        """Clear the whole upright part and prove support plus tactile silence."""
        if not self._gripper.both_fingers_at_or_below(
            self._gripper.RELEASE_OPEN_POSITION
        ):
            raise ExecutionFailure(
                ManipulatePart.Result.GRIPPER_FAILED,
                f"Cannot separate from {part_id}; full release aperture is lost",
            )
        if self._held_part_vertical_offset is None:
            raise ExecutionFailure(
                ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
                f"Cannot derive supported release geometry for {part_id}",
            )
        separation_distance = supported_release_separation_distance(
            self._held_part_vertical_offset
        )
        self._feedback(
            goal_handle,
            "SEPARATE_FROM_SUPPORTED_PART",
            f"Lifting open fingers {separation_distance:.3f} m",
        )
        separation = self._current_relative_lift_pose(
            separation_distance
        )
        self._move(
            goal_handle,
            separation,
            "SEPARATE_FROM_SUPPORTED_PART",
            planner="proof_lin",
        )

        deadline = time.monotonic() + SUPPORTED_RELEASE_CLEAR_TIMEOUT
        clear_since: float | None = None
        last_contact = self._evidence.recent_two_finger_contact(part_id)
        support_present = False
        fixture_clamped: bool | None = None
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            now = time.monotonic()
            contact = self._evidence.recent_two_finger_contact(part_id)
            last_contact = contact
            support_present = self._evidence.recent_support_contact(
                part_id,
                station_id,
                received_after=support_received_after,
            )
            if station_id.startswith("machine_"):
                fixture_clamped = self._fixture_clamp.is_clamped(part_id)
            retained = released_part_retention_is_valid(
                station_id,
                support_contact=support_present,
                fixture_clamped=fixture_clamped,
            )
            if not contact.left and not contact.right and retained:
                if clear_since is None:
                    clear_since = now
                elif now - clear_since >= 0.25:
                    self.get_logger().info(
                        f"Physical release separation verified for {part_id}: "
                        f"{contact.describe()}, support={support_present}, "
                        f"fixture_clamped={fixture_clamped}"
                    )
                    return
            else:
                clear_since = None
            time.sleep(0.02)
        contact_summary = self._evidence.recent_contact_summary(part_id)
        raise ExecutionFailure(
            ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
            f"Open-finger separation did not clear {part_id}; "
            f"last contact sample: {last_contact.describe()}, "
            f"support={support_present}, fixture_clamped={fixture_clamped}; "
            f"{contact_summary}",
        )

    def _open_gripper_after_machine_exit(
        self, goal_handle, part_id: str
    ) -> None:
        """Fully open in free space after a confined CNC release.

        The first opening only has to clear the clamped workpiece.  Requiring
        the normal free-space aperture after egress prevents that partial
        opening from becoming the starting state of the next unload.
        """
        self._feedback(
            goal_handle,
            "OPEN_GRIPPER_IN_FREE_SPACE",
            f"Fully opening after leaving the {part_id} fixture",
        )
        self._run_gripper_open(
            goal_handle,
            command_name="post-egress open",
            acceptable_open_position=self._gripper.MAX_SAFE_PICK_POSITION,
        )

    def _log_loaded_grasp_sample(
        self, part_id: str, _calibrated_offset: float
    ) -> None:
        """Log tactile and joint evidence during loaded motion."""

        now = time.monotonic()
        if now - self._last_grasp_monitor_log < 5.0:
            return
        self._last_grasp_monitor_log = now
        position, velocity = self._gripper.measured_state()
        joint_text = (
            "total_closure=unavailable"
            if position is None
            else f"total_closure={position:.3f}, velocity={velocity or 0.0:.4f}"
        )
        contact = self._evidence.recent_two_finger_contact(part_id)
        self.get_logger().info(
            f"Loaded grasp sample for {part_id}: {joint_text}, "
            f"contact=({contact.describe()})"
        )

    def _move_to_transport(self, goal_handle, phase: str) -> None:
        """Fold the arm before the mobile base is allowed to navigate."""
        self._feedback(goal_handle, phase, "Moving arm to stowed transport state")
        self._move_joint_target(goal_handle, STOWED, phase)

    def _move_joint_target(
        self,
        goal_handle,
        target: JointTarget,
        phase: str,
    ) -> None:
        """Execute a taught joint posture with one fresh-state retry.

        Mobile-base motion and a loaded gripper can leave Gazebo's joints a
        few samples behind MoveIt's planning snapshot. A bounded retry lets
        the servo settle and replans from the latest measured state; it does
        not bypass collision checking or controller result validation.
        """
        for attempt in (1, 2):
            self._log_arm_waypoint(f"before {phase} attempt {attempt}")
            try:
                self._execute_motion(
                    goal_handle,
                    self._joint_motion.send_joint_target(target),
                    phase,
                )
                return
            except ExecutionFailure as error:
                if (
                    error.code != ManipulatePart.Result.MOTION_FAILED
                    or attempt == 2
                    or not error.safe_to_retry
                ):
                    raise
                self.get_logger().warning(
                    f"{phase} failed from a stale joint-state snapshot; "
                    "waiting for the arm to settle before one retry"
                )
                self._wait_for_arm_replan_settle(goal_handle)

    def _execute_motion(
        self,
        goal_handle,
        goal_future,
        phase: str,
        *,
        progress_callback: Callable[[], None] | None = None,
        early_completion_condition: Callable[[], bool] | None = None,
    ) -> bool:
        """Apply one timeout and result policy to every MoveIt request."""
        move_handle = self._wait_future(
            goal_handle,
            goal_future,
            timeout_sec=15.0,
            timeout_code=ManipulatePart.Result.MOTION_FAILED,
            timeout_message=f"MoveIt did not accept {phase}",
        )
        if move_handle is None or not move_handle.accepted:
            raise ExecutionFailure(
                ManipulatePart.Result.MOTION_FAILED,
                f"MoveIt rejected {phase}",
            )
        wrapped_result = self._wait_future(
            goal_handle,
            move_handle.get_result_async(),
            timeout_sec=self._motion_execution_timeout,
            timeout_code=ManipulatePart.Result.MOTION_FAILED,
            timeout_message=f"MoveIt timed out during {phase}",
            cancel_callback=move_handle.cancel_goal_async,
            progress_callback=progress_callback,
            early_completion_condition=early_completion_condition,
        )
        if wrapped_result is _EARLY_MOTION_COMPLETION:
            return False
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or not moveit_succeeded(wrapped_result.result.error_code)
        ):
            code = wrapped_result.result.error_code.val
            raise ExecutionFailure(
                ManipulatePart.Result.MOTION_FAILED,
                f"MoveIt failed during {phase} with code {code}",
            )
        return True

    def _open_gripper_for_pick(
        self, goal_handle, part_id: str, *, confined: bool
    ) -> None:
        """Create a measured-clear aperture before approaching one part."""
        acceptable_opening = (
            self._gripper.MAX_CONFINED_PICK_POSITION
            if confined
            else self._gripper.MAX_SAFE_PICK_POSITION
        )
        self._feedback(
            goal_handle, "OPEN_GRIPPER", f"Opening at pre-grasp for {part_id}"
        )
        self._run_gripper_open(
            goal_handle,
            command_name="open",
            acceptable_open_position=acceptable_opening,
        )

    def _start_gripper_close(
        self,
        goal_handle,
        target: float,
    ) -> None:
        """Start force-limited closure; bilateral contact proves grasp."""
        self._check_cancel(goal_handle)
        self._gripper.close_to(target)
        self._gripper_effort_active = True

    def _start_gripper_hold(
        self,
        goal_handle,
        *,
        measured_position: float,
        stabilized: bool = False,
    ) -> tuple[float, float]:
        """Replace search effort with the bounded carrying effort."""
        self._stop_gripper_effort()
        hold = (
            self._gripper.hold_stabilized_carry
            if stabilized
            else self._gripper.hold_at_contact
        )
        try:
            hold_positions = hold(measured_position)
        except RuntimeError as error:
            raise ExecutionFailure(
                ManipulatePart.Result.GRIPPER_FAILED, str(error)
            ) from error
        self._gripper_effort_active = True
        return hold_positions

    def _run_gripper_open(
        self,
        goal_handle,
        *,
        command_name: str,
        acceptable_open_position: float,
    ) -> None:
        """Apply opening force until both measured rails are safely clear."""
        self._gripper.open()
        self._gripper_effort_active = True
        deadline = time.monotonic() + 10.0
        stable_since: float | None = None
        last_position: float | None = None
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            position, velocity = self._gripper.measured_state()
            last_position = position
            safely_open = (
                self._gripper.both_fingers_at_or_below(
                    acceptable_open_position
                )
            )
            settled = velocity is None or velocity <= 0.01
            if safely_open and settled:
                now = time.monotonic()
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= 0.10:
                    self._stop_gripper_effort()
                    self.get_logger().info(
                        f"Gripper {command_name} verified at "
                        f"position={position:.3f}, velocity={velocity or 0.0:.4f}"
                    )
                    return
            else:
                stable_since = None
            time.sleep(0.02)
        self._stop_gripper_effort()
        measured = "unknown" if last_position is None else f"{last_position:.3f}"
        raise ExecutionFailure(
            ManipulatePart.Result.GRIPPER_FAILED,
            f"Gripper did not reach safe aperture during {command_name}; "
            f"position={measured}",
        )

    def _secure_fixture_clamp(self, goal_handle, part_id: str) -> None:
        """Hold a verified placement before the open fingers retreat."""
        deadline = time.monotonic() + 5.0
        next_request = 0.0
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            if self._fixture_clamp.is_clamped(part_id) is True:
                self.get_logger().info(
                    f"Fixture secured {part_id} for physical processing"
                )
                return
            if time.monotonic() >= next_request:
                self._fixture_clamp.request_clamp(part_id)
                next_request = time.monotonic() + 0.25
            time.sleep(0.05)
        raise ExecutionFailure(
            ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
            f"Fixture did not clamp {part_id}",
        )

    def _release_fixture_clamp(self, goal_handle, part_id: str) -> None:
        """Release the vise only after the gripper owns a stable grasp."""
        deadline = time.monotonic() + 5.0
        next_request = 0.0
        while time.monotonic() < deadline:
            self._check_cancel(goal_handle)
            if self._fixture_clamp.is_clamped(part_id) is False:
                self.get_logger().info(
                    f"Fixture released {part_id} for physical unloading"
                )
                return
            if time.monotonic() >= next_request:
                self._fixture_clamp.request_release(part_id)
                next_request = time.monotonic() + 0.25
            time.sleep(0.05)
        raise ExecutionFailure(
            ManipulatePart.Result.PHYSICAL_EVIDENCE_FAILED,
            f"Fixture did not release {part_id}",
        )

    def _wait_future(
        self,
        goal_handle,
        future,
        *,
        timeout_sec: float,
        timeout_code: int,
        timeout_message: str,
        cancel_callback=None,
        progress_callback: Callable[[], None] | None = None,
        early_completion_condition: Callable[[], bool] | None = None,
    ):
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_and_wait(future, cancel_callback)
                raise ExecutionFailure(
                    ManipulatePart.Result.CANCELLED,
                    "Manipulation cancelled at a safe phase boundary",
                )
            if (
                early_completion_condition is not None
                and early_completion_condition()
            ):
                settled = self._cancel_and_wait(future, cancel_callback)
                if not settled:
                    raise ExecutionFailure(
                        ManipulatePart.Result.MOTION_FAILED,
                        "Guarded motion did not acknowledge contact stop",
                        safe_to_retry=False,
                    )
                return _EARLY_MOTION_COMPLETION
            if time.monotonic() >= deadline:
                settled = self._cancel_and_wait(future, cancel_callback)
                detail = timeout_message
                if cancel_callback is not None and not settled:
                    detail += "; the child action did not acknowledge cancellation"
                raise ExecutionFailure(
                    timeout_code,
                    detail,
                    safe_to_retry=settled,
                )
            if progress_callback is not None:
                progress_callback()
            time.sleep(0.05)
        return future.result()

    def _cancel_and_wait(
        self,
        result_future,
        cancel_callback,
        *,
        grace_period: float = 10.0,
    ) -> bool:
        """Cancel a child action and prove it stopped before any retry."""
        if result_future.done():
            return True
        if cancel_callback is None:
            return False
        try:
            cancel_callback()
        except Exception as error:
            self.get_logger().error(f"Could not request child cancellation: {error}")
            return False

        deadline = time.monotonic() + grace_period
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return result_future.done()

    @staticmethod
    def _check_cancel(goal_handle) -> None:
        if goal_handle.is_cancel_requested:
            raise ExecutionFailure(
                ManipulatePart.Result.CANCELLED,
                "Manipulation cancelled at a safe phase boundary",
            )

    @staticmethod
    def _feedback(
        goal_handle, phase: str, detail: str, *, attempt: int = 1
    ) -> None:
        feedback = ManipulatePart.Feedback()
        feedback.phase = phase
        feedback.attempt = attempt
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(
        error_code: int,
        message: str,
        *,
        physical_part_id: str = "",
    ) -> ManipulatePart.Result:
        result = ManipulatePart.Result()
        result.success = error_code == ManipulatePart.Result.OK
        result.error_code = error_code
        result.message = message
        result.physical_part_id = physical_part_id
        return result

    def destroy_node(self) -> None:
        self._server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManipulatePartServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
