import math

import pytest

from factory_core.dock_station import (
    BIN_LATERAL_TOLERANCE,
    DockStationClient,
    FINISHED_BIN_TAG_TARGET,
    MACHINE_LATERAL_TOLERANCE,
    MACHINE_POSITION_TOLERANCE,
    MACHINE_TAG_TARGET,
    RAW_BIN_TAG_TARGET,
)
from factory_core.visual_station_alignment import (
    VISUAL_APPROACH_MAX_LINEAR_SPEED,
    VISUAL_APPROACH_PROPORTIONAL_GAIN,
    VISUAL_SIDE_STEP_LINEAR_SPEED,
    AlignmentError,
    BasePoseError,
    ObservationFreshnessClock,
    OdometryPose,
    PoseAlignmentController,
    VisualStationAligner,
    TagAlignmentTarget,
    alignment_error,
    base_pose_error,
    observation_may_drive,
    planar_heading_from_tag_normal,
    select_odometry_correction,
    tag_position_in_odometry,
    tag_reacquisition_heading,
    velocity_for_alignment,
    wrap_angle,
)


TARGET = MACHINE_TAG_TARGET


def test_visual_commands_expire_before_the_tag_loss_timeout():
    assert observation_may_drive(0.0)
    assert observation_may_drive(0.30)
    assert not observation_may_drive(0.31)
    assert not observation_may_drive(-0.01)


def test_machine_tag_contract_matches_collision_bounded_work_pose():
    assert TARGET.x == pytest.approx(0.564)
    assert MACHINE_POSITION_TOLERANCE == pytest.approx(0.010)
    assert MACHINE_LATERAL_TOLERANCE == pytest.approx(0.010)


def test_machine_alignment_closes_nav2_lateral_and_heading_residuals():
    class RecordingAligner:
        def align(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs
            return AlignmentError(0.0, 0.0, 0.0)

    client = DockStationClient.__new__(DockStationClient)
    client._visual_aligner = RecordingAligner()

    client.align_machine_tag("machine_1_tag", timeout_sec=12.0)

    assert client._visual_aligner.args == ("machine_1_tag", TARGET)
    assert client._visual_aligner.kwargs["longitudinal_only"] is False
    assert client._visual_aligner.kwargs["control_lateral"] is True
    assert client._visual_aligner.kwargs["use_odometry_corrections"] is True

    client.align_bin_pose("raw_bin_tag", timeout_sec=12.0)

    assert client._visual_aligner.args == (
        "raw_bin_tag",
        RAW_BIN_TAG_TARGET,
    )
    assert client._visual_aligner.kwargs["lateral_tolerance"] == pytest.approx(
        BIN_LATERAL_TOLERANCE
    )
    assert client._visual_aligner.kwargs["control_lateral"] is True
    assert client._visual_aligner.kwargs["use_odometry_corrections"] is True

    client.align_bin_pose("finished_bin_tag", timeout_sec=12.0)

    assert client._visual_aligner.args == (
        "finished_bin_tag",
        FINISHED_BIN_TAG_TARGET,
    )
    assert client._visual_aligner.kwargs["lateral_tolerance"] == pytest.approx(
        BIN_LATERAL_TOLERANCE
    )
    assert client._visual_aligner.kwargs["control_lateral"] is True
    assert client._visual_aligner.kwargs["use_odometry_corrections"] is True
    assert client._visual_aligner.kwargs["heading_from_tag_normal"] is True


def test_alignment_error_is_zero_at_calibrated_tag_pose():
    error = alignment_error(TARGET.x, TARGET.y, TARGET.yaw, TARGET)

    assert error == AlignmentError(0.0, 0.0, 0.0)
    assert error.within(position=0.015, heading=0.035)


def test_machine_acceptance_matches_factory_docking_contract():
    measured = AlignmentError(0.0, -0.009, 0.024)

    assert measured.within(
        position=MACHINE_POSITION_TOLERANCE, heading=0.035
    )


def test_alignment_error_wraps_heading_across_pi_boundary():
    error = alignment_error(
        TARGET.x,
        TARGET.y,
        TARGET.yaw + 2.0 * math.pi - 0.02,
        TARGET,
    )

    assert error.heading == pytest.approx(-0.02)


def test_reacquisition_aims_at_last_fresh_tag_without_reusing_its_error():
    observed_from = OdometryPose(x=1.0, y=2.0, yaw=math.pi / 2.0)
    tag_position = tag_position_in_odometry(observed_from, 2.0, 0.0)
    assert tag_position == pytest.approx((1.0, 4.0))

    current = OdometryPose(x=1.0, y=2.5, yaw=0.0)
    target = TagAlignmentTarget(x=1.0, y=0.0, yaw=0.0)
    heading = tag_reacquisition_heading(current, tag_position, target)
    assert heading == pytest.approx(math.pi / 2.0)

    searched = tag_reacquisition_heading(
        current,
        tag_position,
        target,
        search_offset=math.radians(12.0),
    )
    assert searched == pytest.approx(math.pi / 2.0 + math.radians(12.0))


def test_controller_moves_forward_when_tag_is_too_far():
    linear, angular = velocity_for_alignment(
        AlignmentError(longitudinal=0.10, lateral=0.0, heading=0.0)
    )

    assert linear > 0.0
    assert angular == 0.0


def test_controller_turns_toward_left_tag_and_limits_commands():
    linear, angular = velocity_for_alignment(
        AlignmentError(longitudinal=1.0, lateral=1.0, heading=1.0)
    )

    assert linear == pytest.approx(0.12)
    assert angular == pytest.approx(0.35)


@pytest.mark.parametrize(
    ("angle", "expected"),
    [
        (0.0, 0.0),
        (2.0 * math.pi + 0.25, 0.25),
        (-2.0 * math.pi - 0.25, -0.25),
    ],
)
def test_wrap_angle(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected)

def test_base_pose_error_is_zero_at_calibrated_tag_pose():
    pose = base_pose_error(AlignmentError(0.0, 0.0, 0.0), TARGET)

    assert pose.longitudinal == pytest.approx(0.0)
    assert pose.lateral == pytest.approx(0.0)
    assert pose.heading == pytest.approx(0.0)


def test_base_pose_error_supports_a_tighter_lateral_limit():
    pose = BasePoseError(longitudinal=0.04, lateral=0.04, heading=0.01)

    assert pose.within(position=0.05, heading=0.035)
    assert not pose.within(
        position=0.05,
        lateral_position=0.025,
        heading=0.035,
    )

def test_machine_alignment_contract_leaves_fixture_refinement_margin():
    assert MACHINE_POSITION_TOLERANCE == pytest.approx(0.010)
    assert MACHINE_LATERAL_TOLERANCE == pytest.approx(0.010)
    assert math.hypot(MACHINE_POSITION_TOLERANCE, MACHINE_LATERAL_TOLERANCE) < 0.015



def test_pose_controller_escapes_lateral_heading_false_equilibrium():
    # This is the measured residual that previously made the proportional
    # lateral and heading terms cancel and left the robot parked 30 mm off.
    error = AlignmentError(longitudinal=0.0, lateral=0.030, heading=-0.041)
    controller = PoseAlignmentController(MACHINE_LATERAL_TOLERANCE)

    linear, angular = controller.command(error, TARGET)

    assert linear == 0.0
    assert angular < 0.0


def test_odometry_heading_command_tapers_without_stalling():
    bounded = VisualStationAligner._bounded_command

    assert bounded(2.0 * 0.30, minimum=0.18, maximum=0.50) == 0.50
    assert bounded(2.0 * 0.08, minimum=0.18, maximum=0.50) == 0.18
    assert bounded(2.0 * -0.08, minimum=0.18, maximum=0.50) == -0.18
    assert VisualStationAligner._ODOM_HEADING_TOLERANCE == 0.025


def test_bounded_side_step_matches_measured_station_clearance():
    assert VisualStationAligner._MAX_SIDE_STEP_DISTANCE == pytest.approx(0.60)
    assert VisualStationAligner._SIDE_STEP_ANGLE == pytest.approx(math.pi / 4.0)


def test_large_heading_error_is_corrected_before_lateral_side_step():
    correction = select_odometry_correction(
        BasePoseError(longitudinal=-0.884, lateral=-0.146, heading=0.379),
        lateral_tolerance=BIN_LATERAL_TOLERANCE,
        heading_tolerance=0.035,
        heading_trigger_margin=0.020,
    )

    assert correction == "heading"


def test_lateral_side_step_follows_once_heading_is_in_coarse_tolerance():
    correction = select_odometry_correction(
        BasePoseError(longitudinal=-0.884, lateral=-0.146, heading=0.040),
        lateral_tolerance=BIN_LATERAL_TOLERANCE,
        heading_tolerance=0.035,
        heading_trigger_margin=0.020,
    )

    assert correction == "lateral"


def test_no_odometry_correction_inside_coarse_pose_tolerances():
    correction = select_odometry_correction(
        BasePoseError(
            longitudinal=-0.20,
            lateral=BIN_LATERAL_TOLERANCE - 0.001,
            heading=0.035,
        ),
        lateral_tolerance=BIN_LATERAL_TOLERANCE,
        heading_tolerance=0.035,
        heading_trigger_margin=0.020,
    )

    assert correction is None


def test_completed_heading_correction_is_not_repeated():
    correction = select_odometry_correction(
        BasePoseError(longitudinal=0.20, lateral=-0.25, heading=0.08),
        lateral_tolerance=MACHINE_LATERAL_TOLERANCE,
        heading_tolerance=0.035,
        heading_trigger_margin=0.020,
        heading_available=False,
    )

    # The remaining heading error is closed under live visual control. A
    # blind lateral manoeuvre is unsafe while the base is still misaligned.
    assert correction is None


def test_unspent_lateral_correction_survives_heading_reacquisition():
    correction = select_odometry_correction(
        BasePoseError(longitudinal=0.20, lateral=-0.25, heading=0.04),
        lateral_tolerance=MACHINE_LATERAL_TOLERANCE,
        heading_tolerance=0.035,
        heading_trigger_margin=0.020,
        heading_available=False,
        lateral_available=True,
    )

    assert correction == "lateral"


def test_observation_freshness_uses_simulation_progress_not_wall_time():
    class Parameter:
        value = True

    class TimePoint:
        nanoseconds = 1_250_000_000

    class Clock:
        def now(self):
            return TimePoint()

    class Node:
        def get_parameter(self, _name):
            return Parameter()

        def get_clock(self):
            return Clock()

    assert ObservationFreshnessClock(Node()).now() == pytest.approx(1.25)


def _tag_error_from_base_pose(
    longitudinal: float, lateral: float, heading: float
) -> AlignmentError:
    """Build a tag observation from a base pose in the goal frame."""
    cosine = math.cos(-heading)
    sine = math.sin(-heading)
    base_goal_x = -(cosine * longitudinal - sine * lateral)
    base_goal_y = -(sine * longitudinal + cosine * lateral)
    measured_x = (
        base_goal_x + cosine * TARGET.x - sine * TARGET.y
    )
    measured_y = (
        base_goal_y + sine * TARGET.x + cosine * TARGET.y
    )
    return alignment_error(
        measured_x, measured_y, TARGET.yaw - heading, TARGET
    )


def test_pose_controller_can_leave_cross_track_error_to_nav2():
    controller = PoseAlignmentController(
        MACHINE_LATERAL_TOLERANCE,
        control_lateral=False,
    )

    turn_command = controller.command(
        _tag_error_from_base_pose(-0.30, 0.08, 0.10),
        TARGET,
    )
    assert controller.phase == "straighten"
    assert turn_command[0] == 0.0
    assert turn_command[1] < 0.0

    approach_command = controller.command(
        _tag_error_from_base_pose(-0.30, 0.08, 0.0),
        TARGET,
    )
    assert controller.phase == "approach"
    assert approach_command[0] == pytest.approx(
        VISUAL_APPROACH_MAX_LINEAR_SPEED
    )


def test_pose_controller_closes_near_boundary_without_relaxing_tolerance():
    controller = PoseAlignmentController(
        MACHINE_LATERAL_TOLERANCE,
        control_lateral=False,
    )
    residual = MACHINE_POSITION_TOLERANCE + 0.001

    linear, angular = controller.command(
        _tag_error_from_base_pose(-residual, 0.0, 0.0),
        TARGET,
    )

    assert controller.phase == "approach"
    assert linear == pytest.approx(
        VISUAL_APPROACH_PROPORTIONAL_GAIN * residual
    )
    assert linear > 0.02
    assert angular == 0.0


def test_pose_controller_uses_observed_arc_before_close_side_step():
    controller = PoseAlignmentController(MACHINE_LATERAL_TOLERANCE)

    linear, angular = controller.command(
        _tag_error_from_base_pose(
            -0.15,
            -(MACHINE_LATERAL_TOLERANCE + 0.006),
            0.0,
        ),
        TARGET,
    )

    assert controller.phase == "approach"
    assert linear > 0.0
    assert angular != 0.0
    assert abs(angular) <= 0.20
    assert controller._SIDE_STEP_LONGITUDINAL_TRIGGER == pytest.approx(0.05)


def test_pose_controller_keeps_side_step_phases_stable():
    controller = PoseAlignmentController(MACHINE_LATERAL_TOLERANCE)

    turn_command = controller.command(
        _tag_error_from_base_pose(
            0.0, -(MACHINE_LATERAL_TOLERANCE + 0.009), 0.0
        ),
        TARGET,
    )
    assert controller.phase == "turn_for_side_step"
    assert turn_command[0] == 0.0
    assert turn_command[1] < 0.0

    side_command = controller.command(
        _tag_error_from_base_pose(
            0.0,
            -(MACHINE_LATERAL_TOLERANCE + 0.009),
            -controller._SIDE_STEP_HEADING,
        ), TARGET
    )
    assert controller.phase == "side_step"
    assert side_command[0] == pytest.approx(-VISUAL_SIDE_STEP_LINEAR_SPEED)
    assert math.degrees(controller._SIDE_STEP_HEADING) == pytest.approx(5.0)

    straighten_command = controller.command(
        _tag_error_from_base_pose(
            -0.30, -0.005, -controller._SIDE_STEP_HEADING
        ), TARGET
    )
    assert controller.phase == "straighten_after_side_step"
    assert straighten_command[0] == 0.0
    assert straighten_command[1] > 0.0

    approach_command = controller.command(
        _tag_error_from_base_pose(-0.30, -0.005, 0.0), TARGET
    )
    assert controller.phase == "approach"
    assert approach_command[0] == pytest.approx(
        VISUAL_APPROACH_MAX_LINEAR_SPEED
    )


def test_vertical_tag_normal_has_stable_planar_heading():
    half_sqrt = math.sqrt(0.5)

    heading = planar_heading_from_tag_normal(0.0, -half_sqrt, 0.0, half_sqrt)

    assert heading == pytest.approx(0.0)


def test_observed_vertical_tag_normal_recovers_small_heading_error():
    heading = planar_heading_from_tag_normal(-0.017, -0.708, -0.017, 0.706)

    assert heading == pytest.approx(0.0481, abs=0.002)


def test_acceptance_uses_base_pose_when_tag_offset_couples_yaw_and_lateral():
    bin_target = TagAlignmentTarget(x=0.620, y=-0.550, yaw=0.0)
    # Captured from a full-stack run: the raw Tag y residual is outside 2 cm,
    # while the actual base pose in the station-goal frame is inside 2 cm.
    tag_error = AlignmentError(-0.004, -0.024, -0.010)

    pose_error = base_pose_error(tag_error, bin_target)

    assert not tag_error.within(position=0.020, heading=0.035)
    assert pose_error.within(position=0.020, heading=0.035)
    assert pose_error.lateral == pytest.approx(0.0178, abs=0.001)
