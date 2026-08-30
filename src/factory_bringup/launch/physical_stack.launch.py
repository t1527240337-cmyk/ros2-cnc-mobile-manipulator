from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _include(package: str, launch_file: str, *, condition=None, launch_arguments=()):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        condition=condition,
        launch_arguments=launch_arguments,
    )


def generate_launch_description():
    use_navigation = LaunchConfiguration("use_navigation")
    world_file = LaunchConfiguration("world_file")
    use_moveit = LaunchConfiguration("use_moveit")
    enable_perception = LaunchConfiguration("enable_perception")
    enable_sparse_bin_perception = LaunchConfiguration(
        "enable_sparse_bin_perception"
    )
    enable_finished_slot_perception = LaunchConfiguration(
        "enable_finished_slot_perception"
    )
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_yaw = LaunchConfiguration("robot_yaw")
    initial_battery_percentage = LaunchConfiguration(
        "initial_battery_percentage"
    )
    randomize_raw_bin = LaunchConfiguration("randomize_raw_bin")
    raw_bin_seed = LaunchConfiguration("raw_bin_seed")
    raw_part_count = LaunchConfiguration("raw_part_count")
    raw_part_perception_timeout = LaunchConfiguration(
        "raw_part_perception_timeout"
    )
    enable_task_queue_runtime = LaunchConfiguration(
        "enable_task_queue_runtime"
    )
    task_queue_dispatch_enabled = LaunchConfiguration(
        "task_queue_dispatch_enabled"
    )
    task_queue_path = LaunchConfiguration("task_queue_path")
    physical_orders_enabled = IfCondition(
        PythonExpression([
            "'", use_navigation, "' == 'true' and '",
            use_moveit, "' == 'true'",
        ])
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_navigation",
                default_value="false",
                description="Start Nav2, docking and AprilTag nodes",
            ),
            DeclareLaunchArgument(
                "use_moveit",
                default_value="true",
                description="Start MoveIt and the manipulation action server",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run only the Gazebo server (useful for CI and tests)",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=PathJoinSubstitution([
                    FindPackageShare("mobile_manipulator_description"),
                    "worlds",
                    "factory.sdf",
                ]),
                description="SDF world file passed to Gazebo",
            ),
            DeclareLaunchArgument(
                "enable_perception",
                default_value="true",
                description="Enable LiDAR and RGB-D sensors",
            ),
            DeclareLaunchArgument(
                "enable_sparse_bin_perception",
                default_value="true",
                description="Use RGB-D candidates for raw-bin grasp targets",
            ),
            DeclareLaunchArgument(
                "enable_finished_slot_perception",
                default_value="true",
                description=(
                    "Require three fresh RGB-D occupancy frames before "
                    "placing into a finished-bin slot"
                ),
            ),
            DeclareLaunchArgument(
                "randomize_raw_bin",
                default_value="false",
                description="Randomize separated raw-part poses at startup",
            ),
            DeclareLaunchArgument(
                "raw_bin_seed",
                default_value="7",
                description="Repeatable sparse-bin randomization seed",
            ),
            DeclareLaunchArgument(
                "raw_part_count",
                default_value="6",
                description="Active raw workpieces provisioned at startup (1-6)",
            ),
            DeclareLaunchArgument(
                "raw_part_perception_timeout",
                default_value="8.0",
                description=(
                    "Maximum wait for three fresh target observations under "
                    "full navigation and rendering load"
                ),
            ),
            DeclareLaunchArgument(
                "robot_x",
                default_value="0.0",
                description="Initial robot X coordinate in the Gazebo world",
            ),
            DeclareLaunchArgument(
                "robot_y",
                default_value="-1.2",
                description="Initial robot Y coordinate in the Gazebo world",
            ),
            DeclareLaunchArgument(
                "robot_yaw",
                default_value="0.0",
                description="Initial robot yaw in radians",
            ),
            DeclareLaunchArgument(
                "initial_battery_percentage",
                default_value="0.42",
                description="Initial physical battery state in the range 0 to 1",
            ),
            DeclareLaunchArgument(
                "physics_engine",
                # DART is validated for both the physical differential base
                # and the loop-free two-joint parallel gripper.
                default_value="gz-physics-dartsim-plugin",
                description="Gazebo physics plugin used by the factory simulation",
            ),
            DeclareLaunchArgument(
                "enable_task_queue_runtime",
                default_value="false",
                description=(
                    "Observe machine states and persist generated robot tasks"
                ),
            ),
            DeclareLaunchArgument(
                "task_queue_dispatch_enabled",
                default_value="false",
                description=(
                    "Dispatch persisted tasks through the physical executor. "
                    "Do not submit ExecuteOrder goals at the same time."
                ),
            ),
            DeclareLaunchArgument(
                "task_queue_path",
                default_value="~/.ros/factory_robot_tasks.json",
                description="Persistent machine-event task queue JSON path",
            ),
            _include(
                "factory_bringup",
                "gazebo.launch.py",
                launch_arguments={
                    "headless": LaunchConfiguration("headless"),
                    "enable_perception": enable_perception,
                    "world_file": world_file,
                    "robot_x": robot_x,
                    "robot_y": robot_y,
                    "robot_yaw": robot_yaw,
                    "initial_battery_percentage": initial_battery_percentage,
                    "physics_engine": LaunchConfiguration("physics_engine"),
                    "enable_semantic_order_execution": "false",
                    "raw_part_count": raw_part_count,
                }.items(),
            ),
            Node(
                package="factory_core",
                executable="machine_task_queue",
                name="machine_task_queue",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "queue_path": task_queue_path,
                        "planned_quantity": ParameterValue(
                            raw_part_count, value_type=int
                        ),
                        "allow_loading": True,
                        "dispatch_enabled": ParameterValue(
                            task_queue_dispatch_enabled, value_type=bool
                        ),
                    }
                ],
                condition=IfCondition(enable_task_queue_runtime),
            ),
            _include(
                "factory_moveit_config",
                "move_group.launch.py",
                condition=IfCondition(use_moveit),
            ),
            Node(
                package="factory_core",
                executable="manipulate_part_server",
                name="factory_manipulate_part",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "use_finished_slot_perception": ParameterValue(
                            enable_finished_slot_perception, value_type=bool
                        ),
                        "use_raw_part_perception": ParameterValue(
                            enable_sparse_bin_perception, value_type=bool
                        ),
                        "use_machine_part_perception": ParameterValue(
                            enable_sparse_bin_perception, value_type=bool
                        ),
                        "raw_part_perception_timeout": ParameterValue(
                            raw_part_perception_timeout,
                            value_type=float,
                        ),
                    }
                ],
                condition=IfCondition(use_moveit),
            ),
            Node(
                package="factory_perception",
                executable="sparse_bin_detector",
                name="sparse_bin_detector",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "upright_cylinder_radius": 0.025,
                        "supported_center_height": 0.254,
                        "cylinder_radius_tolerance": 0.010,
                    }
                ],
                condition=IfCondition(enable_sparse_bin_perception),
            ),
            Node(
                package="factory_perception",
                executable="sparse_bin_detector",
                name="sparse_bin_detector_aux",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "output_topic": (
                            "/perception/raw_part_candidates_aux"
                        ),
                        "camera_info_topic": "/camera_aux/camera_info",
                        "depth_topic": "/camera_aux/depth/image_raw",
                        "upright_cylinder_radius": 0.025,
                        "supported_center_height": 0.254,
                        "cylinder_radius_tolerance": 0.010,
                    }
                ],
                condition=IfCondition(enable_sparse_bin_perception),
            ),
            Node(
                package="factory_perception",
                executable="sparse_bin_detector",
                name="machine_fixture_detector",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "output_topic": (
                            "/perception/machine_part_candidates"
                        ),
                        # Exclude the 0.88 m fixture top while retaining the
                        # upright 120 mm workpiece above it. Coordinates are
                        # expressed in base_link after CNC visual alignment.
                        "region_min": [0.62, -0.22, 0.89],
                        "region_max": [1.00, 0.22, 1.03],
                        "minimum_component_pixels": 15,
                        "maximum_component_span": 0.10,
                        "maximum_candidates": 2,
                        "upright_cylinder_radius": 0.025,
                        "cylinder_radius_tolerance": 0.010,
                    }
                ],
                condition=IfCondition(enable_sparse_bin_perception),
            ),
            Node(
                package="factory_perception",
                executable="finished_slot_detector",
                name="finished_slot_detector",
                output="screen",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(enable_finished_slot_perception),
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="factory_perception",
                        executable="randomize_raw_bin",
                        name="raw_bin_randomizer",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "seed": ParameterValue(
                                    raw_bin_seed, value_type=int
                                ),
                                "active_part_count": ParameterValue(
                                    raw_part_count, value_type=int
                                ),
                                "randomize_positions": ParameterValue(
                                    randomize_raw_bin, value_type=bool
                                ),
                            }
                        ],
                    )
                ],
            ),
            Node(
                package="factory_core",
                executable="physical_order_executor",
                name="physical_order_executor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "use_finished_slot_perception": ParameterValue(
                            enable_finished_slot_perception,
                            value_type=bool,
                        ),
                    }
                ],
                condition=physical_orders_enabled,
            ),
            Node(
                package="factory_task_bt",
                executable="factory_task_bt_executor",
                name="factory_task_bt_executor",
                output="screen",
                parameters=[{"use_sim_time": True}],
                condition=physical_orders_enabled,
            ),
            TimerAction(
                period=10.0,
                condition=IfCondition(use_navigation),
                actions=[
                    _include(
                        "factory_bringup",
                        "navigation.launch.py",
                        launch_arguments={
                            "initial_x": robot_x,
                            "initial_y": robot_y,
                            "initial_yaw": robot_yaw,
                        }.items(),
                    )
                ],
            ),
        ]
    )
