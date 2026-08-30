from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    IfElseSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    world_file = LaunchConfiguration("world_file")
    enable_perception = LaunchConfiguration("enable_perception")
    physics_engine = LaunchConfiguration("physics_engine")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_yaw = LaunchConfiguration("robot_yaw")
    initial_battery_percentage = LaunchConfiguration("initial_battery_percentage")
    raw_part_count = LaunchConfiguration("raw_part_count")
    headless_flag = IfElseSubstitution(headless, if_value="-s --headless-rendering", else_value="")
    description_share = FindPackageShare("mobile_manipulator_description")
    bringup_share = FindPackageShare("factory_bringup")
    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "mobile_manipulator.urdf.xacro"]
    )
    default_world_file = PathJoinSubstitution(
        [description_share, "worlds", "factory.sdf"]
    )
    enable_semantic_order_execution = LaunchConfiguration(
        "enable_semantic_order_execution"
    )
    # ParameterValue(value_type=str) stops launch_ros from YAML-parsing the
    # URDF string; without it any "colon plus space" inside the xacro output
    # (comments included) aborts the whole launch.
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " enable_perception:=",
                enable_perception,
            ]),
            value_type=str,
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [
                headless_flag,
                " -r -v 3 --physics-engine ",
                physics_engine,
                " ",
                world_file,
            ],
            # Closing the Gazebo GUI also stops semantic nodes and bridges.
            "on_exit_shutdown": "true",
        }.items(),

    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )
    physics_description_filter = Node(
        package="factory_core",
        executable="physics_description_filter",
        output="screen",
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "factory_robot", "-topic", "gazebo_robot_description",
            "-x", robot_x,
            "-y", robot_y,
            "-z", "0.02",
            "-Y", robot_yaw,
        ],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": PathJoinSubstitution(
                [bringup_share, "config", "gz_bridge.yaml"]
            )
        }],
        output="screen",
    )
    # The arm must be claimed before gravity can move its uncommanded
    # joints outside MoveIt's valid start state. The spawner waits for the
    # Gazebo controller manager, then the remaining controllers start only
    # after arm activation succeeds.
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager-timeout",
            "30",
            "--switch-timeout",
            "15",
        ],
        output="screen",
    )
    remaining_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        controller,
                        "--controller-manager-timeout",
                        "30",
                        "--switch-timeout",
                        "15",
                    ],
                    output="screen",
                )
                for controller in (
                    "joint_state_broadcaster",
                    "base_controller",
                    "gripper_controller",
                )
            ],
        )
    )
    semantic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [bringup_share, "launch", "semantic_demo.launch.py"]
            )
        ),
        launch_arguments={
            "enable_order_execution": enable_semantic_order_execution,
            "raw_part_count": raw_part_count,
            # The physical stack owns its task queue. Keep the semantic demo's
            # optional observer disabled so one process is the sole writer of
            # the persistent queue file.
            "enable_semantic_task_queue_runtime": "false",
        }.items(),
    )
    door_visualizer = Node(
        package="factory_core",
        executable="door_visualizer",
        output="screen",
    )
    physical_battery = Node(
        package="factory_core",
        executable="physical_battery",
        name="physical_battery",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "initial_percentage": ParameterValue(
                    initial_battery_percentage, value_type=float
                ),
            }
        ],
    )
    base_state_estimator = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_state_estimator",
        output="screen",
        parameters=[PathJoinSubstitution([bringup_share, "config", "base_ekf.yaml"])],
        remappings=[("odometry/filtered", "/odometry/filtered")],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo without the graphical client",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=default_world_file,
            description="SDF world file; recording may use a generated copy",
        ),
        DeclareLaunchArgument(
            "enable_perception",
            default_value="true",
            description="Enable LiDAR and RGB-D sensors",
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
            "raw_part_count",
            default_value="6",
            description="Initial semantic and physical raw inventory",
        ),
        DeclareLaunchArgument(
            "enable_semantic_order_execution",
            default_value="true",
            description="Use the fast semantic ExecuteOrder server",
        ),
        DeclareLaunchArgument(
            "physics_engine",
            # DART preserves the articulated arm while the physical drive
            # wheels move the base. The project-owned two-joint parallel gripper
            # has no mimic-chain dependency and is physically verified on DART.
            default_value="gz-physics-dartsim-plugin",
            description="Gazebo physics plugin used by the factory simulation",
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([description_share, "models"]),
        ),
        gazebo,
        state_publisher,
        physics_description_filter,
        spawn_robot,
        bridge,
        base_state_estimator,
        arm_controller,
        remaining_controllers,
        semantic,
        door_visualizer,
        physical_battery,
    ])
