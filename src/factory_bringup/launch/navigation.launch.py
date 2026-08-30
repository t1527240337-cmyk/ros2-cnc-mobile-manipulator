from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = FindPackageShare("factory_bringup")
    station_navigation_tree = PathJoinSubstitution(
        [share, "behavior_trees", "navigate_to_station_staging.xml"]
    )
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    configured_params = RewrittenYaml(
        source_file=PathJoinSubstitution(
            [share, "config", "nav2_params.yaml"]
        ),
        param_rewrites={
            "amcl.ros__parameters.initial_pose.x": initial_x,
            "amcl.ros__parameters.initial_pose.y": initial_y,
            "amcl.ros__parameters.initial_pose.yaw": initial_yaw,
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml":
                station_navigation_tree,
            "docking_server.ros__parameters.navigator_bt_xml":
                station_navigation_tree,
        },
        convert_types=True,
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
            )
        ),
        launch_arguments={
            "map": PathJoinSubstitution([share, "maps", "factory_map.yaml"]),
            "params_file": configured_params,
            "use_sim_time": "true",
            "autostart": "true",
            # Nav2 otherwise starts more than a dozen Python-managed OS
            # processes. Composable nodes reduce DDS traffic and leave the
            # Gazebo physics thread enough CPU to advance predictably.
            "use_composition": "True",
        }.items(),
    )
    velocity_adapter = Node(
        package="factory_core",
        executable="twist_stamper",
        name="cmd_vel_stamper",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    docking_velocity_relay = Node(
        package="factory_core",
        executable="twist_relay",
        name="dock_cmd_vel_relay",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    laser_filter = Node(
        package="factory_bringup",
        executable="laser_self_filter",
        name="laser_self_filter",
        output="screen",
        parameters=[
            PathJoinSubstitution([share, "config", "laser_filters.yaml"]),
            {"use_sim_time": True},
        ],
    )
    # The lower RGB-D camera retains responsibility for tray and dock tags.
    bin_tags = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        parameters=[
            PathJoinSubstitution([share, "config", "apriltags.yaml"]),
            {"use_sim_time": True},
        ],
        remappings=[
            ("image_rect", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
        ],
    )
    # A second, physically supported high camera keeps CNC tags visible while
    # the arm carries stock.  Splitting tag IDs also prevents two TF sources
    # from publishing the same child frame.
    machine_tags = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="machine_apriltag",
        output="screen",
        parameters=[
            PathJoinSubstitution([share, "config", "apriltags.yaml"]),
            {"use_sim_time": True},
        ],
        remappings=[
            ("image_rect", "/tag_camera/image_raw"),
            ("camera_info", "/tag_camera/camera_info"),
        ],
    )
    dock_pose = Node(
        package="factory_perception",
        executable="dock_pose_from_tag",
        name="dock_pose_from_tag",
        output="screen",
        parameters=[{"use_sim_time": True, "maximum_detection_age": 12.0}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "initial_x",
                default_value="0.0",
                description="Initial robot X coordinate used by AMCL",
            ),
            DeclareLaunchArgument(
                "initial_y",
                default_value="-1.2",
                description="Initial robot Y coordinate used by AMCL",
            ),
            DeclareLaunchArgument(
                "initial_yaw",
                default_value="0.0",
                description="Initial robot yaw used by AMCL",
            ),
            laser_filter,
            nav2,
            docking_velocity_relay,
            velocity_adapter,
            bin_tags,
            machine_tags,
            dock_pose,
        ]
    )
