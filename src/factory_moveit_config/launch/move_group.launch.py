from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (
        MoveItConfigsBuilder("factory_mobile_manipulator", package_name="factory_moveit_config")
        .robot_description(
            file_path="config/mobile_manipulator.urdf.xacro",
            mappings={"use_sim": "true"},
        )
        .robot_description_semantic(file_path="config/factory_mobile_manipulator.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    parameters = config.to_dict()
    # Normalize adapter parameters across MoveIt config utility releases.
    for key in ("request_adapters", "response_adapters"):
        if isinstance(parameters["ompl"].get(key), str):
            parameters["ompl"][key] = parameters["ompl"][key].split()
    if "planning_plugin" in parameters["ompl"]:
        parameters["ompl"]["planning_plugins"] = [
            parameters["ompl"].pop("planning_plugin")
        ]

    simulation_execution = {
        "use_sim_time": True,
        # MoveIt plans in simulation time but monitors controllers in wall
        # time. Headless Gazebo often runs below real time on CI / WSL.
        "trajectory_execution.allowed_execution_duration_scaling": 5.0,
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
    }

    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[parameters, simulation_execution],
        )
    ])
