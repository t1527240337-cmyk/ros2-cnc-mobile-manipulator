from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_order_execution = LaunchConfiguration("enable_order_execution")
    raw_part_count = LaunchConfiguration("raw_part_count")
    semantic_step_period = LaunchConfiguration("semantic_step_period")
    automatic_max_batch_size = LaunchConfiguration(
        "automatic_max_batch_size"
    )
    enable_semantic_task_queue_runtime = LaunchConfiguration(
        "enable_semantic_task_queue_runtime"
    )
    task_queue_path = LaunchConfiguration("task_queue_path")
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_order_execution",
            default_value="true",
            description="Let the semantic runtime own ExecuteOrder and ControlTask",
        ),
        DeclareLaunchArgument(
            "semantic_step_period",
            default_value="0.15",
            description="Wall-clock delay between semantic scheduling steps",
        ),
        DeclareLaunchArgument(
            "automatic_max_batch_size",
            default_value="4",
            description="Largest batch dispatched by automatic production",
        ),
        DeclareLaunchArgument(
            "raw_part_count",
            default_value="6",
            description="Initial raw inventory supplied by scene provisioning",
        ),
        DeclareLaunchArgument(
            "enable_semantic_task_queue_runtime",
            default_value="false",
            description="Persist event-generated tasks without dispatching motion",
        ),
        DeclareLaunchArgument(
            "task_queue_path",
            default_value="~/.ros/factory_robot_tasks.json",
            description="Persistent machine-event task queue JSON path",
        ),
        Node(
            package="factory_core",
            executable="factory_runtime",
            name="factory_runtime",
            output="screen",
            parameters=[{
                "raw_part_count": ParameterValue(
                    raw_part_count, value_type=int
                ),
                "initial_battery": 0.42,
                "machine_cycle_seconds": 12.0,
                "step_period": ParameterValue(
                    semantic_step_period, value_type=float
                ),
                "enable_order_execution": ParameterValue(
                    enable_order_execution, value_type=bool
                ),
            }],
        ),
        Node(
            package="factory_agent",
            executable="factory_agent_node",
            name="factory_agent",
            output="screen",
        ),
        Node(
            package="factory_core",
            executable="automatic_production_coordinator",
            name="automatic_production_coordinator",
            output="screen",
            parameters=[{
                "maximum_batch_size": ParameterValue(
                    automatic_max_batch_size, value_type=int
                ),
            }],
        ),
        Node(
            package="factory_core",
            executable="machine_task_queue",
            name="machine_task_queue",
            output="screen",
            parameters=[{
                "queue_path": task_queue_path,
                "allow_loading": True,
            }],
            condition=IfCondition(enable_semantic_task_queue_runtime),
        ),
    ])
