from setuptools import find_packages, setup

package_name = "factory_core"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            [
                "config/stations.yaml",
                "config/manipulation.yaml",
                "config/routes.yaml",
            ],
        ),
        (
            "share/" + package_name + "/behavior_trees",
            [
                "behavior_trees/navigate_through_factory_route.xml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Factory Robot Maintainer",
    maintainer_email="maintainer@example.com",
    description="Deterministic factory mission core and ROS adapters",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "factory_runtime = factory_core.ros_runtime:main",
        "factory_demo = factory_core.demo:main",
        "door_visualizer = factory_core.door_visualizer:main",
        "move_arm = factory_core.move_arm:main",
        "move_pose = factory_core.move_pose:main",
        "initial_pose_publisher = factory_core.initial_pose_publisher:main",
        "navigate_station = factory_core.navigate_station:main",
        "navigate_route = factory_core.navigate_route:main",
        "manipulate_part_server = factory_core.manipulate_part_server:main",
        "check_base_kinematics = factory_core.check_base_kinematics:main",
        "dock_station = factory_core.dock_station:main",
        "undock_station = factory_core.undock_station:main",
        "twist_stamper = factory_core.twist_stamper:main",
        "twist_relay = factory_core.twist_relay:main",
        "physical_battery = factory_core.physical_battery:main",
        "physical_order_executor = factory_core.physical_order_executor:main",
        "physical_seed_campaign = factory_core.physical_campaign:main",
        "automatic_production_coordinator = factory_core.automatic_production_coordinator:main",
        "machine_task_queue = factory_core.machine_task_queue_node:main",
        "physics_description_filter = factory_core.physics_description_filter:main",
    ]},
)
