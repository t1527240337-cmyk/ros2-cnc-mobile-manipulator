from setuptools import find_packages, setup

package_name = "factory_perception"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Factory Robot Maintainer",
    maintainer_email="maintainer@example.com",
    description="RGB-D station and sparse-bin perception",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dock_pose_from_tag = factory_perception.dock_pose_node:main",
            "slot_detector = factory_perception.slot_detector_node:main",
            "finished_slot_detector = factory_perception.finished_slot_detector_node:main",
            "sparse_bin_detector = factory_perception.sparse_bin_detector_node:main",
            "randomize_raw_bin = factory_perception.raw_bin_randomizer:main",
            "record_overview = factory_perception.overview_recorder:main",
        ]
    },
)
