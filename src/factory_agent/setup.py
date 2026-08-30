from setuptools import find_packages, setup

package_name = "factory_agent"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        package_name: [
            "knowledge_base.yaml",
            "web/*.css",
            "web/*.html",
            "web/*.js",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pydantic>=1.10", "PyYAML>=5.4"],
    extras_require={"mcp": ["mcp>=1.27,<2"]},
    zip_safe=True,
    maintainer="Factory Robot Maintainer",
    maintainer_email="maintainer@example.com",
    description="Constrained factory operator agent with auditable SOP retrieval",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "factory_agent_node = factory_agent.ros_node:main",
        "factory_agent_cli = factory_agent.cli:main",
        "factory_mcp_server = factory_agent.mcp_server:main",
        "factory_operator = factory_agent.operator_cli:main",
        "factory_operator_web = factory_agent.operator_web:main",
    ]},
)
