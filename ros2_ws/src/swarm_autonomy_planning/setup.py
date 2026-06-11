from setuptools import find_packages, setup

package_name = "swarm_autonomy_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Swarm Autonomy",
    maintainer_email="manasreddyarumalla@gmail.com",
    description="Planner bringup shim integrating ego-planner-swarm and RACER.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "planner_node = swarm_autonomy_planning.planner_node:main",
        ],
    },
)
