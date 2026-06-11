from setuptools import find_packages, setup

package_name = "swarm_autonomy_coordination"

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
    description="Decentralized role allocation (CBBA) and pursuit geometry.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "coordination_node = swarm_autonomy_coordination.coordination_node:main",
        ],
    },
)
