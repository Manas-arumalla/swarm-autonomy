from setuptools import find_packages, setup

package_name = "swarm_autonomy_mapping"

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
    description="ESDF/occupancy mapping and shared-map merge.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_merge_node = swarm_autonomy_mapping.map_merge_node:main",
        ],
    },
)
