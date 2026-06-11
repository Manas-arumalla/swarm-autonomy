from setuptools import find_packages, setup

package_name = "swarm_autonomy_perception"

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
    description="VIO bringup, VIO->EKF2 bridge, target detection.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vio_to_ekf2_bridge = swarm_autonomy_perception.vio_to_ekf2_bridge:main",
            "target_detector = swarm_autonomy_perception.target_detector:main",
        ],
    },
)
