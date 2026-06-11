from setuptools import find_packages, setup

package_name = "swarm_autonomy_comms"

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
    description="Bandwidth-limited inter-drone comms middleware.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "comms_middleware = swarm_autonomy_comms.comms_middleware:main",
        ],
    },
)
