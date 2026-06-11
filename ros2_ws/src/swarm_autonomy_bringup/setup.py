import os
from glob import glob

from setuptools import find_packages, setup

package_name = "swarm_autonomy_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Swarm Autonomy",
    maintainer_email="manasreddyarumalla@gmail.com",
    description="Launch files, namespacing, and parameters for Swarm Autonomy.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
