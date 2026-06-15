import os
from glob import glob

from setuptools import find_packages, setup

package_name = "navigation_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="ROS 2 узлы навигации по waypoints (обёртка над navigation_module).",
    entry_points={
        "console_scripts": [
            "waypoint_nav = navigation_ros.waypoint_nav:main",
        ],
    },
)
