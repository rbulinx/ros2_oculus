import os
from glob import glob

from setuptools import find_packages, setup


package_name = "unity_mavlink_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
    ],
    install_requires=["setuptools", "pymavlink>=2.4.40"],
    zip_safe=True,
    maintainer="kis",
    maintainer_email="kis@example.com",
    description="Safe ROS 2 Twist to Unity MAVLink bridge and cable follower.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "unity_mavlink_bridge = unity_mavlink_bridge.bridge_node:main",
            "cable_follow_controller = unity_mavlink_bridge.follow_controller_node:main",
        ],
    },
)
