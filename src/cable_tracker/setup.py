import os
from glob import glob

from setuptools import find_packages, setup


package_name = "cable_tracker"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kis",
    maintainer_email="kis@example.com",
    description="Camera and Oculus sonar fusion node for underwater cable tracking.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cable_tracker_node = cable_tracker.cable_tracker_node:main",
        ],
    },
)
