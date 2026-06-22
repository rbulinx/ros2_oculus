import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    configuration = os.path.join(
        get_package_share_directory("cable_tracker"),
        "config",
        "cable_tracker.yaml",
    )
    return LaunchDescription([
        Node(
            package="cable_tracker",
            executable="cable_tracker_node",
            name="cable_tracker_node",
            output="screen",
            parameters=[configuration],
        ),
    ])
