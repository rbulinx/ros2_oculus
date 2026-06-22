import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    configuration = os.path.join(
        get_package_share_directory("unity_mavlink_bridge"),
        "config",
        "unity_mavlink_bridge.yaml",
    )
    return LaunchDescription([
        Node(
            package="unity_mavlink_bridge",
            executable="cable_follow_controller",
            name="cable_follow_controller",
            output="screen",
            parameters=[configuration],
        ),
        Node(
            package="unity_mavlink_bridge",
            executable="unity_mavlink_bridge",
            name="unity_mavlink_bridge",
            output="screen",
            parameters=[configuration],
        ),
    ])
