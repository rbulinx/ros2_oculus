import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bridge_launch = os.path.join(
        get_package_share_directory("unity_mavlink_bridge"),
        "launch",
        "unity_mavlink_bridge.launch.py",
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch)),
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="cable_follow_image_view",
            arguments=["/cable_tracking/camera/debug"],
            output="screen",
        ),
    ])
