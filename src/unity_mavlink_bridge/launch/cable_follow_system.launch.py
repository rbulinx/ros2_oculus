import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    tracker_launch = os.path.join(
        get_package_share_directory("cable_tracker"),
        "launch",
        "cable_tracker.launch.py",
    )
    control_launch = os.path.join(
        get_package_share_directory("unity_mavlink_bridge"),
        "launch",
        "unity_mavlink_bridge.launch.py",
    )
    ros_tcp_endpoint_launch = os.path.join(
        get_package_share_directory("ros_tcp_endpoint"),
        "launch",
        "endpoint.py",
    )
    rviz_config = os.path.join(
        get_package_share_directory("unity_mavlink_bridge"),
        "config",
        "sonar_point_cloud.rviz",
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(ros_tcp_endpoint_launch)),
        Node(
            package="oculus_bridge",
            executable="oculus_bridge_node",
            name="oculus_bridge_node",
            output="screen",
            parameters=[{
                "sonar_address": "192.168.50.177",
                "sonar_data_port": 52100,
                "reconnect_interval_sec": 1.0,
                "auto_fire": True,
                "fire_interval_sec": 1.0,
                "fire_message_version": 2,
                "fire_range_percent_or_meters": 55.0,
            }],
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(tracker_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(control_launch)),
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="cable_follow_image_view",
            arguments=["/cable_tracking/camera/debug"],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="sonar_point_cloud_view",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
