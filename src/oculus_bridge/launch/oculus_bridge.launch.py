from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oculus_bridge',
            executable='oculus_bridge_node',
            name='oculus_bridge_node',
            output='screen',
            parameters=[{
                'sonar_address': '',
                'sonar_data_port': 52100,
                'status_bind_address': '0.0.0.0',
                'status_udp_port': 52102,
                'tcp_receive_buffer_size': 200000,
                'max_status_packet_size': 2048,
                'ping_raw_topic': 'oculus/ping/raw_packet',
                'ping_image_topic': 'oculus/ping/image',
                'ping_bearings_topic': 'oculus/ping/bearings',
                'ping_metadata_topic': 'oculus/ping/metadata',
                'ping_point_cloud_topic': 'oculus/ping/points',
                'status_topic': 'oculus/status',
                'auto_fire': True,
                'fire_interval_sec': 1.0,
                'fire_message_version': 1,
                'fire_frequency_mode': 2,
                'fire_ping_rate': 1,
                'fire_network_speed': 0,
                'fire_gamma_correction': 127,
                'fire_flags': 0,
                'fire_range_percent_or_meters': 20.0,
                'fire_gain_percent': 50.0,
                'fire_speed_of_sound': 1500.0,
                'fire_salinity': 35.0,
                'point_cloud_min_intensity': 32,
                'point_cloud_range_stride': 4,
                'point_cloud_beam_stride': 2,
                'point_cloud_frame_id': 'oculus_sonar',
            }],
        )
    ])
