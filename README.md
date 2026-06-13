# ros2_oculus

ROS 2 Humble workspace for receiving Oculus-compatible sonar packets and publishing them as ROS 2 topics.

Japanese version: [README_ja.md](/home/kis/ros2_oculus/README_ja.md)

This repository currently contains the `oculus_bridge` package, which connects to a Unity-based Oculus emulator or other Oculus-compatible data source, decodes sonar ping packets, and publishes image, metadata, bearings, status, and point cloud topics.

## Package

- `src/oculus_bridge`

Main source files:

- [src/oculus_bridge/src/oculus_bridge_node.cpp](/home/kis/ros2_oculus/src/oculus_bridge/src/oculus_bridge_node.cpp)
- [src/oculus_bridge/src/oculus_packet_decoder.cpp](/home/kis/ros2_oculus/src/oculus_bridge/src/oculus_packet_decoder.cpp)
- [src/oculus_bridge/include/oculus_bridge/oculus_protocol.hpp](/home/kis/ros2_oculus/src/oculus_bridge/include/oculus_bridge/oculus_protocol.hpp)
- [src/oculus_bridge/launch/oculus_bridge.launch.py](/home/kis/ros2_oculus/src/oculus_bridge/launch/oculus_bridge.launch.py)

## What `oculus_bridge` does

- Connects to sonar TCP data stream on port `52100`
- Listens for sonar UDP status packets on port `52102`
- Sends `SimpleFire` requests to start sonar ping streaming
- Decodes Oculus `SimplePingResult` packets
- Publishes ROS 2 topics for downstream processing

## Published topics

- `/oculus/ping/raw_packet`
  `std_msgs/msg/UInt8MultiArray`
- `/oculus/ping/image`
  `sensor_msgs/msg/Image`
- `/oculus/ping/bearings`
  `std_msgs/msg/Int16MultiArray`
- `/oculus/ping/metadata`
  `std_msgs/msg/String`
- `/oculus/ping/points`
  `sensor_msgs/msg/PointCloud2`
- `/oculus/status`
  `std_msgs/msg/String`

## Build

```bash
cd /home/kis/ros2_oculus
source /opt/ros/humble/setup.bash
colcon build --packages-select oculus_bridge
```

After build:

```bash
source /opt/ros/humble/setup.bash
source /home/kis/ros2_oculus/install/setup.bash
```

## Run

Set `sonar_address` to the IP address of the Unity emulator or sonar source.

```bash
cd /home/kis/ros2_oculus
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run oculus_bridge oculus_bridge_node --ros-args -p sonar_address:=192.168.0.10
```

Launch file:

```bash
ros2 launch oculus_bridge oculus_bridge.launch.py
```

If you use the launch file, edit the `sonar_address` parameter in:

- [src/oculus_bridge/launch/oculus_bridge.launch.py](/home/kis/ros2_oculus/src/oculus_bridge/launch/oculus_bridge.launch.py)

## Common parameters

Communication:

- `sonar_address`
- `sonar_data_port`
- `status_bind_address`
- `status_udp_port`

Fire request:

- `auto_fire`
- `fire_interval_sec`
- `fire_message_version`
- `fire_frequency_mode`
- `fire_ping_rate`
- `fire_network_speed`
- `fire_gamma_correction`
- `fire_flags`
- `fire_range_percent_or_meters`
- `fire_gain_percent`
- `fire_speed_of_sound`
- `fire_salinity`

Point cloud:

- `point_cloud_min_intensity`
- `point_cloud_range_stride`
- `point_cloud_beam_stride`
- `point_cloud_frame_id`

## Examples

### Change requested range

```bash
ros2 run oculus_bridge oculus_bridge_node --ros-args \
  -p sonar_address:=192.168.0.10 \
  -p fire_range_percent_or_meters:=25.0
```

### Reduce point cloud load

```bash
ros2 run oculus_bridge oculus_bridge_node --ros-args \
  -p sonar_address:=192.168.0.10 \
  -p point_cloud_min_intensity:=48 \
  -p point_cloud_range_stride:=8 \
  -p point_cloud_beam_stride:=4
```

## Notes

- The current point cloud is generated from sonar intensity image data, not from a true lidar-like range return.
- For SLAM use, tuning the point cloud threshold and stride parameters is recommended.
- On Raspberry Pi 4, reducing point cloud density usually helps a lot.

## Git

This repository ignores the following local build artifacts:

- `build/`
- `install/`
- `log/`
- `.codex`
