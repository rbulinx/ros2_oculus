# ros2_oculus

Oculus 互換ソナーパケットを受信し、ROS 2 トピックへ変換するための ROS 2 Humble ワークスペースです。

このリポジトリには現在 `oculus_bridge` パッケージが含まれており、Unity 製 Oculus エミュレータや Oculus 互換データソースへ接続し、ソナー ping パケットを decode して、画像、メタデータ、bearing、status、点群トピックを publish します。

英語版: [README.md](/home/kis/ros2_oculus/README.md)

## パッケージ構成

- `src/oculus_bridge`

主なソースファイル:

- [src/oculus_bridge/src/oculus_bridge_node.cpp](/home/kis/ros2_oculus/src/oculus_bridge/src/oculus_bridge_node.cpp)
- [src/oculus_bridge/src/oculus_packet_decoder.cpp](/home/kis/ros2_oculus/src/oculus_bridge/src/oculus_packet_decoder.cpp)
- [src/oculus_bridge/include/oculus_bridge/oculus_protocol.hpp](/home/kis/ros2_oculus/src/oculus_bridge/include/oculus_bridge/oculus_protocol.hpp)
- [src/oculus_bridge/launch/oculus_bridge.launch.py](/home/kis/ros2_oculus/src/oculus_bridge/launch/oculus_bridge.launch.py)

## `oculus_bridge` の役割

- `52100` 番ポートの TCP データストリームへ接続
- `52102` 番ポートの UDP status パケットを受信
- `SimpleFire` を送ってソナーの ping 配信を開始
- Oculus `SimplePingResult` パケットを decode
- 下流処理向けの ROS 2 トピックを publish

## publish されるトピック

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

## ビルド方法

```bash
cd /home/kis/ros2_oculus
source /opt/ros/humble/setup.bash
colcon build --packages-select oculus_bridge
```

ビルド後:

```bash
source /opt/ros/humble/setup.bash
source /home/kis/ros2_oculus/install/setup.bash
```

## 起動方法

`sonar_address` に Unity エミュレータ、またはソナーデータソースの IP アドレスを指定します。

```bash
cd /home/kis/ros2_oculus
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run oculus_bridge oculus_bridge_node --ros-args -p sonar_address:=192.168.0.10
```

launch ファイルを使う場合:

```bash
ros2 launch oculus_bridge oculus_bridge.launch.py
```

launch を使う場合は、次のファイル内の `sonar_address` を編集してください。

- [src/oculus_bridge/launch/oculus_bridge.launch.py](/home/kis/ros2_oculus/src/oculus_bridge/launch/oculus_bridge.launch.py)

## よく使うパラメータ

通信:

- `sonar_address`
- `sonar_data_port`
- `status_bind_address`
- `status_udp_port`

Fire 要求:

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

点群:

- `point_cloud_min_intensity`
- `point_cloud_range_stride`
- `point_cloud_beam_stride`
- `point_cloud_frame_id`

## 使用例

### 要求レンジを変更する

```bash
ros2 run oculus_bridge oculus_bridge_node --ros-args \
  -p sonar_address:=192.168.0.10 \
  -p fire_range_percent_or_meters:=25.0
```

### 点群の負荷を下げる

```bash
ros2 run oculus_bridge oculus_bridge_node --ros-args \
  -p sonar_address:=192.168.0.10 \
  -p point_cloud_min_intensity:=48 \
  -p point_cloud_range_stride:=8 \
  -p point_cloud_beam_stride:=4
```

## 補足

- 現在の点群は、ソナー強度画像から生成したもので、LiDAR のような厳密距離点群ではありません。
- SLAM に使う場合は、点群しきい値や stride の調整をおすすめします。
- Raspberry Pi 4 では、点群密度を下げるとかなり扱いやすくなります。

## Git

このリポジトリでは、次のローカル生成物を除外しています。

- `build/`
- `install/`
- `log/`
- `.codex`
