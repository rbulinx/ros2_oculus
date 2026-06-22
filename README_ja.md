# ros2_oculus

Oculus 互換ソナーパケットを受信し、ROS 2 トピックへ変換するための ROS 2 Humble ワークスペースです。

このリポジトリには `oculus_bridge` と `cable_tracker` パッケージが含まれています。Unity 製 Oculus エミュレータや Oculus 互換データソースへ接続し、ソナー ping パケットを decode するほか、カメラ画像とソナーを融合してケーブルを追跡します。

英語版: [README.md](/home/kis/ros2_oculus/README.md)

## パッケージ構成

- `src/oculus_bridge`
- `src/cable_tracker`

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
- `reconnect_interval_sec`（TCP切断時の再接続間隔）
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
- `point_cloud_invert_bearing`

`point_cloud_invert_bearing` は、Oculus の右正方位を ROS の左正（REP-103）へ
合わせるため、点群生成時に bearing の符号を反転します。既定値は `true` です。

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

## カメラ・ソナー融合ケーブルトラッキング

`cable_tracker` は次の入力を購読します。

- `/rov/camera/image/compressed`: Unity カメラの JPEG 画像
- `/oculus/ping/image`: ソナー強度画像
- `/oculus/ping/bearings`: beam 方位
- `/oculus/ping/metadata`: range resolution などの ping 情報

起動:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch cable_tracker cable_tracker.launch.py
```

主な出力:

- `/cable_tracking/camera/debug`: ケーブル中心線を描画した画像
- `/cable_tracking/sonar/debug`: 永続性フィルタと選択ターゲットの画像
- `/cable_tracking/sonar/points_filtered`: 点滅ノイズを除去した点群
- `/cable_tracking/target`: `oculus_sonar` 座標系のターゲット位置
- `/cable_tracking/state`: `FUSED_TRACK`、`CAMERA_ONLY`、`SONAR_ONLY`、`LOST`

既定値は緑色ケーブル向けです。HSV、カメラ水平 FOV、カメラ・ソナー間の yaw、ソナー強度しきい値は `cable_tracker.yaml` で調整してください。

## Unity MAVLink 操作と自動追従

`unity_mavlink_bridge` は `/rov/cmd_vel` (`geometry_msgs/msg/Twist`) の最新値だけを
Unity の UDP 14550 番ポートへ MAVLink `MANUAL_CONTROL` として 20 Hz で送信します。
0.5 秒間指令がなければゼロ指令へ切り替え、終了時にもゼロ指令を10回送ります。

`cable_follow_controller` は `/cable_tracking/state` を使い、次の制御を行います。

- `linear.x`: ソナー距離を2.0 mに維持
- `linear.y`: 左右移動でカメラ上のターゲットを中央へ合わせる
- `angular.z`: 常に0（Unity側のヘディングホールドへ干渉しない）
- `linear.z`: `/rov/manual_cmd_vel.linear.z` の手動値をそのまま使用
- ターゲットロスト時: surge、sway、yawを即時停止

起動:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch unity_mavlink_bridge unity_mavlink_bridge.launch.py
```

手動heave入力例（潜航）:

```bash
ros2 topic pub -r 10 /rov/manual_cmd_vel geometry_msgs/msg/Twist \
  "{linear: {z: -0.3}}"
```

controllerだけを起動して、MAVLink送信前に `/rov/cmd_vel` を確認することもできます。

```bash
ros2 launch unity_mavlink_bridge cable_follow_controller.launch.py
ros2 topic echo /rov/cmd_vel
```

Oculus、認識、距離制御、MAVLinkをまとめて起動する場合:

```bash
ros2 launch unity_mavlink_bridge cable_follow_system.launch.py
```

画像ビューアはこのlaunchから独立しているため、controllerの再起動では閉じません。

## Git

このリポジトリでは、次のローカル生成物を除外しています。

- `build/`
- `install/`
- `log/`
- `.codex`
