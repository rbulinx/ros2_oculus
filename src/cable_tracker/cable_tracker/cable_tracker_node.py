"""ROS 2 node that fuses a camera cable centerline with persistent sonar returns."""

import json
import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Int16MultiArray, String

from cable_tracker.tracking import (
    CameraTrack,
    SonarPersistenceTracker,
    SonarTrack,
    sonar_bearing_to_image_x,
    track_green_cable,
)


class CableTrackerNode(Node):
    """Track a cable in camera imagery and select its sonar range return."""

    def __init__(self) -> None:
        super().__init__("cable_tracker_node")

        self.camera_topic = self.declare_parameter(
            "camera_topic", "/rov/camera/image/compressed"
        ).value
        self.sonar_image_topic = self.declare_parameter(
            "sonar_image_topic", "/oculus/ping/image"
        ).value
        self.sonar_bearings_topic = self.declare_parameter(
            "sonar_bearings_topic", "/oculus/ping/bearings"
        ).value
        self.sonar_metadata_topic = self.declare_parameter(
            "sonar_metadata_topic", "/oculus/ping/metadata"
        ).value
        self.output_prefix = self.declare_parameter("output_prefix", "/cable_tracking").value
        self.sonar_frame_id = self.declare_parameter("sonar_frame_id", "oculus_sonar").value

        self.hsv_lower = (
            int(self.declare_parameter("camera.hsv_lower_h", 35).value),
            int(self.declare_parameter("camera.hsv_lower_s", 70).value),
            int(self.declare_parameter("camera.hsv_lower_v", 35).value),
        )
        self.hsv_upper = (
            int(self.declare_parameter("camera.hsv_upper_h", 95).value),
            int(self.declare_parameter("camera.hsv_upper_s", 255).value),
            int(self.declare_parameter("camera.hsv_upper_v", 255).value),
        )
        self.camera_roi_top = float(self.declare_parameter("camera.roi_top_fraction", 0.15).value)
        self.camera_row_step = int(self.declare_parameter("camera.row_step", 4).value)
        self.camera_morphology_kernel = int(
            self.declare_parameter("camera.morphology_kernel", 5).value
        )
        self.camera_minimum_rows = int(self.declare_parameter("camera.minimum_rows", 12).value)
        self.camera_lookahead = float(
            self.declare_parameter("camera.lookahead_fraction", 0.80).value
        )
        self.camera_horizontal_fov = math.radians(
            float(self.declare_parameter("camera.horizontal_fov_deg", 90.0).value)
        )
        self.camera_minimum_confidence = float(
            self.declare_parameter("camera.minimum_confidence", 0.20).value
        )
        self.camera_timeout_sec = float(self.declare_parameter("camera.timeout_sec", 0.5).value)
        self.camera_to_sonar_yaw = math.radians(
            float(self.declare_parameter("camera_to_sonar_yaw_deg", 0.0).value)
        )

        self.sonar_tracker = SonarPersistenceTracker(
            history_length=int(self.declare_parameter("sonar.history_length", 5).value),
            required_hits=int(self.declare_parameter("sonar.required_hits", 3).value),
            intensity_threshold=float(
                self.declare_parameter("sonar.intensity_threshold", 0.25).value
            ),
            morphology_kernel=int(self.declare_parameter("sonar.morphology_kernel", 3).value),
            minimum_cluster_pixels=int(
                self.declare_parameter("sonar.minimum_cluster_pixels", 8).value
            ),
            bearing_gate_rad=math.radians(
                float(self.declare_parameter("sonar.camera_bearing_gate_deg", 12.0).value)
            ),
            invert_bearing=bool(self.declare_parameter("sonar.invert_bearing", True).value),
        )
        self.point_cloud_stride = max(
            1, int(self.declare_parameter("sonar.point_cloud_stride", 2).value)
        )
        self.sonar_overlay_timeout_sec = float(
            self.declare_parameter("sonar.overlay_timeout_sec", 1.0).value
        )

        self.latest_camera_track: Optional[CameraTrack] = None
        self.latest_camera_time_ns = 0
        self.latest_sonar_track: Optional[SonarTrack] = None
        self.latest_sonar_time_ns = 0
        self.latest_bearings = np.empty(0, dtype=np.int16)
        self.latest_metadata = {}
        self.warned_missing_sonar_context = False

        self.camera_debug_pub = self.create_publisher(
            Image, f"{self.output_prefix}/camera/debug", 10
        )
        self.camera_mask_pub = self.create_publisher(
            Image, f"{self.output_prefix}/camera/mask", 10
        )
        self.camera_observation_pub = self.create_publisher(
            Float32MultiArray, f"{self.output_prefix}/camera/observation", 10
        )
        self.sonar_debug_pub = self.create_publisher(
            Image, f"{self.output_prefix}/sonar/debug", 10
        )
        self.sonar_observation_pub = self.create_publisher(
            Float32MultiArray, f"{self.output_prefix}/sonar/observation", 10
        )
        self.filtered_cloud_pub = self.create_publisher(
            PointCloud2, f"{self.output_prefix}/sonar/points_filtered", 10
        )
        self.target_point_pub = self.create_publisher(
            PointStamped, f"{self.output_prefix}/target", 10
        )
        self.tracking_state_pub = self.create_publisher(
            String, f"{self.output_prefix}/state", 10
        )

        self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self.camera_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.sonar_image_topic,
            self.sonar_image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Int16MultiArray,
            self.sonar_bearings_topic,
            self.sonar_bearings_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            self.sonar_metadata_topic,
            self.sonar_metadata_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Cable tracker started: camera={self.camera_topic}, sonar={self.sonar_image_topic}"
        )

    def camera_callback(self, message: CompressedImage) -> None:
        encoded = np.frombuffer(bytes(message.data), dtype=np.uint8)
        bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warning("Failed to decode the compressed camera image.")
            return

        track = track_green_cable(
            bgr=bgr,
            hsv_lower=self.hsv_lower,
            hsv_upper=self.hsv_upper,
            roi_top_fraction=self.camera_roi_top,
            row_step=self.camera_row_step,
            morphology_kernel=self.camera_morphology_kernel,
            minimum_rows=self.camera_minimum_rows,
            lookahead_fraction=self.camera_lookahead,
            horizontal_fov_rad=self.camera_horizontal_fov,
        )
        self.latest_camera_track = track
        self.latest_camera_time_ns = self.get_clock().now().nanoseconds

        debug = bgr.copy()
        if track.detected:
            polyline = np.rint(track.centerline).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(debug, [polyline], False, (0, 0, 255), 3, cv2.LINE_AA)
            lookahead_y = int(np.clip(self.camera_lookahead, 0.0, 1.0) * (bgr.shape[0] - 1))
            lookahead_index = int(
                np.argmin(np.abs(track.centerline[:, 1] - lookahead_y))
            )
            target = tuple(np.rint(track.centerline[lookahead_index]).astype(int))
            cv2.circle(debug, target, 8, (255, 0, 255), -1, cv2.LINE_AA)
            text = (
                f"lateral={track.lateral_error:+.2f} "
                f"heading={math.degrees(track.heading_error_rad):+.1f}deg "
                f"conf={track.confidence:.2f}"
            )
            cv2.putText(debug, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(debug, "CABLE LOST", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 0, 255), 2, cv2.LINE_AA)

        self._draw_sonar_overlay(debug, track)

        self.camera_debug_pub.publish(self._bgr_image_message(debug, message.header))
        self.camera_mask_pub.publish(self._mono_image_message(track.mask, message.header))
        observation = Float32MultiArray()
        observation.data = [
            float(track.lateral_error),
            float(track.heading_error_rad),
            float(track.bearing_rad),
            float(track.confidence),
        ]
        self.camera_observation_pub.publish(observation)

    def sonar_bearings_callback(self, message: Int16MultiArray) -> None:
        self.latest_bearings = np.asarray(message.data, dtype=np.int16)

    def sonar_metadata_callback(self, message: String) -> None:
        try:
            self.latest_metadata = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning("Received invalid Oculus metadata JSON.")

    def sonar_image_callback(self, message: Image) -> None:
        intensity = self._sonar_image_array(message)
        if intensity is None:
            return

        range_resolution = float(self.latest_metadata.get("range_resolution", 0.0))
        if self.latest_bearings.size != message.width or range_resolution <= 0.0:
            if not self.warned_missing_sonar_context:
                self.get_logger().warning(
                    "Waiting for matching Oculus bearings and range_resolution metadata."
                )
                self.warned_missing_sonar_context = True
            return
        self.warned_missing_sonar_context = False

        camera_track = self._fresh_camera_track()
        camera_bearing = None
        if camera_track is not None and camera_track.confidence >= self.camera_minimum_confidence:
            # Camera image +x is right; ROS sonar bearing +y is left.
            camera_bearing = self.camera_to_sonar_yaw - camera_track.bearing_rad

        sonar_track = self.sonar_tracker.update(
            intensity=intensity,
            bearings_centideg=self.latest_bearings,
            range_resolution=range_resolution,
            camera_bearing_rad=camera_bearing,
        )
        self.latest_sonar_track = sonar_track
        self.latest_sonar_time_ns = self.get_clock().now().nanoseconds

        self.sonar_debug_pub.publish(
            self._sonar_debug_message(intensity, sonar_track, message.header)
        )
        self.filtered_cloud_pub.publish(
            self._filtered_cloud_message(
                message,
                intensity,
                sonar_track.stable_mask,
                range_resolution,
            )
        )

        observation = Float32MultiArray()
        observation.data = [
            float(sonar_track.range_m),
            float(sonar_track.bearing_rad),
            float(sonar_track.confidence),
            float(sonar_track.intensity),
        ]
        self.sonar_observation_pub.publish(observation)

        if sonar_track.detected:
            point = PointStamped()
            point.header = message.header
            point.header.frame_id = self.sonar_frame_id
            point.point.x = sonar_track.range_m * math.cos(sonar_track.bearing_rad)
            point.point.y = sonar_track.range_m * math.sin(sonar_track.bearing_rad)
            point.point.z = 0.0
            self.target_point_pub.publish(point)

        self._publish_state(camera_track, sonar_track, message)

    def _fresh_camera_track(self) -> Optional[CameraTrack]:
        if self.latest_camera_track is None:
            return None
        age_sec = (
            self.get_clock().now().nanoseconds - self.latest_camera_time_ns
        ) / 1.0e9
        if age_sec > self.camera_timeout_sec or not self.latest_camera_track.detected:
            return None
        return self.latest_camera_track

    def _fresh_sonar_track(self) -> Optional[SonarTrack]:
        if self.latest_sonar_track is None or not self.latest_sonar_track.detected:
            return None
        age_sec = (
            self.get_clock().now().nanoseconds - self.latest_sonar_time_ns
        ) / 1.0e9
        if age_sec > self.sonar_overlay_timeout_sec:
            return None
        return self.latest_sonar_track

    def _draw_sonar_overlay(self, image: np.ndarray, camera_track: CameraTrack) -> None:
        sonar_track = self._fresh_sonar_track()
        if sonar_track is None:
            cv2.putText(
                image,
                "SONAR: NO TARGET",
                (20, 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 120, 255),
                2,
                cv2.LINE_AA,
            )
            return

        projected_x, visible = sonar_bearing_to_image_x(
            sonar_bearing_rad=sonar_track.bearing_rad,
            camera_to_sonar_yaw_rad=self.camera_to_sonar_yaw,
            horizontal_fov_rad=self.camera_horizontal_fov,
            image_width=image.shape[1],
        )
        label = (
            f"SONAR range={sonar_track.range_m:.2f}m "
            f"bearing={math.degrees(sonar_track.bearing_rad):+.1f}deg "
            f"conf={sonar_track.confidence:.2f}"
        )
        cv2.putText(
            image,
            label,
            (20, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if not visible:
            cv2.putText(
                image,
                "SONAR REFLECTION OUTSIDE CAMERA FOV",
                (20, 98),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 120, 255),
                2,
                cv2.LINE_AA,
            )
            return

        x = int(round(projected_x))
        if camera_track.detected and camera_track.centerline.size:
            index = int(np.argmin(np.abs(camera_track.centerline[:, 0] - projected_x)))
            y = int(round(float(camera_track.centerline[index, 1])))
        else:
            # Oculus gives range and horizontal bearing, but not elevation.
            y = image.shape[0] // 2

        cv2.line(image, (x, 0), (x, image.shape[0] - 1), (0, 255, 255), 1, cv2.LINE_AA)
        cv2.drawMarker(
            image,
            (x, y),
            (0, 255, 255),
            cv2.MARKER_CROSS,
            28,
            3,
            cv2.LINE_AA,
        )
        cv2.circle(image, (x, y), 14, (0, 255, 255), 2, cv2.LINE_AA)

    def _publish_state(
        self,
        camera_track: Optional[CameraTrack],
        sonar_track: SonarTrack,
        sonar_message: Image,
    ) -> None:
        camera_detected = camera_track is not None
        if camera_detected and sonar_track.detected:
            state = "FUSED_TRACK"
        elif camera_detected:
            state = "CAMERA_ONLY"
        elif sonar_track.detected:
            state = "SONAR_ONLY"
        else:
            state = "LOST"

        payload = {
            "state": state,
            "stamp": {
                "sec": sonar_message.header.stamp.sec,
                "nanosec": sonar_message.header.stamp.nanosec,
            },
            "camera": {
                "detected": camera_detected,
                "lateral_error": camera_track.lateral_error if camera_detected else None,
                "heading_error_deg": (
                    math.degrees(camera_track.heading_error_rad) if camera_detected else None
                ),
                "bearing_deg": (
                    math.degrees(self.camera_to_sonar_yaw - camera_track.bearing_rad)
                    if camera_detected else None
                ),
                "confidence": camera_track.confidence if camera_detected else 0.0,
            },
            "sonar": {
                "detected": sonar_track.detected,
                "range_m": sonar_track.range_m if sonar_track.detected else None,
                "bearing_deg": (
                    math.degrees(sonar_track.bearing_rad) if sonar_track.detected else None
                ),
                "confidence": sonar_track.confidence if sonar_track.detected else 0.0,
            },
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        self.tracking_state_pub.publish(message)

    def _sonar_image_array(self, message: Image) -> Optional[np.ndarray]:
        if message.encoding == "mono8":
            dtype = np.uint8
            bytes_per_pixel = 1
        elif message.encoding == "mono16":
            dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
            bytes_per_pixel = 2
        else:
            self.get_logger().warning(f"Unsupported sonar encoding: {message.encoding}")
            return None

        row_elements = message.step // bytes_per_pixel
        expected = int(message.height) * row_elements
        array = np.frombuffer(bytes(message.data), dtype=dtype, count=expected)
        array = array.reshape((message.height, row_elements))[:, : message.width]
        return np.asarray(array, dtype=np.uint16 if bytes_per_pixel == 2 else np.uint8)

    def _filtered_cloud_message(
        self,
        source: Image,
        intensity: np.ndarray,
        stable_mask: np.ndarray,
        range_resolution: float,
    ) -> PointCloud2:
        ranges, beams = np.nonzero(stable_mask)
        if self.point_cloud_stride > 1:
            ranges = ranges[:: self.point_cloud_stride]
            beams = beams[:: self.point_cloud_stride]

        if ranges.size:
            bearing = np.deg2rad(
                self.latest_bearings[beams].astype(np.float64) / 100.0
            )
            if self.sonar_tracker.invert_bearing:
                bearing = -bearing
            radius = (ranges.astype(np.float64) + 0.5) * range_resolution
            cloud_data = np.empty(
                ranges.size,
                dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")],
            )
            cloud_data["x"] = radius * np.cos(bearing)
            cloud_data["y"] = radius * np.sin(bearing)
            cloud_data["z"] = 0.0
            cloud_data["intensity"] = intensity[ranges, beams].astype(np.float32)
        else:
            cloud_data = np.empty(
                0,
                dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")],
            )

        message = PointCloud2()
        message.header = source.header
        message.header.frame_id = self.sonar_frame_id
        message.height = 1
        message.width = int(cloud_data.size)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 16
        message.row_step = message.point_step * message.width
        message.data = cloud_data.tobytes()
        message.is_dense = True
        return message

    def _sonar_debug_message(
        self,
        intensity: np.ndarray,
        track: SonarTrack,
        header,
    ) -> Image:
        if intensity.dtype == np.uint16:
            grayscale = np.right_shift(intensity, 8).astype(np.uint8)
        else:
            grayscale = intensity.astype(np.uint8)
        debug = cv2.applyColorMap(grayscale, cv2.COLORMAP_TURBO)
        debug[track.stable_mask == 0] = (debug[track.stable_mask == 0] * 0.25).astype(np.uint8)
        if track.detected:
            debug[track.target_mask > 0] = (0, 255, 0)
            text = (
                f"range={track.range_m:.2f}m "
                f"bearing={math.degrees(track.bearing_rad):+.1f}deg "
                f"conf={track.confidence:.2f}"
            )
            cv2.putText(debug, text, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1, cv2.LINE_AA)
        return self._bgr_image_message(debug, header)

    @staticmethod
    def _bgr_image_message(image: np.ndarray, header) -> Image:
        message = Image()
        message.header = header
        message.height, message.width = image.shape[:2]
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = image.tobytes()
        return message

    @staticmethod
    def _mono_image_message(image: np.ndarray, header) -> Image:
        message = Image()
        message.header = header
        message.height, message.width = image.shape[:2]
        message.encoding = "mono8"
        message.is_bigendian = False
        message.step = message.width
        message.data = image.astype(np.uint8, copy=False).tobytes()
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CableTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
